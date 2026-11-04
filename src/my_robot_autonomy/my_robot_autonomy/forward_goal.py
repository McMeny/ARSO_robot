"""Keeps the robot driving forward by always giving Nav2 a goal a few metres ahead.

The robot has a travel direction. Every second a new goal is placed straight ahead
along it (a "carrot" that moves with the robot), so on a clear run the path stays
a straight line and the robot never slows down for a goal.

The travel direction only changes when something WIDE (a wall, the end of an aisle)
is coming up ahead. Then it turns a little at a time (turn_step every update) towards
the more open side, so the robot makes a smooth, progressive curve instead of a sharp
turn. Small clutter (a pallet, a cart) doesn't change the direction - Nav2 just plans
around it.

Uses: TF map -> base_footprint, /global_costmap/costmap, and the navigate_to_pose action.
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

LETHAL = 99          # costmap value for "obstacle / robot would touch it"
NEAR_OBSTACLE = 50   # robot centre this close (~0.25 m) to an obstacle - too close
GOAL_MAX_COST = 30   # goals must be this clear of obstacles (~0.45 m away)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ForwardGoal(Node):
    def __init__(self):
        super().__init__('forward_goal')
        self.goal_distance = self.declare_parameter('goal_distance', 3.0).value
        # start turning when a wall is this far ahead
        self.turn_trigger = self.declare_parameter('turn_trigger', 2.0).value
        # how much the travel direction turns per update (2 updates a second)
        self.turn_step = math.radians(self.declare_parameter('turn_step_deg', 12.0).value)
        self.stuck_time = self.declare_parameter('stuck_time', 15.0).value
        self.frame = self.declare_parameter('global_frame', 'map').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.marker_pub = self.create_publisher(MarkerArray, 'walker_markers', 10)

        self.costmap = None
        costmap_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.on_costmap, costmap_qos)

        self.heading = None        # direction the robot is trying to travel in (map frame)
        self.turn_side = 0         # +1 turning left, -1 turning right, 0 going straight
        self.goal_id = 0           # so results of old, replaced goals are ignored
        self.goal_failed = False
        self.last_send_time = -math.inf
        self.progress_pose = None  # where the robot was when it last made progress
        self.progress_time = None

        self.create_timer(0.5, self.update)
        self.get_logger().info('Waiting for Nav2 (navigate_to_pose), the global costmap and the map -> base_footprint TF...')

    # ---------- helpers ----------

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_costmap(self, msg):
        self.costmap = msg

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def cost_at(self, x, y):
        """Costmap value at (x, y): 0 free .. 100 obstacle, -1 unknown or off the map."""
        if self.costmap is None:
            return -1
        info = self.costmap.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height):
            return -1
        return self.costmap.data[row * info.width + col]

    def free_distance(self, x, y, heading, max_distance, threshold, side=0.0):
        """How far you can go from (x, y) along heading before hitting a costmap cell
        >= threshold. side shifts the line sideways (metres, + = left)."""
        x += -math.sin(heading) * side
        y += math.cos(heading) * side
        step = 0.05
        d = 0.0
        while d < max_distance:
            if self.cost_at(x + d * math.cos(heading), y + d * math.sin(heading)) >= threshold:
                return d
            d += step
        return max_distance

    def wall_ahead(self, x, y, heading, distance):
        """True only if something WIDE is within distance - three parallel lines
        (centre and 0.35 m to each side) all hit an obstacle. A pallet or cart blocks
        one or two lines and Nav2 drives around it; a wall or shelf end blocks all three."""
        return all(self.free_distance(x, y, heading, distance, LETHAL, side) < distance
                   for side in (-0.35, 0.0, 0.35))

    def openness(self, x, y, heading):
        """How open a direction is - free distance looking that way, and a bit either side."""
        return sum(self.free_distance(x, y, heading + math.radians(a), 6.0, NEAR_OBSTACLE)
                   for a in (-20, 0, 20))

    # ---------- choosing the direction and the goal ----------

    def update_heading(self, x, y):
        if not self.wall_ahead(x, y, self.heading, self.turn_trigger):
            self.turn_side = 0
            return
        if self.turn_side == 0:
            # pick a side once and stick with it until the turn is done (no zig-zag)
            left = self.openness(x, y, self.heading + math.radians(60))
            right = self.openness(x, y, self.heading - math.radians(60))
            self.turn_side = 1 if left > right else -1
            self.get_logger().info(f'Wall ahead - turning {"left" if self.turn_side > 0 else "right"}')
        self.heading = wrap(self.heading + self.turn_side * self.turn_step)

    def pick_goal(self, x, y):
        """A goal straight ahead along the travel direction, as far as is clear (up to
        goal_distance). If that spot is inside clutter, try slightly to either side."""
        ahead = self.free_distance(x, y, self.heading, self.goal_distance, NEAR_OBSTACLE)
        distances = [self.goal_distance] if ahead >= self.goal_distance else [max(ahead - 0.3, 1.0)]
        distances += [d for d in (2.0, 1.5, 1.0) if d < distances[0]]
        for d in distances:
            for side in (0.0, 0.4, -0.4, 0.8, -0.8):
                gx = x + d * math.cos(self.heading) - side * math.sin(self.heading)
                gy = y + d * math.sin(self.heading) + side * math.cos(self.heading)
                if 0 <= self.cost_at(gx, gy) <= GOAL_MAX_COST or self.cost_at(gx, gy) == -1:
                    return gx, gy
        return None

    def send_goal(self, gx, gy):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(self.heading / 2)
        goal.pose.pose.orientation.w = math.cos(self.heading / 2)

        self.goal_id += 1
        goal_id = self.goal_id
        self.goal_failed = False
        self.last_send_time = self.now()
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self.on_goal_response(f, goal_id))

        # purple arrow in RViz where the goal is
        arrow = Marker()
        arrow.header = goal.pose.header
        arrow.ns, arrow.id = 'forward_goal', 0
        arrow.type, arrow.action = Marker.ARROW, Marker.ADD
        arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = gx, gy, 0.05
        arrow.pose.orientation = goal.pose.pose.orientation
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.6, 0.12, 0.12
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.7, 0.3, 1.0, 1.0
        self.marker_pub.publish(MarkerArray(markers=[arrow]))

    def on_goal_response(self, future, goal_id):
        handle = future.result()
        if not handle.accepted:
            # e.g. Nav2 still starting up - just send again next update, don't turn
            self.last_send_time = -math.inf
            return
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, goal_id))

    def on_result(self, future, goal_id):
        # A goal we replaced on purpose also ends here - only the current one matters
        if goal_id == self.goal_id and future.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn('Nav2 could not reach the goal')
            self.goal_failed = True

    # ---------- main loop ----------

    def update(self):
        pose = self.robot_pose()
        if pose is None or self.costmap is None or not self.nav.server_is_ready():
            return
        x, y, yaw = pose
        now = self.now()
        if self.heading is None:
            self.heading = yaw
            self.progress_pose, self.progress_time = (x, y), now
            self.get_logger().info('Nav2 is ready - driving forward')

        # stuck (or Nav2 gave up) -> make a bigger turn towards the more open side
        if math.hypot(x - self.progress_pose[0], y - self.progress_pose[1]) > 0.3:
            self.progress_pose, self.progress_time = (x, y), now
        if self.goal_failed or now - self.progress_time > self.stuck_time:
            left = self.openness(x, y, self.heading + math.radians(90))
            right = self.openness(x, y, self.heading - math.radians(90))
            self.heading = wrap(self.heading + math.radians(45 if left > right else -45))
            self.turn_side = 0
            self.progress_pose, self.progress_time = (x, y), now
            self.get_logger().warn('Stuck - turning 45 deg towards the open side')

        self.update_heading(x, y)

        # a fresh goal every second, so the carrot stays ahead of the robot
        if now - self.last_send_time < 1.0 and not self.goal_failed:
            return
        new_goal = self.pick_goal(x, y)
        if new_goal is None:
            self.get_logger().warn('No free spot ahead - waiting')
            return
        self.send_goal(*new_goal)


def main():
    rclpy.init()
    node = ForwardGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
