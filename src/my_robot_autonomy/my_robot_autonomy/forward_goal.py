"""Keeps the walker driving forward by always giving Nav2 a goal a few metres ahead,
in a travel direction shared between the user's head and the machine.

The carrot: every second a new goal is placed straight ahead along the travel direction,
so on a clear run the path stays straight and the walker never slows down for a goal.

Head-guided direction, shared according to risk (risk = clearance, the free length of
the straight tentacle):
  open space (clearance >= open_clearance)  - the user's (filtered) head direction pulls
                                              the travel direction towards it
  obstacles nearby                          - the head's pull fades with clearance; at a
                                              wall it only picks which open way to turn
  collision imminent (<= near_clearance)    - the machine alone decides
Safeguards: head yaw is low-pass filtered (head_filter_time), has a deadband
(head_deadband_deg) and a turn-rate limit (head_turn_rate_deg); looks towards a tagged
hazard are ignored (that's the probe at work, not a choice of direction). Head direction
only ever moves the goal - Nav2 still plans every path around obstacles.

The machine's own turning: when something WIDE (a wall, an aisle end) is within
turn_trigger, it turns turn_step every update towards the more open side (or the side
the user is looking, if that side is open) - a smooth, progressive curve.

Publishes /machine_authority (0 = the head steers, 1 = the machine is fully in charge).
Stops sending goals (and cancels the current one) while /walker_enabled is false.
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
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from walker_interfaces.msg import HazardArray, Tentacles

LETHAL = 99          # costmap value for "obstacle / robot would touch it"
NEAR_OBSTACLE = 50   # robot centre this close (~0.25 m) to an obstacle - too close
GOAL_MAX_COST = 30   # goals must be this clear of obstacles (~0.45 m away)
UPDATE_PERIOD = 0.5  # s


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ForwardGoal(Node):
    def __init__(self):
        super().__init__('forward_goal')
        p = self.declare_parameter
        self.goal_distance = p('goal_distance', 3.0).value
        self.turn_trigger = p('turn_trigger', 2.0).value          # start turning when a wall is this far ahead
        self.turn_step = math.radians(p('turn_step_deg', 12.0).value)  # per update (2 a second)
        self.stuck_time = p('stuck_time', 15.0).value
        self.frame = p('global_frame', 'map').value
        self.enabled = p('start_enabled', True).value
        # head-guided direction
        self.head_filter_time = p('head_filter_time', 1.5).value
        self.head_deadband = math.radians(p('head_deadband_deg', 12.0).value)
        self.head_turn_rate = math.radians(p('head_turn_rate_deg', 15.0).value)
        self.hazard_exclusion = math.radians(p('hazard_look_exclusion_deg', 10.0).value)
        self.open_clearance = p('open_clearance', 2.5).value
        self.near_clearance = p('near_clearance', 0.8).value
        self.head_timeout = p('head_timeout', 1.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.marker_pub = self.create_publisher(MarkerArray, 'walker_markers', 10)
        self.authority_pub = self.create_publisher(Float32, 'machine_authority', 10)

        self.costmap = None
        costmap_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.on_costmap, costmap_qos)
        self.create_subscription(Bool, 'walker_enabled', self.on_enabled, latched)
        self.create_subscription(PoseStamped, 'head_pose', self.on_head, 10)
        self.create_subscription(Tentacles, 'tentacles', lambda m: setattr(self, 'tentacles', m), 10)
        self.create_subscription(HazardArray, 'hazards', lambda m: setattr(self, 'hazards', m.hazards), 10)

        self.tentacles = None
        self.hazards = []
        self.head_x = self.head_y = None
        self.head_filtered = None   # filtered head yaw (map frame)
        self.head_time = -math.inf
        self.head_weight = 0.0      # how much say the head has right now (0..1)

        self.heading = None        # direction the walker is trying to travel in (map frame)
        self.turn_side = 0         # +1 turning left, -1 turning right, 0 going straight
        self.goal_id = 0           # so results of old, replaced goals are ignored
        self.goal_handle = None
        self.goal_failed = False
        self.last_send_time = -math.inf
        self.progress_pose = None  # where the walker was when it last made progress
        self.progress_time = None

        self.create_timer(UPDATE_PERIOD, self.update)
        self.get_logger().info('Waiting for Nav2 (navigate_to_pose), the global costmap and the map -> base_footprint TF...')

    # ---------- helpers ----------

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_costmap(self, msg):
        self.costmap = msg

    def on_enabled(self, msg):
        self.enabled = msg.data
        if not self.enabled and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.progress_time = None  # restart the stuck timer when walking resumes

    def on_head(self, msg):
        """Low-pass filter the head yaw, ignoring looks towards a tagged hazard."""
        q = msg.pose.orientation
        yaw = yaw_of(q)
        hx, hy = msg.pose.position.x, msg.pose.position.y
        now = self.now()
        for h in self.hazards:
            if h.frequency > 0:
                bearing = math.atan2(h.position.y - hy, h.position.x - hx)
                if abs(wrap(bearing - yaw)) < self.hazard_exclusion:
                    self.head_time = now  # still "watching", just not steering
                    return
        dt = min(max(now - self.head_time, 0.0), 1.0) if math.isfinite(self.head_time) else 1.0
        if self.head_filtered is None or not math.isfinite(self.head_time) or now - self.head_time > self.head_timeout:
            self.head_filtered = yaw
        else:
            a = min(dt / self.head_filter_time, 1.0)
            self.head_filtered = wrap(self.head_filtered + a * wrap(yaw - self.head_filtered))
        self.head_x, self.head_y, self.head_time = hx, hy, now

    def head_valid(self):
        return self.head_filtered is not None and self.now() - self.head_time < self.head_timeout

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

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
        d = 0.0
        while d < max_distance:
            if self.cost_at(x + d * math.cos(heading), y + d * math.sin(heading)) >= threshold:
                return d
            d += 0.05
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

    # ---------- shared steering ----------

    def update_authority(self):
        clearance = self.tentacles.clearance if self.tentacles is not None else 0.0
        w = (clearance - self.near_clearance) / (self.open_clearance - self.near_clearance)
        self.head_weight = max(0.0, min(1.0, w)) if self.head_valid() else 0.0
        self.authority_pub.publish(Float32(data=1.0 - self.head_weight))

    def apply_head(self):
        """Pull the travel direction towards where the user is looking, scaled by the head's say."""
        if self.head_weight <= 0.0:
            return
        delta = wrap(self.head_filtered - self.heading)
        if abs(delta) <= self.head_deadband:
            return
        step = min(abs(delta) - self.head_deadband, self.head_turn_rate * UPDATE_PERIOD) * self.head_weight
        self.heading = wrap(self.heading + math.copysign(step, delta))

    def update_heading(self, x, y):
        if not self.wall_ahead(x, y, self.heading, self.turn_trigger):
            self.turn_side = 0
            return
        if self.turn_side == 0:
            left = self.openness(x, y, self.heading + math.radians(60))
            right = self.openness(x, y, self.heading - math.radians(60))
            side = 1 if left > right else -1
            # decision point - if the user is looking towards an open way, take that one
            if self.head_valid():
                look = wrap(self.head_filtered - self.heading)
                if abs(look) > self.head_deadband:
                    wanted = 1 if look > 0 else -1
                    if (left if wanted > 0 else right) > 3.0:  # that way is open enough
                        side = wanted
            self.turn_side = side
            self.get_logger().info(f'Wall ahead - turning {"left" if side > 0 else "right"}')
        self.heading = wrap(self.heading + self.turn_side * self.turn_step)

    # ---------- goals ----------

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
        if goal_id == self.goal_id:
            self.goal_handle = handle
            if not self.enabled:
                handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, goal_id))

    def on_result(self, future, goal_id):
        # A goal we replaced or cancelled on purpose also ends here - only the current one matters
        status = future.result().status
        if goal_id == self.goal_id and self.enabled and status not in (GoalStatus.STATUS_SUCCEEDED,
                                                                       GoalStatus.STATUS_CANCELED):
            self.get_logger().warn('Nav2 could not reach the goal')
            self.goal_failed = True

    # ---------- main loop ----------

    def update(self):
        self.update_authority()
        pose = self.robot_pose()
        if pose is None or self.costmap is None or not self.nav.server_is_ready():
            return
        x, y, yaw = pose
        now = self.now()
        if self.heading is None:
            self.heading = yaw
            self.get_logger().info('Nav2 is ready - driving forward' if self.enabled else
                                   'Nav2 is ready - waiting for Start walking')
        if not self.enabled:
            self.heading = yaw  # resume in whatever direction the walker is facing
            return
        if self.progress_time is None:
            self.progress_pose, self.progress_time = (x, y), now

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

        self.apply_head()
        self.update_heading(x, y)

        # a fresh goal every second, so the carrot stays ahead of the walker
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
