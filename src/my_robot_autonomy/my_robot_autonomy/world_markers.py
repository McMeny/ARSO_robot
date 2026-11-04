"""Draws the Gazebo world (shelves, carts, boxes, walls) in RViz as 3D blocks.

Reads the world .sdf file and publishes every box and cylinder as a marker on
/world_markers - the "ground truth" to compare against what SLAM has mapped.

Drawn in the map frame. SLAM starts its map where the robot spawns, and the robot
spawns at the world origin (0, 0) facing +x, so world and map line up.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from my_robot_autonomy.world_model import load_world


class WorldMarkers(Node):
    def __init__(self):
        super().__init__('world_markers')
        world_file = self.declare_parameter('world_file', '').value
        self.frame = self.declare_parameter('frame', 'map').value

        # Transient local - RViz gets the markers even if it opens later
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'world_markers', qos)

        self.markers = MarkerArray()
        for i, obj in enumerate(load_world(world_file)):
            m = Marker()
            m.header.frame_id = self.frame
            m.ns, m.id, m.action = 'world', i, Marker.ADD
            m.type = Marker.CUBE if obj.kind == 'box' else Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = obj.x, obj.y, obj.z
            m.pose.orientation.z, m.pose.orientation.w = math.sin(obj.yaw / 2), math.cos(obj.yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = obj.size_x, obj.size_y, obj.height
            m.color.r, m.color.g, m.color.b = obj.colour
            # walls see-through, so they don't hide the robot
            m.color.a = 0.25 if obj.name.startswith('wall_') else 0.9
            self.markers.markers.append(m)
        self.get_logger().info(f'Drawing {len(self.markers.markers)} objects from {world_file}')
        self.pub.publish(self.markers)


def main():
    rclpy.init()
    node = WorldMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
