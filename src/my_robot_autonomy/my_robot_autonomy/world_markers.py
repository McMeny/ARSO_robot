"""Draws the Gazebo world (shelves, carts, boxes, walls) in RViz as 3D blocks.

Reads the world .sdf file and publishes every box and cylinder as a marker on
/world_markers - the "ground truth" to compare against what SLAM has mapped.

Drawn in the map frame. SLAM starts its map where the robot spawns, and the robot
spawns at the world origin (0, 0) facing +x, so world and map line up.
"""
import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class WorldMarkers(Node):
    def __init__(self):
        super().__init__('world_markers')
        world_file = self.declare_parameter('world_file', '').value
        self.frame = self.declare_parameter('frame', 'map').value

        # Transient local - RViz gets the markers even if it opens later
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'world_markers', qos)

        self.markers = self.read_world(world_file)
        self.get_logger().info(f'Drawing {len(self.markers.markers)} objects from {world_file}')
        self.pub.publish(self.markers)

    def read_world(self, world_file):
        markers = MarkerArray()
        root = ET.parse(world_file).getroot()
        for i, model in enumerate(root.iter('model')):
            geometry = model.find('.//visual/geometry')
            pose = model.find('pose')
            if geometry is None or pose is None:
                continue  # e.g. the ground plane
            x, y, z, _, _, yaw = (float(v) for v in pose.text.split())

            m = Marker()
            m.header.frame_id = self.frame
            m.ns = 'world'
            m.id = i
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = yaw_to_quaternion(yaw)

            box, cylinder = geometry.find('box'), geometry.find('cylinder')
            if box is not None:
                m.type = Marker.CUBE
                m.scale.x, m.scale.y, m.scale.z = (float(v) for v in box.find('size').text.split())
            elif cylinder is not None:
                m.type = Marker.CYLINDER
                r = float(cylinder.find('radius').text)
                m.scale.x = m.scale.y = 2 * r
                m.scale.z = float(cylinder.find('length').text)
            else:
                continue

            diffuse = model.find('.//visual/material/diffuse')
            r, g, b = (float(v) for v in diffuse.text.split()[:3]) if diffuse is not None else (0.7, 0.7, 0.7)
            m.color.r, m.color.g, m.color.b = r, g, b
            # walls see-through, so they don't hide the robot
            m.color.a = 0.25 if model.get('name', '').startswith('wall_') else 0.9
            markers.markers.append(m)
        return markers


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
