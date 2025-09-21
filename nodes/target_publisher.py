#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.pub = self.create_publisher(Marker, '/target_marker', 10)
        self.timer = self.create_timer(0.1, self.publish_marker)  # 10 Hz

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 2.0  # your target position
        marker.pose.position.y = 3.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3  # sphere size
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
