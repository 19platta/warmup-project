""" This script explores publishing messages in ROS2"""

from turtle import color, stamp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

class SendMessageNode(Node):
    def __init__(self):
        super().__init__('send_message_node')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Marker, 'orb', 10)

    def run_loop(self):
        msg = self.create_marker()
        self.publisher.publish(msg)

    def create_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.0
        marker.pose.position.y = 1.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = SendMessageNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
