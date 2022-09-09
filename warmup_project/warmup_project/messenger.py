""" This script explores publishing messages in ROS2"""

from turtle import stamp
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
        self.publisher = self.create_publisher(PointStamped, 'my_point', 10)
    
    def run_loop(self):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id='odom')
        point = Point(x=1, y=2)
        msg = Marker(header=header, point=point)
        self.publisher.publish(msg)
        print('Hi from in_class_day02.')

def main(args=None):
    rclpy.init(args=args)
    node = SendMessageNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
