'''
Create a sphere visualization in rviz2 at 1,2,0 in the robots coordinate frame.
Code taken from https://comprobo22.github.io/Sample_code/marker_sample after much troubleshooting.
'''

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class VisualizationPublisher(Node):
    def __init__(self):
        super().__init__('test_vis')
        self.vis_pub = self.create_publisher(Marker, 'orb', 10)

        timer_period = 0.2 # seconds
        self.timer = self.create_timer(timer_period, self.publish_marker)
    
    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.0
        marker.pose.position.y = 2.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0 
        marker.color.r = 0.9
        marker.color.g = 0.0
        marker.color.b = 1.0

        self.vis_pub.publish( marker )

def main(args=None):
    rclpy.init(args=args)

    simple_visualization_publisher = VisualizationPublisher()

    rclpy.spin(simple_visualization_publisher)

    #shutdown
    simple_visualization_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
