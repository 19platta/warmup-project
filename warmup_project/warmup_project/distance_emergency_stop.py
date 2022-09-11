from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node

class SendTwist(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.turn_enabled = 0
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_laser, 10)


    def run_loop(self):
        if not(self.turn_enabled):
            linear = Vector3(x=0.5,y=0.0,z=0.0)
            angular = Vector3(x=0.0,y=0.0,z=0.0)
        else:
            linear = Vector3(x=0.0,y=0.0,z=0.0)
            angular = Vector3(x=0.0,y=0.0,z=1.0)
        cmd_vel = Twist(linear=linear,angular=angular)
        self.publisher.publish(cmd_vel)

    def get_laser(self,msg):
        if msg.ranges[0] < .50:
            self.turn_enabled = 1
        else:
            self.turn_enabled = 0



def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SendTwist()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()