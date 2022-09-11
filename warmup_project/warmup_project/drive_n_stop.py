from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from neato2_interfaces.msg import Bump
import rclpy
from rclpy.node import Node

class SendTwist(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.stop_enabled = 0
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(Bump, 'bump', self.get_bump, 10)


    def run_loop(self):
        if not(self.stop_enabled):
            linear = Vector3(x=1.0,y=0.0,z=0.0)
            angular = Vector3(x=0.0,y=0.0,z=0.0)
        else:
            linear = Vector3(x=0.0,y=0.0,z=0.0)
            angular = Vector3(x=0.0,y=0.0,z=0.0)
        cmd_vel = Twist(linear=linear,angular=angular)
        self.publisher.publish(cmd_vel)

    def get_bump(self,msg):
        if msg.left_front == 1 or msg.left_side == 1 or msg.right_front == 1 or msg.right_side == 1:
            self.stop_enabled = 1



def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SendTwist()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()