'''
Ros Node which follows a wall on the left side of the neato
'''

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node

turn_multiplier = 5
angle_from_perp = 10

class SendTwist(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.front_dist = 0
        self.back_dist = 0
        self.offset = .2
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_laser, 10)


    def run_loop(self):
        '''
        Function which gets called repeatedly until node is ended
        Determines neato direction based on the wall to the left
        and publishes velocity accordingly.
        '''
        #only try to turn if we are seeing a wall at both front and back
        if self.front_dist != 0 and self.back_dist != 0:
            turn_angle = float(-1*self.front_dist + self.back_dist)*turn_multiplier
        else:
            turn_angle=0.0

        #set robot to always drive forwards and turn based on wall location
        linear = Vector3(x=0.15,y=0.0,z=0.0)
        angular = Vector3(x=0.0,y=0.0,z=turn_angle)

        #publish velocity
        cmd_vel = Twist(linear=linear,angular=angular)
        self.publisher.publish(cmd_vel)

    def get_laser(self,msg):
        #get laser distance at n degrees in front and behind perpendicular to robot
        self.front_dist = msg.ranges[270+angle_from_perp]
        self.back_dist = msg.ranges[270-angle_from_perp]


def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SendTwist()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()