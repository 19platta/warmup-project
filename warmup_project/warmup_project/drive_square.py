from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from neato2_interfaces.msg import Bump
import rclpy
import math
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node

SPEED = 1.0
ROTATION_SPEED = -0.5

class Square(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.movement_flag = 1
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, 'odom', self.get_Odom, 10)
        self.side_counter = 0
        self.start_position = Point(x=0.0, y=0.0, z=0.0)
        self.start_orientation = Quaternion(w=0.0, x=0.0, y=0.0, z=0.0)
        self.position = Point(x=0.0, y=0.0, z=0.0)
        self.orientation = Quaternion(w=0.0, x=0.0, y=0.0, z=0.0)

    def get_Odom(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        print("Position =", self.position, " Orientation =", self.orientation)
    
    def drive_one_meter(self):
        # if we've already driven a meter, stop
        if bool(abs(self.position.x - self.start_position.x) >= 1.0) \
        | bool(abs(self.position.y - self.start_position.y) >= 1.0):
            self.movement_flag = 0
            self.side_counter += 1
            self.start_position = self.position
            # we only need to return the linear, angular will always be 0
            return Vector3(x=0.0, y=0.0, z=0.0)
        # if we haven't yet gone a meter, continue
        else:
            return Vector3(x=SPEED, y=0.0, z=0.0)

    def turn_ninety_deg(self):
        if self.start_orientation.z - self.orientation.z >= (math.sqrt(2)/2):
            self.movement_flag = 1
            self.start_orientation = self.orientation
            return Vector3(x=0.0, y=0.0, z=0.0)
        else: 
            return Vector3(x=0.0, y=0.0, z=ROTATION_SPEED)
        

    def run_loop(self):
        if self.movement_flag:
            linear = self.drive_one_meter()
            angular = Vector3(x=0.0,y=0.0,z=0.0)
        else:
            linear = Vector3(x=0.0,y=0.0,z=0.0)
            angular = self.turn_ninety_deg()
        cmd_vel = Twist(linear=linear,angular=angular)
        self.publisher.publish(cmd_vel)



def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = Square()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()