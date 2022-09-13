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

import math
 
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
        self.start_orientation = euler_from_quaternion(Quaternion(w=0.0, x=0.0, y=0.0, z=0.0))
        self.position = Point(x=0.0, y=0.0, z=0.0)
        self.orientation = euler_from_quaternion(Quaternion(w=0.0, x=0.0, y=0.0, z=0.0))

    def get_Odom(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = euler_from_quaternion(msg.pose.pose.orientation)
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
        if abs(self.start_orientation.z - self.orientation.z) >= math.pi/2:
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

def euler_from_quaternion(quaternion):
        """
        From https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/ 
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return Vector3(x=roll_x, y=pitch_y, z=yaw_z) # in radians

def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = Square()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()