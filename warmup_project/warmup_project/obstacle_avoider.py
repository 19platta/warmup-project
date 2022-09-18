from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import rclpy
from rclpy.node import Node
import math 

ROTATION_SPEED = -0.5
COLLIDE_DIST = 1.0
MARGIN = 0.1

class SendTwist(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        # flags 
        self.obstacle_detected = 0
        self.turn_flag = 0
        self.rotation_speed = ROTATION_SPEED
        self.turn_count = 0
        self.turn_dir = 1
        # manage subscriptions and publishers
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.get_odom, 10)
        self.laser_subscriber = self.create_subscription(LaserScan, 'scan', self.get_laser, 10)
        # track our starting and current position and orientation
        self.start_position = Point(x=0.0, y=0.0, z=0.0)
        self.start_orientation = euler_from_quaternion(Quaternion(w=0.0, x=0.0, y=0.0, z=0.0))
        self.curr_position = Point(x=0.0, y=0.0, z=0.0)
        self.curr_orientation = euler_from_quaternion(Quaternion(w=0.0, x=0.0, y=0.0, z=0.0))

    def get_odom(self, msg):
        self.curr_position = msg.pose.pose.position
        self.curr_orientation = euler_from_quaternion(msg.pose.pose.orientation)

    def get_laser(self,msg):
        angle = round(-self.curr_orientation.z / math.pi * 180)
        print("angle =", angle, "value = ", msg.ranges[angle])
        if msg.ranges[angle] < COLLIDE_DIST:
            self.obstacle_detected = 1
        else:
            self.obstacle_detected = 0

    def turn_ninety_deg(self):
        if abs(self.start_orientation.z - self.curr_orientation.z) >= math.pi/2:
            self.turn_flag = 0
            self.turn_count += 1
            if self.turn_count % 2 == 1:
                self.turn_dir == 1
            else:
                self.turn_dir == -1
            self.start_orientation = self.curr_orientation
            return Vector3(x=0.0, y=0.0, z=0.0)
        else: 
            return Vector3(x=0.0, y=0.0, z=self.rotation_speed)

    def run_loop(self):
        # if we're heading in the right direction, go until we hit an obstacle, then turn
        print(self.turn_flag)
        if abs(self.curr_orientation.z) < MARGIN:
            if self.obstacle_detected:
                self.turn_flag = 1
                self.rotation_speed = ROTATION_SPEED * self.turn_dir
        # if we're not going in the right direction, we've turned to avoice an obstacle
        # we want to drive until it's gone, then turn back
        else: 
            if not self.obstacle_detected:
                self.turn_flag = 1
                self.rotation_speed = -ROTATION_SPEED * self.turn_dir
        # checks to see if we're turning 
        if self.turn_flag:
            linear = Vector3(x=0.0,y=0.0,z=0.0)
            angular = self.turn_ninety_deg()
        else:
             linear = Vector3(x=0.5,y=0.0,z=0.0)
             angular = Vector3(x=0.0,y=0.0,z=0.0)        
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
    node = SendTwist()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()