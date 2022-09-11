from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from neato2_interfaces.msg import Bump
import rclpy
from rclpy.node import Node
import tty
import select
import sys
import termios

settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    print('key -----', key)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class SendTwist(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.stop_enabled = 0
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.key = None
        self.key_updated = 0

    def update_key(self, key):
        key_updated = 1


    def run_loop(self):
        
        linear = Vector3(x=0.0,y=0.0,z=0.0)
        angular = Vector3(x=0.0,y=0.0,z=0.0)
        if self.key_updated:
            if self.key == 'w':
                linear.x = 1.0
            if self.key == 's':
                linear.x = -1.0
            if self.key == 'a':
                angular.z =1.0
            if self.key == 'd':
                angular.z = -1.0
            self.key_updated = 0
        cmd_vel = Twist(linear=linear,angular=angular)
        self.publisher.publish(cmd_vel)





def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SendTwist()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    while key != '\x03':
        print('test test')
        key = getKey()
        print(key)
        node.update_key(key)
    rclpy.shutdown()           # cleanup


if __name__ == '__main__':
    main()