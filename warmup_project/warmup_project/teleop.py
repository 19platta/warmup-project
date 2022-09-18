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
    ''' 
    Waits until key pressed and returns
    returns: key (char) - key which has been most recently pressed
    '''
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopControl(Node):
    '''
    Node which handles neato motion
    '''
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        # Publisher which sends neato velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Stores key which has been pressed last
        self.key = None


    def run_loop(self):
        '''
        Function which gets called repeatedly until node is ended
        Determines direction to move based on key press and publishes neato velocity

        Uses 'w' to move forward, 's' to move back, 'a' to turn left, and 'd' to turn right
        Press any other key to stop and ctrl-c to end teleop control

        '''
        while self.key != '\x03':
            #set velocity to 0 unless a direction key is pressed
            linear = Vector3(x=0.0,y=0.0,z=0.0)
            angular = Vector3(x=0.0,y=0.0,z=0.0)
            self.key = getKey()
            print(self.key)
            if self.key == 'w':
                linear.x = 1.0
            if self.key == 's':
                linear.x = -1.0
            if self.key == 'a':
                angular.z =1.0
            if self.key == 'd':
                angular.z = -1.0

            #publish velocity
            cmd_vel = Twist(linear=linear,angular=angular)
            self.publisher.publish(cmd_vel)


def main(args=None):
    # Spin up ros node
    rclpy.init(args=args)      # Initialize communication with ROS
    node = TeleopControl()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup


if __name__ == '__main__':
    main()