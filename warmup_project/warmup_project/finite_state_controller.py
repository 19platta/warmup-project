'''
Attack neato in the style of battle bots. Assume a waiting postition
and upon seeing and enemy (enemy is everything except self), attacks.
It then retreats, having asserted dominance, and continues searching
for opponents.
'''

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
from enum import Enum

class State(Enum):
    SPIN = 1
    ATTACK = 2
    RUNAWAY = 3

ATTACK_DISTANCE = 1.5
ATTACK_HIT_DISTANCE = .5
RUNAWAY_DISTANCE = 2.0


SPIN_ANGULAR_SPEED = .5
SPIN_LINEAR_SPEED = .1
ATTACK_SPEED = 1.0
RUNAWAY_SPEED = -1.0


class AttackNeato(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

        self.state = State.SPIN
        self.target_detected = 0
        self.target_distance = 0

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_laser, 10)


    def run_loop(self):
        linear = Vector3(x=0.0,y=0.0,z=0.0)
        angular = Vector3(x=0.0,y=0.0,z=0.0)

        if self.state == State.SPIN:
            if self.target_detected:
                #switch states if target
                self.state = State.ATTACK
            else:
                #otherwise spin until target acquired
                linear.x = SPIN_LINEAR_SPEED
                angular.z = SPIN_ANGULAR_SPEED

        elif self.state == State.ATTACK:
            if self.target_distance < ATTACK_HIT_DISTANCE:
                #switch state if reached target
                self.state = State.RUNAWAY
            else:
                #otherwise, CHARGE!
                linear.x = ATTACK_SPEED
        else:
            #state = run away
            print(self.target_distance)
            if self.target_distance > RUNAWAY_DISTANCE:
                #if far enough away, return to attack preparation
                self.state = State.SPIN
            else:
                #retreat if too close still
                linear.x = RUNAWAY_SPEED
        
        cmd_vel = Twist(linear=linear,angular=angular)
        self.publisher.publish(cmd_vel)


    def get_laser(self,msg):
        self.target_distance = msg.ranges[0]
        if msg.ranges[0] < ATTACK_DISTANCE:
            self.target_detected = 1
        else:
            self.target_detected = 0



def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = AttackNeato()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()