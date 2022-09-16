from statistics import mean
from turtle import st
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from statistics import mean
import rclpy
from rclpy.node import Node
import numpy as np
from matplotlib import pyplot as plt

from sklearn import linear_model, datasets

rolling_avg_len = 3
new_cluster_dist = .5

class SendTwist(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.angle_to_go = 0
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_laser, 10)


    def run_loop(self):
        angular_speed = self.angle_to_go/360
        try:
            linear_speed = 1.0/self.angle_to_go
        except:
            linear_speed = 1.0
        if 360-self.angle_to_go < self.angle_to_go:
            try:
                linear_speed = 1.0/(360-self.angle_to_go)
            except:
                linear_speed = 1.0
            angular_speed = -(360-self.angle_to_go)/360
        

        linear = Vector3(x=linear_speed,y=0.0,z=0.0)
        angular = Vector3(x=0.0,y=0.0,z=-3*angular_speed)

        cmd_vel = Twist(linear=linear,angular=angular)
        self.publisher.publish(cmd_vel)

    def get_laser(self,msg):
        #from https://scikit-learn.org/stable/auto_examples/linear_model/plot_ransac.html
        current_cluster = {'start': rolling_avg_len+1, 'end': rolling_avg_len+1}
        largest_cluster = {'start': rolling_avg_len+1, 'end': rolling_avg_len+1}

        for curr_idx in range(rolling_avg_len+1, len(msg.ranges)):
            last_avg = mean(msg.ranges[curr_idx-rolling_avg_len-1:curr_idx-1])
            curr_avg = mean(msg.ranges[curr_idx-rolling_avg_len:curr_idx])

            if abs(curr_avg - last_avg) > new_cluster_dist:
                #make new cluster
                current_cluster['start'] = curr_idx
                current_cluster['end'] = curr_idx
            else:
                current_cluster['end'] = curr_idx
                if current_cluster['end'] - current_cluster['start'] > largest_cluster['end'] - largest_cluster['start']:
                    largest_cluster = current_cluster
        
        self.angle_to_go = mean([largest_cluster['end'],largest_cluster['start'] - 3])


def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SendTwist()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()