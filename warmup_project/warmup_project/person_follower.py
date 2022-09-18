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
from visualization_msgs.msg import Marker


import math 
import numpy as np

rolling_avg_len = 3
new_cluster_dist = .5

def pol2cart(phi, rho):
    ''' from - https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates 
    '''
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

class SendTwist(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.angle_to_go = 0
        self.person_dist = 0
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vis_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_laser, 10)


    def publish_marker(self,x,y):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0 
        marker.color.r = 0.9
        marker.color.g = 0.0
        marker.color.b = 1.0

        self.vis_publisher.publish(marker)

    def run_loop(self):

        angular_speed = .074*math.sqrt(abs(self.angle_to_go))
        if self.angle_to_go < 0:
            angular_speed *= -1

        if self.angle_to_go != 0:
            linear_speed = abs(1.0/self.angle_to_go)
        else:
            linear_speed = 1.0
                

        linear = Vector3(x=linear_speed,y=0.0,z=0.0)
        angular = Vector3(x=0.0,y=0.0,z=angular_speed)

        cmd_vel = Twist(linear=linear,angular=angular)
        self.publisher.publish(cmd_vel)

        x,y = pol2cart(self.angle_to_go, self.person_dist)
        print('x',x,'y',y,'theta',self.angle_to_go,'person', self.person_dist)
        self.publish_marker(x,y)

    def get_laser(self,msg):
        #from https://scikit-learn.org/stable/auto_examples/linear_model/plot_ransac.html
        shifted_ranges = msg.ranges[180:]
        shifted_ranges.extend(msg.ranges[:180])

        current_cluster = {'start': rolling_avg_len+1, 'end': rolling_avg_len+1}
        largest_cluster = {'start': rolling_avg_len+1, 'end': rolling_avg_len+1}

        for curr_idx in range((rolling_avg_len//2) + 1, len(msg.ranges)-rolling_avg_len//2):

            last_avg = mean(shifted_ranges[(curr_idx-(rolling_avg_len//2))-1 : (curr_idx+(rolling_avg_len//2))])
            curr_avg = mean(shifted_ranges[(curr_idx-rolling_avg_len//2) : (curr_idx+rolling_avg_len//2) + 1])
            if last_avg == float('inf'):
                last_avg = 0
            if curr_avg == float('inf'):
                curr_avg = 0

            if abs(curr_avg - last_avg) > new_cluster_dist :
                #make new cluster
                current_cluster['start'] = curr_idx-180
                current_cluster['end'] = curr_idx-180
            elif curr_avg > .1:
                current_cluster['end'] = curr_idx-180
                if current_cluster['end'] - current_cluster['start'] > largest_cluster['end'] - largest_cluster['start']:
                    largest_cluster = current_cluster
        self.angle_to_go = mean([largest_cluster['end'],largest_cluster['start']])
        if self.angle_to_go > 360:
            self.angle_to_go = self.angle_to_go - 360
        self.person_dist = shifted_ranges[round(self.angle_to_go)]


def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SendTwist()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()