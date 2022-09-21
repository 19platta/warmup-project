'''
Finds the largest object visible by lidar and naviagates towards it.
Follows this object at a speed proportional to the distance it is from the object.
'''

from optparse import Values
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

#parameters
LEN_TO_LOOK = 1
NEW_CLUSTER_DIST = .5

def pol2cart(phi, rho):
    ''' 
    Converts polar coordinates to cartesian
    Parameters:
        phi (float) - angle in degrees
        rho (float) - distance in meters
    Returns:
        x (float) - x coordinate in meters
        y (float) - y coordinate in meters
    Source - https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates 
    '''
    x = rho * np.cos(np.deg2rad(phi))
    y = rho * np.sin(np.deg2rad(phi))
    return(x, y)

class Cluster():
    '''
    Stores a 'cluster' - or a set of points at similar distance detected by lidar

    start_angle (int) - lidar angle at which the cluster starts
    end_angle (int) - lidar angle at which the cluster ends
    values ([float]) - array of distances determined at each angle in the cluster
    '''
    def __init__(self, start, end) -> None:
        '''
        Params:
            start (int) - clusterstart index which is converted to angle
            end (int) - cluster end index which is converted to angle
        '''
        self.start_angle = start - 180
        self.end_angle = end - 180
        self.values = []

    def get_midpoint(self):
        '''
        Returns the angle at the center of the cluster as a float
        '''
        return (self.start_angle + self.end_angle) // 2

    def set_end_point(self, idx):
        '''
        Updates the end point of the cluster given an idx (int)
        '''
        self.end_angle = idx - 180
    
    def get_cluster_len(self):
        '''
        Returns the length of the cluster based on the angle range it encompasses
        '''
        return self.end_angle - self.start_angle

    def get_cluster_avg(self):
        '''
        Returns the average distance of cluster as a float in meters.
        If cluster is empty, returns 0
        '''
        if self.values:
            return mean(self.values)
        return 0.0

class NeatoTrajectory(Node):
    '''
    Controls neato motion based on lidar scan.
    '''
    def __init__(self):
        super().__init__('send_velocity_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        # Initialize angle we want to track and distance we are going to
        self.angle_to_go = 0 
        self.person_dist = 0 
        # Create publisher and subscribers
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vis_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_laser, 10)


    def publish_marker(self,x,y):
        '''
        Publish visualization of person location.
        Parameters:
            x (float) - person x location relative to neato in meters
            y (float) - person y location relative to neato in meters
        '''
        marker = Marker()
        marker.header.frame_id = "base_link"
        # marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
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
        '''
        Handle person following logic. Determines angular and linear speeds.
        '''
        #calculate angular speed as proportional to the square root of the goal angle
        angular_speed = .074*math.sqrt(abs(self.angle_to_go))
        #add direction back in
        if self.angle_to_go < 0:
            angular_speed *= -1

        #calculate linear speed as inversely proportional to the goal angle
        if self.angle_to_go != 0:
            linear_speed = abs(1.0/self.angle_to_go)
        else:
            linear_speed = 1.0
                
        #set speeds based on aformentioned calculations
        linear = Vector3(x=linear_speed,y=0.0,z=0.0)
        angular = Vector3(x=0.0,y=0.0,z=angular_speed)

        cmd_vel = Twist(linear=linear,angular=angular)
        self.publisher.publish(cmd_vel)

        #publish person position
        x,y = pol2cart(self.angle_to_go, self.person_dist)
        print('x',x,'y',y,'theta',self.angle_to_go,'person', self.person_dist)
        self.publish_marker(x,y)

    def get_laser(self,msg):
        '''
        Takes in laser data from neato and determines angle_to_go and person_distance
        To do this, finds largest cluster of laser points at a distance > 0 and identifies it as the target
        '''
        # shift ranges to be centered around 0 so we don't break up clusters in front of us
        shifted_ranges = msg.ranges[180:]
        shifted_ranges.extend(msg.ranges[:180])

        #initialize cluster array
        clusters = [Cluster(0, 0)]
        # loop through lidar angles and find clusters
        for curr_idx in range(LEN_TO_LOOK + 1, len(msg.ranges) - LEN_TO_LOOK):
            #look at a rolling average to account for any gaps in our data
            last_avg = mean(shifted_ranges[curr_idx - LEN_TO_LOOK - 1 : curr_idx + LEN_TO_LOOK])
            curr_avg = mean(shifted_ranges[curr_idx - LEN_TO_LOOK : curr_idx + LEN_TO_LOOK + 1])
            #remove any infs from our data
            if last_avg == float('inf'):
                last_avg = 0
            if curr_avg == float('inf'):
                curr_avg = 0
            #if the last average and current average are not close make a new cluster
            if abs(curr_avg - last_avg) > NEW_CLUSTER_DIST :
                clusters.append(Cluster(curr_idx, curr_idx))
            #if the clusters are at similar distances, extend the current cluster
            elif curr_avg > .1:
                clusters[-1].set_end_point(curr_idx)
                clusters[-1].values.append(shifted_ranges[curr_idx])
        
        # do largest cluster calculation here
        clusters.sort(key=Cluster.get_cluster_avg, reverse=True)
        for cluster in clusters:
            # if most of the values are larger than 0, cluster is valid
            if cluster.get_cluster_avg() > 0.1:
                #set angle of target and distance of target accordingly
                self.angle_to_go = cluster.get_midpoint()
                self.person_dist = cluster.get_cluster_avg()
                break


def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = NeatoTrajectory()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()