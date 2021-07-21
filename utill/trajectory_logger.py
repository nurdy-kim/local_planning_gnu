#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

import rospy
from visualization_msgs.msg import Marker, MarkerArray

class logger:
    def __init__(self):
        self.wp = np.genfromtxt('/home/nurdy/f1tenth_ws/src/local_planning_gnu/map/wp_vegas.csv',delimiter=',')
        self.tr = np.genfromtxt('/home/nurdy/f1tenth_ws/src/local_planning_gnu/utill/trajectory.csv',delimiter=',')

        self.wp_list = self.wp[:,:2]
        self.tr_list = self.tr[:,:2]
        self.pub1 = rospy.Publisher("waypoint_array", MarkerArray,queue_size=1)
        self.pub2 = rospy.Publisher("trajectory_array",MarkerArray, queue_size=1)
        self.wpArray = MarkerArray()
        self.trArray = MarkerArray()
        
        self.marking_wp()

        self.tr_idx_current = 0
        self.nearest_dist = 0
    
    def marking_wp(self):
        for i in range(len(self.wp_list)):
            # Waypoints Marking
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "mpc"
            marker.id = i
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.pose.position.x = self.wp_list[i][0]
            marker.pose.position.y = self.wp_list[i][1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.wpArray.markers.append(marker)
        
        for j in range(len(self.tr_list)):
            # Trajectory Marking
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "mpc"
            marker.id = j
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.pose.position.x = self.tr_list[j][0]
            marker.pose.position.y = self.tr_list[j][1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            self.trArray.markers.append(marker)
    
    def calc_theta(self):
        # Waypoints has no theta, so calculation theta using arctan2
        theta = []
        for i in range(len(self.wp)-1):
            _dx = self.wp[i,0] - self.wp[i+1,0]
            _dy = self.wp[i,1] - self.wp[i+1,1]
            theta.append(np.arctan2(_dy,_dx))
        
        _dx = self.wp[-1,0] - self.wp[0,0]
        _dy = self.wp[-1,1] - self.wp[0,1]
        theta.append(np.arctan2(_dy,_dx))

        return theta
    
    def calc_FR(self):
        # To Calculation FR 
        N = len(self.wp)
        theta_i = self.tr[:,2]
        theta_v = self.calc_theta()

        div_u = 0

        for i in range(N-1):
            self.find_nearest_on_trajectory(i)
            tr_idx = self.tr_idx_current

            div_u += np.fabs(theta_v[i] - theta_i[tr_idx])
        
        FR = div_u / ((N-1)*180)
        return FR
    
    def get_distance(self,x,y):
        dx = x[0] - y[1]
        dy = x[1] - y[1]
        
        return np.sqrt(dx**2 + dy**2)

    def find_nearest_on_trajectory(self,idx):
        # Waypoint and Trajectory is not matching. So Find nearest point on trajectory.
        point = self.wp_list[idx]
        print(point)
        
        idx_tmp = self.tr_idx_current
        self.nearest_dist = self.get_distance(self.tr_list[idx_tmp], point)

        while True:
            idx_tmp += 1
            if idx_tmp >= len(self.wp_list) -1:
                idx_tmp = 0
            
            tmp_dist = self.get_distance(self.tr_list[idx_tmp],point)

            if tmp_dist < self.nearest_dist:
                self.nearest_dist = tmp_dist
                self.tr_idx_current = idx_tmp
                # print(1)
            elif tmp_dist > self.nearest_dist or idx_tmp == self.tr_idx_current:
                # print(2)
                break
        

    def run(self):
        while not rospy.is_shutdown():
            self.pub1.publish(self.wpArray)
            self.pub2.publish(self.trArray)
            FR = self.calc_FR()
            print(FR)
            rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node("logging")
    A = logger()
    A.run()
    rospy.spin()