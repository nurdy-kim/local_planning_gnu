#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

import rospy
from visualization_msgs.msg import Marker, MarkerArray

class logger:
    def __init__(self):
        self.wp = np.genfromtxt(rospy.get_param('wpt_path'),rospy.get_param('wpt_delimeter'))
        self.tr = np.genfromtxt(rospy.get_param('trj_path'),rospy.get_param('wpt_delimeter'))

        self.wp_list = self.wp[:,:2]
        self.tr_list = self.tr[:,1:3]
        self.pub1 = rospy.Publisher("waypoint_array", MarkerArray,queue_size=1)
        self.pub2 = rospy.Publisher("trajectory_array",MarkerArray, queue_size=1)
        self.wpArray = MarkerArray()
        self.trArray = MarkerArray()
        
        self.marking_wp()

        self.tr_idx_current = 0
        self.nearest_dist = 0

        self.weighted_RMS_k = 1.4

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
        theta_i = self.tr[:,3]
        theta_v = self.calc_theta()

        div_u = 0

        for i in range(N-1):
            self.find_nearest_on_trajectory(i)
            tr_idx = self.tr_idx_current
            # print(i, tr_idx)
            div_u += np.fabs(theta_v[i] - theta_i[tr_idx])
        
        FR = div_u / ((N-1)*180)

        return FR
    
    def calc_DR(self):
        di_list = self.tr[:,4]
        dv_list = self.tr[:,5]
        N = len(di_list)
        sum_dr = 0
        for i in range(N):
            tmp = di_list[i]/(N*dv_list[i])
            sum_dr += tmp
        
        return sum_dr
    
    def get_distance(self,x,y):
        dx = x[0] - y[0]
        dy = x[1] - y[1]
        
        return np.sqrt(dx**2 + dy**2)

    def find_nearest_on_trajectory(self,idx):
        # Waypoint and Trajectory is not matching. So Find nearest point on trajectory.
        point = self.wp_list[idx]
        
        idx_tmp = self.tr_idx_current
        self.nearest_dist = self.get_distance(self.tr_list[idx_tmp], point)

        while True:
            idx_tmp += 1
            if idx_tmp >= len(self.tr_list) -1:
                idx_tmp = 0
            
            tmp_dist = self.get_distance(self.tr_list[idx_tmp],point)

            if tmp_dist < self.nearest_dist:
                self.nearest_dist = tmp_dist
                self.tr_idx_current = idx_tmp
                # print(1)
            elif (tmp_dist > self.nearest_dist) or (idx_tmp == self.tr_idx_current):
                # print(2)
                break
    
    def calc_comfort(self):
        comfort_score = 0
        t = self.tr[:,0]
        acc = self.calc_acc()
        ax = []
        ay = []
        for i in range(len(acc)-1):
            d_time = t[i+1] - t[i]
            d_vertical = acc[i+1][0] - acc[i][0] 
            d_horizontal = acc[i+1][1] - acc[i][1]
            
            ax.append(d_vertical / d_time)
            ay.append(d_horizontal / d_time)

        awx = np.mean(ax)
        awy = np.mean(ay)

        final_aw = np.sqrt(self.weighted_RMS_k**2 * awx + self.weighted_RMS_k**2 * awy)
        
        if final_aw < 0.315:
            comfort_score = 10
        elif final_aw < 0.5:
            comfort_score = 8
        elif final_aw <0.63:
            comfort_score = 7
        elif final_aw <0.8:
            comfort_score = 6
        elif final_aw <1:
            comfort_score = 5
        elif final_aw <1.25:
            comfort_score = 4
        elif final_aw <1.6:
            comfort_score = 3
        elif final_aw <2.5:
            comfort_score = 2
        elif final_aw > 2.5:
            comfort_score = 0
        
        return comfort_score
    
    def calc_acc(self):
        v = self.tr[:,6]
        theta = self.tr[:,3]
        acc = []

        for i in range(len(v)-1):
            v1 = v[i]
            v2 = v[i+1]
            theta1 = theta[i]
            theta2 = theta[i+1]

            v_vertical = (np.fabs(v1)*np.fabs(v2)*np.cos(theta2-theta1))
            v_horizontal = (np.fabs(v1)*np.fabs(v2)*np.sin(theta2-theta1))

            acc.append([v_vertical,v_horizontal])
        
        print(len(acc))
        return acc

    
    def run(self):
        while not rospy.is_shutdown():
            self.pub1.publish(self.wpArray)
            self.pub2.publish(self.trArray)
            FR = self.calc_FR()
            DR = self.calc_DR()
            Comfort = self.calc_comfort()
            print("Safety metric :",(1-FR)*DR)
            print("Comfort : ", Comfort)

            rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node("logging")
    A = logger()
    A.run()
    rospy.spin()