#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

import rospy
from visualization_msgs.msg import Marker, MarkerArray

class logger:
    def __init__(self):
        wpt_path = rospy.get_param('wpt_path')
        trj_path = rospy.get_param('trj_path')
        wpt_delimeter = rospy.get_param('wpt_delimeter')
        self.wp = np.genfromtxt(wpt_path, delimiter=wpt_delimeter)
        self.tr = np.genfromtxt(trj_path, delimiter=wpt_delimeter)

        self.tr = np.array(self.filtering())
        
        self.wp_list = self.wp[:,:2]
        self.tr_list = self.tr[:,1:3]
        self.pub1 = rospy.Publisher("waypoint_array", MarkerArray,queue_size=1)
        self.pub2 = rospy.Publisher("trajectory_array",MarkerArray, queue_size=1)
        self.wpArray = MarkerArray()
        self.trArray = MarkerArray()
        self.interval = 0.00435
        self.marking_wp()

        self.recording = open('/home/lab/f1tenth_ws/src/local_planning_gnu/utill/recording.csv', 'a')
        self.tr_idx_current = 0
        self.nearest_dist = 0

        self.weighted_RMS_k = 1.4**2
    
    def filtering(self):
        tr = self.tr[:,0]
        filtered_tr = []
        for i in range(len(self.tr)-1):
            if tr[i+1] - tr[i] == 0:
                continue
            else:
                filtered_tr.append(self.tr[i,:])
        return filtered_tr

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
        ax, ay = self.calc_acc()
        _t = np.linspace(0,len(ax))
        
        sum_ax = 0
        sum_ay = 0

        for i in range(len(_t)):
            sum_ax += _t[i] * (ax[i]**2)
            sum_ay += _t[i] * (ay[i]**2)
        
        awx = (1/len(ax)) * sum_ax
        awy = (1/len(ax)) * sum_ay

        aw = np.sqrt(self.weighted_RMS_k * awx + self.weighted_RMS_k * awy)
        # print(aw)
        if aw < 0.315:
            comfort_score = 10
        elif aw < 0.5:
            comfort_score = 8
        elif aw <0.63:
            comfort_score = 7
        elif aw <0.8:
            comfort_score = 6
        elif aw <1:
            comfort_score = 5
        elif aw <1.25:
            comfort_score = 4
        elif aw <1.6:
            comfort_score = 3
        elif aw <2.5:
            comfort_score = 2
        elif aw > 2.5:
            comfort_score = 0
        
        return comfort_score

    
    def calc_acc(self):
        v = self.tr[:,4]
        theta = self.tr[:,3]
        time = self.tr[:,0]

        vertical = []
        horizon = []

        for i in range(len(self.tr)-1):
            _vertical = np.fabs(v[i]) * np.fabs(v[i+1]) * np.cos(theta[i+1]-theta[i])
            _horizon = np.fabs(v[i]) * np.fabs(v[i+1]) * np.sin(theta[i+1]-theta[i])

            vertical.append(_vertical)
            horizon.append(_horizon)
        
        ax = []
        ay = []
        
        for j in range(len(vertical)-1):
            _dt = 1
            _ax = ((vertical[j+1] - vertical[j]) / _dt)
            _ay = ((horizon[j] - horizon[j]) / _dt)

            ax.append(_ax)
            ay.append(_ay)

        return ax, ay

    def run(self):

        while not rospy.is_shutdown():
            self.pub1.publish(self.wpArray)
            self.pub2.publish(self.trArray)
            FR = self.calc_FR()
            Comfort = self.calc_comfort()
            average_speed = np.round(np.average(self.tr[:,4]),3)
            max_speed = np.round(np.max(self.tr[:,4]),3)

            # print("average_speed :",average_speed)
            # print("max_speed :",max_speed)
            # print("Safety metric :",np.round((1-FR),3))
            # print("Comfort : ", Comfort)

            # self.recording.write(f"Safety metric : {np.round((1-FR),3)},Comfort : {Comfort},average : {average_speed},max : {max_speed}\n")

            rospy.sleep(1)


    def write(self):

            FR = self.calc_FR()
            Comfort = self.calc_comfort()
            average_speed = np.round(np.average(self.tr[:,4]),3)
            max_speed = np.round(np.max(self.tr[:,4]),3)

            print("average_speed :",average_speed)
            print("max_speed :",max_speed)
            print("Safety metric :",np.round((1-FR),5))
            print("Comfort : ", Comfort)

            self.recording.write(f"Safety metric : {np.round((1-FR),3)},Comfort : {Comfort},average : {average_speed},max : {max_speed}\n\n")
            

if __name__ == '__main__':
    rospy.init_node("logging")
    A = logger()
    A.run()
    A.write()
    rospy.spin()