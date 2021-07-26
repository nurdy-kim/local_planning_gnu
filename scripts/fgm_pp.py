#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import multiprocessing
import queue
from xmlrpc.client import FastMarshaller
import rospy
import numpy as np
import threading
import matplotlib.pyplot as plt
from queue import Queue
import math
import time
import csv
#from multiprocessing import Process, Queue

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

class maindrive(threading.Thread):
    def __init__(self, main_q):
        super(maindrive, self).__init__()
        self.main_q = main_q
        self.RATE = 100
        self.ackermann_data = AckermannDriveStamped()
        self.drive_topic = rospy.get_param("drive_topic", "/drive")
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=10)

    def run(self):
        obstacle=False 
        rate = rospy.Rate(self.RATE)

        while True:
            #if self.scan_range == 0: continue
            ackermann = self.main_q.get()
            self.ackermann_data.drive.steering_angle = ackermann[1]
            self.ackermann_data.drive.speed = ackermann[0]
            self.drive_pub.publish(self.ackermann_data)
            rate.sleep()

class global_pure(threading.Thread):
    def __init__(self, global_od_q, main_q):
        super(global_pure, self).__init__()
        #multiprocessing.Process.__init__(self)
        self.global_od_q = global_od_q
        self.main_q = main_q

        self.CURRENT_WP_CHECK_OFFSET = 2
        self.DX_GAIN = 2.5
        self.FILTER_SCALE = 10000

        self.LOOKAHEAD_MAX = rospy.get_param('max_look_ahead', 1.9)
        self.LOOKAHEAD_MIN = rospy.get_param('min_look_ahead', 0.9)
        self.SPEED_MAX = rospy.get_param('max_speed', 20.0)
        self.SPEED_MIN = rospy.get_param('min_speed', 1.5)
        self.MSC_MUXSIZE = rospy.get_param('mux_size', 0)
        self.RACECAR_LENGTH = rospy.get_param('robot_length', 0.325)
        self.ROBOT_SCALE = rospy.get_param('robot_scale', 0.35)
        self.PI = rospy.get_param('pi', 3.141592)
        self.GRAVITY_ACCELERATION = rospy.get_param('g', 9.81)
        self.MU = rospy.get_param('mu', 0.523)
        self.MASS = rospy.get_param('mass', 3.47)
        self.RATE = rospy.get_param('rate', 100)

        self.time_data_file_name = "fgm_pp_time_data"
        self.time_data_path = rospy.get_param("time_data_path", "/home/lab/f1tenth_ws/src/car_duri/recording/fgm_gnu_time_data.csv")
        self.time_data = open(f"{self.time_data_path}/{self.time_data_file_name}.csv", "w", newline="")
        self.time_data_writer = csv.writer(self.time_data)
    
        self.wp_index_current = 0
        self.current_position = [0]*3
        self.lookahead_desired = 0
        self.steering_direction = 0
        self.goal_path_radius = 0
        self.goal_path_theta = 0
        self.actual_lookahead = 0
        self.transformed_desired_point = [0]*3
        self.desired_point = [0]*3
        self.dx = 0
        self.nearest_distance = 0

        self.current_speed = 0.0
        self.speed_past = 0.0

        self.scan_filtered = [0]*1080
        self.manualSpeedArray=[]
        self.idx_save = 0
        
        self.lap_time_flag = True
        self.lap_time_start = 0
        self.lap_time = 0
        self.interval = 0
        self.scan_range = 0

    def run(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            sensor_data = self.global_od_q.get()
            

            self.current_position = [sensor_data[0][0], sensor_data[0][1], sensor_data[0][2]]
            self.current_speed = sensor_data[0][3]
            
            self.interval = sensor_data[1][0]
            self.scan_range = sensor_data[1][1]
            self.scan_filtered = sensor_data[1][2]
            self.desired_wp_rt = sensor_data[2]
            self.actual_lookahead = sensor_data[3]
            
            self.t_loop = sensor_data[4][0]
            self.tn0 = sensor_data[4][1]
            self.tn1 = sensor_data[4][2]

            self.find_path()
            steer  = self.setSteeringAngle()
            # speed = self.setSpeed_PossibleMaximumTest()
            speed = self.speed_controller()

            ackermann = [speed, steer, self.idx_save]
            #print(self.scan_filtered)
            # if self.main_q.full():
            #     self.main_q.get()
            self.main_q.put(ackermann)
            # print("global")
            self.tn2 = time.time()
            self.time_data_writer.writerow([self.t_loop, (self.tn2 - self.tn0), (self.tn2 - self.tn1), "gp"])
            # print("local execution time:", time.time() - self.t1)
            rate.sleep()
    
    def find_path(self):
        #right cornering
        if self.desired_wp_rt[0] > 0:
            self.goal_path_radius = pow(self.actual_lookahead, 2)/(2*self.desired_wp_rt[0])
            self.goal_path_theta = np.arcsin(self.desired_wp_rt[1]/self.goal_path_radius)
            self.steering_direction = -1

        #left cornering
        else:
            self.goal_path_radius = pow(self.actual_lookahead, 2)/((-2)*self.desired_wp_rt[0])
            self.goal_path_theta = np.arcsin(self.desired_wp_rt[1]/self.goal_path_radius)
            self.steering_direction = 1

    def setSteeringAngle(self):
        steering_angle = np.arctan2(self.RACECAR_LENGTH,self.goal_path_radius)
        
        steer = self.steering_direction * steering_angle
        return steer

    # def tuneSteeringAngle(self):
    #     if (self.nearest_distance > 0 and self.transformed_desired_point[2] < (self.PI/2) ) or ( self.nearest_distance < 0 and self.transformed_desired_point[2] > (self.PI/2) ):
    #         print("angle tunning X")

    #     else:
    #         steering_slope = self.PI/2 - 2 * np.arcsin(self.actual_lookahead)
    #         controlled_slope = self.dp_angle_proportion * self.transformed_desired_point[2] + (1 - self.dp_angle_proportion * steering_slope)

    #         self.goal_path_radius = self.actual_lookahead/(2*np.sin(self.PI/4 - controlled_slope/2))

    def speed_controller(self):
        current_distance = np.fabs(np.average(self.scan_filtered[499:580]))
        if np.isnan(current_distance):
            print("SCAN ERR")
            current_distance = 1.0
        
        if self.current_speed > 10:
            current_distance -= self.current_speed * 0.7
        
        maximum_speed = np.sqrt(2*self.MU * self.GRAVITY_ACCELERATION * np.fabs(current_distance)) - 2

        if maximum_speed >= self.SPEED_MAX:
            maximum_speed = self.SPEED_MAX
        if self.current_speed <= maximum_speed:
            # ACC
            if self.current_speed >= 10:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed) * 0.7)
            else:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed) * 0.5)
        else:
            # set_speed = 0
            set_speed = self.current_speed - np.fabs((maximum_speed - self.current_speed) * 0.2)
        # print("speed :", set_speed, "current", maximum_speed)
        return set_speed

class local_fgm(threading.Thread):
    def __init__(self, local_od_q, main_q):
        super(local_fgm, self).__init__()
        #multiprocessing.Process.__init__(self)
        self.local_od_q = local_od_q
        self.main_q = main_q

        self.RACECAR_LENGTH = rospy.get_param('robot_length', 0.325)
        self.ROBOT_LENGTH = rospy.get_param('robot_length', 0.325)
        self.ROBOT_SCALE = rospy.get_param('robot_scale', 0.35)
        self.SPEED_MAX = rospy.get_param('max_speed',20.0)
        self.SPEED_MIN = rospy.get_param('min_speed', 1.5)
        self.RATE = rospy.get_param('rate', 100)
        self.GAP_THETA_GAIN = rospy.get_param('gap_theta_gain', 10.0)
        self.REF_THETA_GAIN = rospy.get_param('ref_theta_gain', 5)
        self.MU = rospy.get_param('mu', 0.523)
        self.PI = rospy.get_param('pi', 3.141592)
        self.GRAVITY_ACC = rospy.get_param('g', 9.81)

        self.time_data_file_name = "fgm_pp_time_data"
        self.time_data_path = rospy.get_param("time_data_path", "/home/lab/f1tenth_ws/src/car_duri/recording/fgm_gnu_time_data.csv")
        self.time_data = open(f"{self.time_data_path}/{self.time_data_file_name}.csv", "w", newline="")
        self.time_data_writer = csv.writer(self.time_data)

        self.interval = 0.00435
        self.scan_range = 0
        self.front_idx = 0
        self.LOOK = 2

        self.scan_origin =[0]*1080
        self.scan_filtered =[0]*1080
        self.FILTER_SCALE = 1.3
        self.gaps   = []
        self.GAP_SIZE = 1
        self.THRESHOLD = 3.0
        self.theta_for = self.PI/3
        self.for_point = 0
        self.for_gap = [0,0,0]
        self.gap_cont = 0
        self.current_position = [0]*3
        self.nearest_distance = 0
        self.wp_index_current = 0
        self.wp_num = 1
        self.desired_wp_rt = [0,0]
        self.current_speed = 5.0
        self.MU = 0.523

        self.idx_save = 0

        self.dmin_past = 0
        self.lap_time_flag = True


    def run(self):
        rate = rospy.Rate(self.RATE)
        while True:
            sensor_data = self.local_od_q.get()

            self.current_position = [sensor_data[0][0], sensor_data[0][1], sensor_data[0][2]]
            self.current_speed = sensor_data[0][3]
            
            self.interval = sensor_data[1][0]
            self.scan_range = sensor_data[1][1]
            self.scan_origin = sensor_data[1][2]
            self.scan_filtered = sensor_data[1][3]
            self.desired_wp_rt = sensor_data[2]
            self.front_idx = int(self.scan_range / 2)
            self.actual_lookahead = sensor_data[3]

            self.t_loop = sensor_data[4][0]
            self.tn0 = sensor_data[4][1]
            self.tn1 = sensor_data[4][2]

        
            
            #print(sensor_data[1])
            self.find_gap(self.scan_filtered)
            self.for_find_gap(self.scan_filtered)
            gap = self.find_best_gap(self.desired_wp_rt)
            self.main_drive(gap)

            self.tn2 = time.time()
            self.time_data_writer.writerow([self.t_loop, (self.tn2 - self.tn0), (self.tn2 - self.tn1), "lp"])
            # print("local execution time:", time.time() - self.tn1)
            # print("local")
            #rate.sleep()


    def GAP(self):
        if self.gap_cont <= 1 :
            self.for_point = (self.theta_for/self.interval)
            #[0] = start_idx, [1] = end_idx
            self.for_gap[0] = (self.front_idx) - self.for_point
            self.for_gap[1] = (self.front_idx) + self.for_point

            self.gap_cont += 1

    def find_gap(self, scan):
        self.gaps = []
        i = 0
        while i < self.scan_range - self.GAP_SIZE:
            if scan[i] > self.THRESHOLD:
                start_idx_temp = i
                end_idx_temp = i
                max_temp = scan[i]
                max_idx_temp = i

                while ((scan[i] > self.THRESHOLD) and (i + 1 < self.scan_range)):
                    i += 1
                    if scan[i] > max_temp:
                        max_temp = scan[i]
                        max_idx_temp = i
                if scan[i] > self.THRESHOLD:
                    i += 1
                end_idx_temp = i

                gap_temp = [0] * 3
                gap_temp[0] = start_idx_temp
                gap_temp[1] = end_idx_temp
                gap_temp[2] = max_idx_temp
                self.gaps.append(gap_temp)
            i += 1
        print(self.gaps)

    def for_find_gap(self, scan):
        self.for_point = (int)(self.theta_for / self.interval)
        # [0] = start_idx, [1] = end_idx

        start_idx_temp = (self.front_idx) - self.for_point
        end_idx_temp = (self.front_idx) + self.for_point

        max_idx_temp = start_idx_temp
        max_temp = scan[start_idx_temp]

        for i in range(start_idx_temp, end_idx_temp):
            if max_temp < scan[i]:
                max_temp = scan[i]
                max_idx_temp = i
        # [0] = start_idx, [1] = end_idx, [2] = max_idx_temp
        self.for_gap[0] = start_idx_temp
        self.for_gap[1] = end_idx_temp
        self.for_gap[2] = max_idx_temp

    def find_best_gap(self, ref):
        ##print(self.gaps)
        num = len(self.gaps)
        if num == 0:
            return self.for_gap
        else:

            step = (int)(ref[1]/self.interval)

            ref_idx = self.front_idx + step

            gap_idx = 0

            if self.gaps[0][0] > ref_idx:
                distance = self.gaps[0][0] - ref_idx
            elif self.gaps[0][1] < ref_idx:
                distance = ref_idx - self.gaps[0][1]
            else:
                distance = 0
                gap_idx = 0

            i = 1
            while (i < num):
                if self.gaps[i][0] > ref_idx:
                    temp_distance = self.gaps[i][0] - ref_idx
                    if temp_distance < distance:
                        distance = temp_distance
                        gap_idx = i
                elif self.gaps[i][1] < ref_idx:
                    temp_distance = ref_idx - self.gaps[i][1]
                    if temp_distance < distance:
                        distance = temp_distance
                        gap_idx = i
                else:
                    temp_distance = 0
                    distance = 0
                    gap_idx = i
                    # self.gaps[i][2] = ref_idx
                    break
                i += 1
            #가장 작은 distance를 갖는 gap만 return
            return self.gaps[gap_idx]
    
    def speed_controller(self):
        current_distance = np.average(self.scan_filtered[499:580])
        if np.isnan(current_distance):
            print("SCAN ERR")
            current_distance = 1.0
        
        if self.current_speed > 10:
            current_distance -= self.current_speed * 0.7
        
        maximum_speed = np.sqrt(2*self.MU * self.GRAVITY_ACC * np.fabs(current_distance)) - 2

        if maximum_speed >= self.SPEED_MAX:
            maximum_speed = self.SPEED_MAX
        
        if self.current_speed <= maximum_speed:
            # ACC
            if self.current_speed >= 10:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed))
            else:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed) * self.ROBOT_LENGTH)
        else:
            # set_speed = 0
            set_speed = self.current_speed - np.fabs((maximum_speed - self.current_speed) * 0.2)
        # print("speed :", set_speed, "current", maximum_speed)
        return set_speed
    
    def main_drive(self, goal):
        # goal - [2] = max_idx,
        print(goal)
        self.max_angle = (goal[2] - self.front_idx) * self.interval
        self.wp_angle = self.desired_wp_rt[1]

        temp_avg = 0
        dmin = 0
        for i in range(10):
            
            dmin += self.scan_filtered[i]

        dmin /= 10

        i = 0

        while i < self.scan_range-7:
            j = 0
            while j < 10:
                if i + j > 1079:
                    temp_avg += 0
                else:
                    temp_avg += self.scan_filtered[i+j]
                j += 1
                
            temp_avg /= 10
            
            if dmin > temp_avg:
                if temp_avg == 0:
                    temp_avg = dmin
                dmin = temp_avg
            temp_avg = 0
            i += 3
        print(dmin)
        if dmin == 0:
            dmin = self.dmin_past

        controlled_angle = ((self.GAP_THETA_GAIN/dmin)*self.max_angle + self.REF_THETA_GAIN*self.wp_angle)/(self. GAP_THETA_GAIN/dmin + self.REF_THETA_GAIN) 
        distance = 1.0
        #path_radius = 경로 반지름 
        path_radius = distance / (2*np.sin(controlled_angle))
        #
        steering_angle = np.arctan(self.RACECAR_LENGTH/path_radius)

        # controlled_angle = self.max_angle
        # if controlled_angle == 0.0:
        #     controlled_angle = 0.001
        # path_radius = self.actual_lookahead**2 / (2 * np.sin(controlled_angle))
        # steering_angle = np.arctan(self.RACECAR_LENGTH / path_radius)

        # controlled_angle = self.max_angle
        # distance = 1.0
        # path_radius = distance / (self.actual_lookahead * np.sin(controlled_angle))
        # steering_angle = np.arctan(self.RACECAR_LENGTH / path_radius)
        speed = self.speed_controller()

        ackermann=[speed, steering_angle, self.idx_save]
        if self.main_q.full():
                self.main_q.get()
        self.main_q.put(ackermann)
        #speed, steering_angle
        self.dmin_past = dmin


class Obstacle_detect(threading.Thread):
    def __init__(self, global_od_q, local_od_q):
        super(Obstacle_detect, self).__init__()
        #multiprocessing.Process.__init__(self)
        self.global_od_q = global_od_q
        self.local_od_q = local_od_q

        self.waypoint_real_path = rospy.get_param('wpt_path', '../f1tenth_ws/src/car_duri/wp_vegas_test.csv')
        self.waypoint_delimeter = rospy.get_param('wpt_delimeter', ',')

        self.odom_topic = rospy.get_param("odom_topic", "/odom")
        self.scan_topic = rospy.get_param("scan_topic", "/scan")

        self.interval = 0.00435
        self.scan_range = 0
        self.front_idx = 0

        self.scan_origin = [0]*1080
        self.scan_filtered = [0]*1080
        self.gaps   = []
        self.scan_obs=[]
        self.dect_obs =[]
        self.len_obs = []
        self.obs= False
        self.ROBOT_SCALE = rospy.get_param('robot_scale', 0.35)
        self.FILTER_SCALE = rospy.get_param('filter_scale', 1.1) 
    
        self.PI = rospy.get_param('pi', 3.141592)
        self.RATE = rospy.get_param('rate', 100)

        self.current_position = [0]*5
        self.lidar_data = [0]*3
        self.wp_num = 0
        self.current_speed = [0,0,0]
        self.set_steering = 1.0
        self.waypoints = self.get_waypoint()
        self.desired_wp_rt = [0,0]
        self.wp_index_current = 0
        self.CURRENT_WP_CHECK_OFFSET = 2
        self.actual_lookahead = 0
        self.current_speed = 0

        rospy.Subscriber(self.scan_topic, LaserScan, self.subCallback_od, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.Odome, queue_size = 10)
    
    def Odome(self, odom_msg):
        # print("11")
        qx = odom_msg.pose.pose.orientation.x 
        qy = odom_msg.pose.pose.orientation.y 
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w 

        siny_cosp = 2.0 * (qw*qz + qx*qy)
        cosy_cosp = 1.0-2.0*(qy*qy + qz*qz)

        current_position_theta = np.arctan2(siny_cosp, cosy_cosp)
        current_position_x = odom_msg.pose.pose.position.x
        current_position_y = odom_msg.pose.pose.position.y
        
        _speed = odom_msg.twist.twist.linear.x
        _steer = odom_msg.twist.twist.angular.z
        self.current_speed = _speed
        self.set_steering = _steer
        self.current_position = [current_position_x,current_position_y, current_position_theta, self.current_speed, self.set_steering]
        # print(self.current_position)
    
    def find_nearest_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

        while True:
            wp_index_temp+=1
            if wp_index_temp >= len(self.waypoints)-1:
                wp_index_temp = 0
            
            temp_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

            if temp_distance < self.nearest_distance:
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
            elif temp_distance > (self.nearest_distance + self.CURRENT_WP_CHECK_OFFSET) or (wp_index_temp == self.wp_index_current):
                break
        
        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[self.wp_index_current])
        if(transformed_nearest_point[0] < 0): self.nearest_distance *= -1

    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        while(1):
            if(wp_index_temp >= len(self.waypoints)-1): wp_index_temp = 0
            distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)
            if distance >= self.lookahead_desired:
                if wp_index_temp-2 >=0 and wp_index_temp+2 < len(self.waypoints)-1:
                    self.waypoints[wp_index_temp][2] = np.arctan((self.waypoints[wp_index_temp+2][1]-self.waypoints[wp_index_temp-2][1])/self.waypoints[wp_index_temp+2][0]-self.waypoints[wp_index_temp-2][0])
                self.desired_point = self.waypoints[wp_index_temp]
                self.actual_lookahead = distance
                break
            wp_index_temp += 1
            

    def get_lookahead_desired(self):
        _vel = self.current_speed
        self.lookahead_desired = 0.5 + (0.3 * _vel)
    
    def get_waypoint(self):
        file_wps = np.genfromtxt(self.waypoint_real_path, delimiter=self.waypoint_delimeter, dtype='float')      
        # params.yaml 파일 수정 부탁드립니다... 제발...
        """
        # file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_vegas.csv',delimiter=',',dtype='float')
        # file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_obsmap2.csv',delimiter=',',dtype='float')
        # file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_vegas_test.csv',delimiter=',',dtype='float')
        # file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_floor8.csv',delimiter=',',dtype='float')
        # file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_curve.csv',delimiter=',',dtype='float')
        """
        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0],i[1],0]
            temp_waypoint.append(wps_point)
            self.wp_num += 1
        # print("wp_num",self.wp_num)
        return temp_waypoint

    def getDistance(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return np.sqrt(dx**2 + dy**2)

    def transformPoint(self, origin, target):
        theta = self.PI/2 - origin[2]

        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        dtheta = target[2] + theta
        
        tf_point_x = dx * np.cos(theta) - dy * np.sin(theta)
        tf_point_y = dx * np.sin(theta) + dy * np.cos(theta)
        tf_point_theta = dtheta
        tf_point = [tf_point_x, tf_point_y, tf_point_theta]
        
        return tf_point

    def xyt2rt(self, origin):
        rtpoint = []

        x = origin[0]
        y = origin[1]

        #rtpoint[0] = r, [1] = theta
        rtpoint.append(np.sqrt(x*x + y*y))
        rtpoint.append(np.arctan2(y, x) - (self.PI/2))
        return rtpoint

    def subCallback_od(self, msg_sub):
        self.interval = msg_sub.angle_increment
        self.scan_range = len(msg_sub.ranges)
        self.front_idx = (int)(self.scan_range/2)
        
        self.scan_origin = [0]*self.scan_range
        self.scan_filtered = [0]*self.scan_range
        self.scan_obs = []
        # temp    
        for i in range(self.scan_range):
            self.scan_origin[i] = msg_sub.ranges[i]
            self.scan_filtered[i] = msg_sub.ranges[i]
         
        for i in range(self.scan_range):           
            if self.scan_origin[i] == 0:
                cont = 0 
                sum = 0
                for j in range(1,21):
                    if i-j >= 0:
                        if self.scan_origin[i-j] != 0:
                            cont += 1
                            sum += self.scan_origin[i-j]
                    if i+j < self.scan_range:
                        if self.scan_origin[i+j] != 0:
                            cont += 1
                            sum += self.scan_origin[i+j]
                self.scan_origin[i] = sum/cont

        for i in range(self.scan_range - 1):
            if self.scan_origin[i]*self.FILTER_SCALE < self.scan_filtered[i+1]:
                unit_length = self.scan_origin[i]*self.interval 
                filter_num = self.ROBOT_SCALE/unit_length

                j = 1
                while j < filter_num+1:
                    if i+j < self.scan_range:
                        if self.scan_filtered[i+j] > self.scan_origin[i]:
                            self.scan_filtered[i+j] = self.scan_origin[i]
                        else: break
                    else: break 
                    j += 1
        
            elif self.scan_filtered[i] > self.scan_origin[i+1]*self.FILTER_SCALE:
                unit_length = self.scan_origin[i+1]*self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 0
                while j < filter_num + 1:
                    if i-j > 0:
                        if self.scan_filtered[i-j] > self.scan_origin[i+1]:
                            self.scan_filtered[i-j] = self.scan_origin[i+1]
                        else: break
                    else: break
                    j += 1
        self.lidar_data = [self.interval, self.scan_range, self.scan_origin, self.scan_filtered]

    def obs_dect(self):
        #for i in range(1, self.scan_range - 1):
        self.scan_obs = []
        i=1
        d_group = 1.5
        d_pi = 0.00628
        while(self.scan_range - 1>i):
            start_idx_temp = i
            end_idx_temp = i
            max_idx_temp = i
            min_idx_temp = i
            i = i+1
            while  math.sqrt(math.pow(self.scan_origin[i]*math.sin(math.radians(0.25)),2) + math.pow(self.scan_origin[i-1]-self.scan_origin[i]*math.cos(math.radians(0.25)),2)) < d_group + self.scan_origin[i]*d_pi and (i+1 < self.scan_range ):
                if self.scan_origin[i] > self.scan_origin[max_idx_temp]:
                    max_idx_temp = i
                if self.scan_origin[i] < self.scan_origin[min_idx_temp]:
                    min_idx_temp = i
                i = i+1
            end_idx_temp = i-1
            obs_temp = [0]*5
            obs_temp[0] = start_idx_temp
            obs_temp[1] = end_idx_temp
            obs_temp[2] = max_idx_temp
            obs_temp[3] = self.scan_origin[max_idx_temp] 
            obs_temp[4] = self.scan_origin[min_idx_temp]
            self.scan_obs.append(obs_temp)
            i+=1 

        self.dect_obs=[]
        for i in range(len(self.scan_obs)):
            if self.scan_obs[i][3] < 4 and self.scan_obs[i][3] > 0:
                obs_temp = [0]*5
                obs_temp[0] = self.scan_obs[i][0]
                obs_temp[1] = self.scan_obs[i][1]
                obs_temp[2] = self.scan_obs[i][2]
                obs_temp[3] = self.scan_obs[i][3]
                obs_temp[4] = self.scan_obs[i][4]
                self.dect_obs.append(obs_temp)
        #print(self.dect_obs)
        self.len_obs=[]

        for i in range(len(self.dect_obs)):
            theta = (self.dect_obs[i][1] - self.dect_obs[i][0])*0.25
            lengh = math.sqrt(math.pow(self.scan_origin[self.dect_obs[i][1]]*math.sin(math.radians(theta)),2) + math.pow(self.scan_origin[self.dect_obs[i][0]]-self.scan_origin[self.dect_obs[i][1]]*math.cos(math.radians(theta)),2)) 
            #print(i,lengh)
            if lengh < 2:
                obs_temp = [0]*5
                obs_temp[0] = self.dect_obs[i][0]
                obs_temp[1] = self.dect_obs[i][1]
                obs_temp[2] = self.dect_obs[i][2]
                obs_temp[3] = self.dect_obs[i][3]
                obs_temp[4] = self.dect_obs[i][4]
                self.len_obs.append(obs_temp)
        #print(self.len_obs)

        self.obs = False
        for i in range(len(self.len_obs)):
            if self.len_obs[i][0] > 680 or self.len_obs[i][1] < 400:
                self.obs = False
            else:
                self.obs= True
                break
                
    def run(self):
        t0 = time.time() # init time 
        loop = 0

        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            if self.scan_range == 0: continue
            loop += 1
            t1 = time.time()
            
            self.obs_dect()

            self.find_nearest_wp()
            self.get_lookahead_desired()
            self.find_desired_wp()
            # self.obs= True
            if self.obs:
                self.transformed_desired_point = self.transformPoint(self.current_position, self.desired_point)
                self.transformed_desired_point = self.xyt2rt(self.transformed_desired_point)
            # self.transformed_desired_point = self.xyt2rt(self.transformed_desired_point)
                sensor_data = [self.current_position, self.lidar_data, self.transformed_desired_point, self.actual_lookahead, [loop, t0, t1]]
                if self.local_od_q.full():
                    self.local_od_q.get()
                self.local_od_q.put(sensor_data)
            else:
                self.transformed_desired_point = self.transformPoint(self.current_position, self.desired_point)
            # self.transformed_desired_point = self.xyt2rt(self.transformed_desired_point)
                sensor_data = [self.current_position, self.lidar_data, self.transformed_desired_point, self.actual_lookahead, [loop, t0, t1]]
                if self.global_od_q.full():
                    self.global_od_q.get()
                self.global_od_q.put(sensor_data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("driver_fgm_pp")

    global_od_q = Queue(1)
    local_od_q = Queue(1)
    main_q = Queue(1)

    global_t = global_pure(global_od_q, main_q) 
    local_t = local_fgm(local_od_q, main_q)
    maindrive_t = maindrive(main_q)
    obstacle_t = Obstacle_detect(global_od_q, local_od_q)

    global_t.start()
    local_t.start()
    maindrive_t.start()
    obstacle_t.start()

    rospy.spin()
