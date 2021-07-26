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
from f1tenth_gym_ros.msg import RaceInfo

class maindrive(threading.Thread):
    def __init__(self, main_q):
        super(maindrive, self).__init__()
        self.main_q = main_q
        self.RATE = 100
        self.ackermann_data = AckermannDriveStamped()
        self.drive_topic = rospy.get_param("drive_topic", "/drive")
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=10)

    def run(self):
        rate = rospy.Rate(self.RATE)
        while True:
        # while not rospy.is_shutdown():
            #if self.scan_range == 0: continue
            ackermann = self.main_q.get()
            # print("main", ackermann[1])
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

        self.marker_topic = rospy.get_param("marker_topic", "/marker")

        self.PI = rospy.get_param('pi', 3.141592)
        self.RACECAR_LENGTH = rospy.get_param('robot_length', 0.325)
        self.ROBOT_SCALE = rospy.get_param('robot_scale', 0.25)
        self.GRAVITY_ACCELERATION = rospy.get_param('g', 9.81)
        self.MASS = rospy.get_param('mass', 3.47)
        self.CURRENT_WP_CHECK_OFFSET = 2
        self.DX_GAIN = 2.5
        self.FILTER_SCALE = 10000

        self.waypoint_real_path = rospy.get_param('wpt_path', '../f1tenth_ws/src/car_duri/wp_vegas_test.csv')
        self.waypoint_delimeter = rospy.get_param('wpt_delimeter', ',')

        self.time_data_file_name = "odg_pf_pp_time_data"
        self.time_data_path = rospy.get_param("time_data_path", "/home/lab/f1tenth_ws/src/car_duri/recording/fgm_gnu_time_data.csv")
        self.time_data = open(f"{self.time_data_path}/{self.time_data_file_name}.csv", "w", newline="")
        self.time_data_writer = csv.writer(self.time_data)

        self.trj_path = rospy.get_param("trj_path", "")
        self.time_data_path = rospy.get_param("time_data_path", "")

        self.LOOKAHEAD_MAX = rospy.get_param('max_look_ahead', 1.9)
        self.LOOKAHEAD_MIN = rospy.get_param('min_look_ahead', 0.9)
        self.SPEED_MAX = rospy.get_param('max_speed', 20.0)
        self.SPEED_MIN = rospy.get_param('min_speed', 1.5)
        self.MSC_MUXSIZE = rospy.get_param('mux_size', 0)
        self.MU = rospy.get_param('mu', 0.523)
        self.RATE = rospy.get_param('rate', 100)
    
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

        self.interval = 0
        self.scan_range = 0
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=10)

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

            ackermann = [speed, steer]
            #print(self.scan_filtered)
            # if self.main_q.full():
            #     self.main_q.get()
            self.main_q.put(ackermann)
            self.tn2 = time.time()
            self.time_data_writer.writerow([self.t_loop, (self.tn2 - self.tn0), (self.tn2 - self.tn1), "gp"])
            # print("global")
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
            # print("SCAN ERR")
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

        self.marker_topic = rospy.get_param("marker_topic", "/marker")

        self.rep_count = 0
        self.ackermann_data = AckermannDriveStamped()
        self.PI = rospy.get_param('pi', 3.141592)
        self.MU = rospy.get_param('mu', 0.523)   #1.0
        self.MASS = rospy.get_param('mass', 3.47)
        self.GRAVITY_ACC = rospy.get_param('g', 9.81)
        self.SPEED_MAX = rospy.get_param('max_speed', 7.0)
        self.SPEED_MIN = rospy.get_param('min_speed', 1.5)
        self.RATE = rospy.get_param('rate', 100)
        self.ROBOT_SCALE = rospy.get_param('robot_scale', 0.25)
        self.ROBOT_LENGTH = rospy.get_param('robot_length', 0.325)
        self.LOOK = 5
        self.THRESHOLD = 2.0
        self.FILTER_SCALE = 1.1
        self.scan_range = 0
        self.desired_wp_rt = [0,0]

        self.time_data_file_name = "odg_pf_pp_time_data"
        self.time_data_path = rospy.get_param("time_data_path", "/home/lab/f1tenth_ws/src/car_duri/recording/fgm_gnu_time_data.csv")
        self.time_data = open(f"{self.time_data_path}/{self.time_data_file_name}.csv", "w", newline="")
        self.time_data_writer = csv.writer(self.time_data)

        self.waypoint_real_path = rospy.get_param('wpt_path', '../f1tenth_ws/src/car_duri/wp_vegas_test.csv')
        self.waypoint_delimeter = rospy.get_param('wpt_delimeter', ',')
        
        self.front_idx = 539
        self.detect_range_s = 360
        self.detect_range_e = 720
        self.detect_range = self.detect_range_e - self.detect_range_s
        self.detect_n = 4

        self.safety_threshold = 0
        self.min_idx = 0
        self.f_rep_past_list =[0]*1080
        #self.p = 0.1
        self.w = 0.9
        self.d = 0.05
        self.i = 0.5
        self.steering_min = 5
        self.steering_max = 15

        self.error_past = 0   
        self.current_position_past = 0
        self.steering_angle = 0
        self.set_steering = 0
        self.steering_angle_past = 0

        self.current_position = [0,0,0]
        self.current_speed = 5.0


        self.interval = 0.00435
        self.gamma = 0.5
        #self.a_k = 1.2
        self.current_speed = 1.0
        self.set_speed = 0.0
        self.alpha = 0.9

        self.mode = 0

        self.idx_temp = 0
        self.flag = False

        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=10)

    def run(self):
        rate = rospy.Rate(self.RATE)

        self.s1 = [0]*750
        self.s2 = [0]*750
        self.s3 = [0]*750
        self.s = np.arange(750)

        #speed monitoring
        self.b1 = [0]*750
        self.b2 = [0]*750
        self.b = np.arange(750)
        
        self.c1 = [0]*(self.detect_range*self.detect_n)
        self.c2 = [0]*(self.detect_range*self.detect_n)
        self.c3 = [0]*(self.detect_range*self.detect_n)
        self.c = np.arange(self.detect_range*self.detect_n)
        
        i = 0

        while True:
            i+=1

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
            if self.scan_range == 0: continue
            # print("output",sensor_data[0][4])

            self.t_loop = sensor_data[4][0]
            self.tn0 = sensor_data[4][1]
            self.tn1 = sensor_data[4][2]
            
            obstacles = self.define_obstacles(self.scan_origin)
            #print(len(obstacles))
            rep_list = self.rep_field(obstacles)
            att_list = self.att_field(self.desired_wp_rt)
            total_list = self.total_field(rep_list, att_list)
            desired_angle = total_list#self.angle(total_list)
            steer = self.main_drive(total_list)
            # print("input", steer)

            speed = self.speed_controller()
            ackermann = [speed, steer]
            self.main_q.put(ackermann)
            
            self.tn2 = time.time()
            self.time_data_writer.writerow([self.t_loop, (self.tn2 - self.tn0), (self.tn2 - self.tn1), "lp"])
            
            # print("local")

            # if i % 10 == 0:
            if False:
                del self.s1[0]
                del self.s2[0]
                del self.s3[0]
                
                del self.b1[0]
                del self.b2[0]
                
                del self.c1[0:self.detect_range]
                del self.c2[0:self.detect_range]
                del self.c3[0:self.detect_range]

                self.s1.append(self.f_total_list[total_list])
                self.s2.append(att_list[total_list])
                self.s3.append(rep_list[total_list])

                self.b1.append(self.current_speed)
                self.b2.append(self.set_speed)

                self.c1 = self.c1 + self.f_total_list[self.detect_range_s:self.detect_range_e][::-1]
                self.c2 = self.c2 + att_list[self.detect_range_s:self.detect_range_e][::-1]
                self.c3 = self.c3 + rep_list[self.detect_range_s:self.detect_range_e][::-1]

                # ####################
                plt.subplot(1,1,1)
                plt.plot(self.c,self.c1,color = 'black', label ='total field',linewidth=3.0)
                plt.xticks([self.detect_range*0,self.detect_range*1,self.detect_range*2,self.detect_range*3,self.detect_range*4,self.detect_range*self.detect_n])
                plt.legend(bbox_to_anchor=(1,1))
                plt.ylim([0, 3])
                plt.grid()

                #plt.subplot(1,1,1)
                plt.plot(self.c,self.c2,color = 'b',label ='att field',linewidth=3.0)
                plt.legend(bbox_to_anchor=(1,1))
                plt.xticks([self.detect_range*0,self.detect_range*1,self.detect_range*2,self.detect_range*3,self.detect_range*4,self.detect_range*self.detect_n])
                # plt.ylim([0, 5])
                plt.grid()

                #plt.subplot(1,1,1)
                plt.plot(self.c,self.c3,color = 'r',label ='rep field',linewidth=3.0)
                plt.legend(bbox_to_anchor=(1,1))
                plt.xticks([self.detect_range*0,self.detect_range*1,self.detect_range*2,self.detect_range*3,self.detect_range*4,self.detect_range*self.detect_n])
                # plt.ylim([0, 5])
                plt.grid()
                plt.pause(0.001)
                plt.clf()
                print(self.steering_angle_past)
                # ##################
            #rate.sleep()

    def define_obstacles(self, scan):
        obstacles = []
        
        i = self.detect_range_s

        while True:
            if (i >= self.detect_range_e):
                break
            if scan[i] < self.THRESHOLD:
                
                start_temp = scan[i]
                start_idx_temp = i
                end_temp = start_temp
                end_idx_temp = i
                max_temp = scan[i]
                max_idx_temp = i
                obstacle_count = 1
                
                while ((scan[i] < self.THRESHOLD) and (i+1 < self.detect_range_e)):#self.scan_range
                    i += 1
                    end_temp += scan[i]
                    obstacle_count += 1
                    if scan[i] > max_temp:
                        max_temp = scan[i]
                        max_idx_temp = i
                if scan[i] < self.THRESHOLD:
                    i += 1   
                end_idx_temp = i

                distance_obstacle = end_temp/obstacle_count
                a_k = ((self.THRESHOLD - distance_obstacle)*np.exp(1/8))

                angle_obstacle = (end_idx_temp - start_idx_temp)*self.interval

                sigma_obstacle = np.arctan2((distance_obstacle * np.tan(angle_obstacle/2) + (self.ROBOT_SCALE/2)), distance_obstacle)

                angle_obstacle_center = (int)((end_idx_temp - start_idx_temp)/2) + start_idx_temp 
                angle_obstacle_center = angle_obstacle_center - self.front_idx

                obstacle_inf = [angle_obstacle_center, sigma_obstacle, a_k]
                #print('obstacle', obstacle_imf)
                obstacles.append(obstacle_inf)

            i += 1

        return obstacles

    def rep_field(self, obstacles):

        f_rep_list = [0]*self.scan_range # np.zeros(self.scan_range)
        for i in range(len(obstacles)):
            for j in range(self.detect_range_s, self.detect_range_e):
                f_rep_list[j] += obstacles[i][2] * np.exp((-0.5)*((((j-self.front_idx)*self.interval - obstacles[i][0]*self.interval)**2) / (obstacles[i][1])**2))

        self.f_rep_list = f_rep_list
        
        #reversed(f_rep_list)
        return f_rep_list

    def att_field(self, goal_point):

        f_att_list = []
        for i in range(self.scan_range):
            idx2deg = (-self.front_idx+i)*self.interval
            f_att = self.gamma * np.fabs(goal_point[1] - idx2deg)
            f_att_list.append(f_att)

        return f_att_list 

    def total_field(self, f_rep_list, f_att_list):
        
        f_total_list = [0]*self.scan_range

        for i in range(self.scan_range):
            f_total_list[i] = f_rep_list[i] + f_att_list[i]

        self.min_idx = np.argmin(f_total_list[self.detect_range_s:self.detect_range_e])+self.detect_range_s

        self.f_total_list = f_total_list
        return self.min_idx

    def angle(self, f_total_list):

        min_f = f_total_list[0]*self.scan_range
        min_f_idx = self.detect_range_s

        for i in range(self.detect_range_s + 1, self.detect_range_e-1):
            if min_f > f_total_list[i]:
                min_f = f_total_list[i]
                min_f_idx = i

        return min_f_idx
    
    def speed_controller(self):
        current_distance = np.average(self.scan_filtered[499:580])
        if np.isnan(current_distance):
            # print("SCAN ERR")
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
        self.steering_angle = (goal-self.front_idx)*self.interval

        controlled_angle = self.steering_angle

        if controlled_angle == 0.0:
            controlled_angle = 0.001
        self.LOOK = 0.5 + (0.3 * self.current_speed)
        path_radius = self.LOOK**2 / (2 * np.sin(controlled_angle))
        steering_angle = np.arctan(self.ROBOT_LENGTH / path_radius)

        # if np.fabs(self.steering_angle) > 0.5:
        #     # print("in")
        #     if np.fabs(self.steering_angle_past - self.steering_angle) > 0.5 :
        #         steering_angle = self.steering_angle_past#((self.steering_angle+self.steering_angle_past*(0.5))/2)
        #         # print("to")
        # self.steering_angle_past = steering_angle
        
        return steering_angle
        #speed, steering_angle

class Obstacle_detect(threading.Thread):
    def __init__(self, global_od_q, local_od_q):
        super(Obstacle_detect, self).__init__()
        #multiprocessing.Process.__init__(self)
        self.global_od_q = global_od_q
        self.local_od_q = local_od_q

        self.drive_topic = rospy.get_param("drive_topic", "/drive") 
        self.odom_topic = rospy.get_param("odom_topic", "/odom")
        self.scan_topic = rospy.get_param("scan_topic", "/scan")
        self.marker_topic = rospy.get_param("marker_topic", "/marker")

        self.waypoint_real_path = rospy.get_param('wpt_path', '../f1tenth_ws/src/car_duri/wp_vegas_test.csv')
        self.waypoint_delimeter = rospy.get_param('wpt_delimeter', ',')

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
        self.idx_temp = 0

        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=10)
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


        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mpc"
        marker.id = 2
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position.x = self.waypoints[self.idx_temp][0]
        marker.pose.position.y = self.waypoints[self.idx_temp][1]
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
        # print(self.waypoints[self.idx_temp], self.idx_temp)
        self.marker_pub.publish(marker)
            

    def get_lookahead_desired(self):
        _vel = self.current_speed
        self.lookahead_desired = 0.5 + (0.3 * _vel)
    
    def get_waypoint(self):
        file_wps = np.genfromtxt(self.waypoint_real_path, delimiter=self.waypoint_delimeter, dtype='float')
        """
        #file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_vegas.csv',delimiter=',',dtype='float')
        file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_obsmap2.csv',delimiter=',',dtype='float')
        # file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_floor8.csv',delimiter=',',dtype='float')
        # file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_vegas_test.csv',delimiter=',',dtype='float')
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
        t0 = time.time()
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
            if self.obs:
                self.transformed_desired_point = self.transformPoint(self.current_position, self.desired_point)
                self.transformed_desired_point = self.xyt2rt(self.transformed_desired_point)
            # self.transformed_desired_point = self.xyt2rt(self.transformed_desired_point)
                sensor_data = [self.current_position, self.lidar_data, self.transformed_desired_point, self.actual_lookahead, [loop, t0, t1]]
                # if self.local_od_q.full():
                #     self.local_od_q.get()
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
    rospy.init_node("driver_odg_pf_pp")
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
