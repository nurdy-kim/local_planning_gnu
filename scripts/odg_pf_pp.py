#!/usr/bin/env python

import multiprocessing
import queue
import rospy
import numpy as np
import threading
import matplotlib.pyplot as plt
from queue import Queue
import math
import time
#from multiprocessing import Process, Queue

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

class maindrive:
    def __init__(self, global_q, local_q, obstacle_q):
        #self.local_q= local_q
        self.local_q = local_q
        self.global_q = global_q
        self.obstacle_q = obstacle_q
        self.RATE = 100
        self.ackermann_data = AckermannDriveStamped()
        self.drive_pub = rospy.Publisher("/ICE/drive", AckermannDriveStamped, queue_size=10)

    def maindrives(self):
        obstacle=False 
        rate = rospy.Rate(self.RATE)

        s1=[0]*100
        s2=[0]*100
        s=np.arange(100)


        while not rospy.is_shutdown():
            #if self.scan_range == 0: continue
            obstacle = self.obstacle_q.get()
            if obstacle:     #local
                #print("Local")
                ackermann = self.local_q.get()
                self.ackermann_data.drive.steering_angle = ackermann[1]
                self.ackermann_data.drive.speed = ackermann[0]
                #print("local", ackermann[2])
                a=0
            else:           #global
                #print("Global")
                ackermann = self.global_q.get()
                self.ackermann_data.drive.steering_angle = ackermann[1]
                self.ackermann_data.drive.speed = ackermann[0]
                #print("global", ackermann[2])
                a=1
            self.drive_pub.publish(self.ackermann_data)

            s1.pop(0)
            #s2.pop(0)
            s1.append(ackermann[2])
            #s2.append(a)
            #plt.subplot(1,2,1)
            #plt.plot(s,s1,linewidth=3.0)
            # #plt.ylim([0, 500])
            # plt.subplot(1,2,2)
            # plt.plot(s,s2,linewidth=3.0)
            # #plt.ylim([-0.8, 0.8])
            #plt.pause(0.001)
            #plt.clf()

            rate.sleep()

class global_pure(threading.Thread):
    def __init__(self, global_od_q, global_main_q):
        super(global_pure, self).__init__()
        #multiprocessing.Process.__init__(self)
        self.global_od_q = global_od_q
        self.global_main_q = global_main_q

        self.PI = 3.141592
        self.CURRENT_WP_CHECK_OFFSET = 2
        self.DX_GAIN = 2.5
        self.RACECAR_LENGTH = 0.3302
        self.ROBOT_SCALE = 0.25
        self.GRAVITY_ACCELERATION = 9.81
        self.FILTER_SCALE = 10000
        self.MASS = 3.47

        self.waypoint_real_path = rospy.get_param('wpt_path', '../f1tenth_ws/src/car_duri/wp_vegas_test.csv')
        self.waypoint_delimeter = rospy.get_param('wpt_delimeter', ',')

        #self.LOOKAHEAD_MAX = rospy.get_param("/pure_pursuit/driving/max_look_ahead")
        self.LOOKAHEAD_MAX = rospy.get_param('max_look_ahead', 1.9)
        #self.LOOKAHEAD_MIN = rospy.get_param("/pure_pursuit/driving/min_look_ahead")
        self.LOOKAHEAD_MIN = rospy.get_param('min_look_ahead', 0.9)
        #self.SPEED_MAX = rospy.get_param("/pure_pursuit/driving/max_speed")
        self.SPEED_MAX = rospy.get_param('max_speed', 20.0)
        #self.SPEED_MIN = rospy.get_param("/pure_pursuit/driving/min_speed")
        self.SPEED_MIN = rospy.get_param('min_speed', 1.5)
        #self.DP_ANGLE_PROPORTION = rospy.get_param("/pure_pursuit/tuning/DP_angle_proportion")
        #self.MSC_MUXSIZE = rospy.get_param("/pure_pursuit/driving/manual_speed_control/mux_size")
        self.MSC_MUXSIZE = rospy.get_param('mux_size', 0)
        #self.MU = rospy.get_param("/pure_pursuit/driving/mu")
        self.MU = rospy.get_param('mu', 0.523)
        #self.RATE = rospy.get_param("/pure_pursuit/driving/rate")
        self.RATE = rospy.get_param('rate', 100)
    
        self.waypoints = self.get_waypoint()
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
        

        self.idx_save = 0
        #self.marker_pub = rospy.Publisher('/marker', Marker, queue_size=10)
        
        self.lap_time_flag = True
        self.lap_time_start = 0
        self.lap_time = 0

    def run(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            sensor_data = self.global_od_q.get()
            self.current_position = [sensor_data[0][0], sensor_data[0][1], sensor_data[0][2]]
            self.current_speed = sensor_data[0][3]
            
            self.interver = sensor_data[1][0]
            self.scan_range = sensor_data[1][1]
            self.scan_filtered = sensor_data[1][2]

            self.find_nearest_wp()
            self.get_dx()

            self.get_lookahead_desired()
            self.find_desired_wp()

            self.transformed_desired_point = self.transformPoint(self.current_position, self.desired_point)

            self.find_path()
            steer  = self.setSteeringAngle()
            # speed = self.setSpeed_PossibleMaximumTest()
            speed = self.speed_controller()

            ackermann = [speed, steer, self.idx_save]
            #print(self.scan_filtered)
            if self.global_q.full():
                self.global_q.get()
            self.global_q.put(ackermann)

            rate.sleep()

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
        
    def get_waypoint(self):
        file_wps = np.genfromtxt(self.waypoint_real_path, delimiter=self.wpt_delimiter ,dtype='float')
        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0],i[1],i[2]]
            temp_waypoint.append(wps_point)

        return temp_waypoint
        
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
    
        self.idx_temp = wp_index_temp
        # marker = Marker()
        # marker.header.frame_id = "map"
        # marker.header.stamp = rospy.Time.now()
        # marker.ns = "mpc"
        # marker.id = 2
        # marker.type = marker.CUBE
        # marker.action = marker.ADD
        # marker.pose.position.x = self.waypoints[self.idx_temp][0]
        # marker.pose.position.y = self.waypoints[self.idx_temp][1]
        # marker.pose.position.z = 0.1
        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # marker.pose.orientation.z = 0.0
        # marker.pose.orientation.w = 1.0
        # marker.scale.x = 0.2
        # marker.scale.y = 0.2
        # marker.scale.z = 0.1
        # marker.color.a = 1.0
        # marker.color.r = 1.0
        # marker.color.g = 0.0
        # marker.color.b = 0.0
        # self.marker_pub.publish(marker)





    def get_dx(self):

        wp_min = self.find_lookahead_wp(self.LOOKAHEAD_MIN)
        wp_max = self.find_lookahead_wp(self.LOOKAHEAD_MAX)

        wp_min = self.transformPoint(self.current_position, wp_min)
        wp_max = self.transformPoint(self.current_position, wp_max)

        self.dx = wp_max[0] - wp_min[0]

    def get_lookahead_desired(self):
        _vel = self.current_speed
        self.lookahead_desired = 0.5 + (0.3 * _vel)#np.exp(-(self.DX_GAIN*np.fabs(self.dx) - np.log(self.LOOKAHEAD_MAX - self.LOOKAHEAD_MIN))) + self.LOOKAHEAD_MIN

    def find_lookahead_wp(self, length):
        wp_index_temp = self.wp_index_current
        while(1):
            if(wp_index_temp >= len(self.waypoints)-1): wp_index_temp = 0
            distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

            if(distance >= length): break
            wp_index_temp+=1
        return self.waypoints[wp_index_temp]
    
    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        while(1):
            if(wp_index_temp >= len(self.waypoints)-1): 
                wp_index_temp = 0
                lap_time_end = time.time()
                self.lap_time = lap_time_end - self.lap_time_start
                print(self.lap_time)
            distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)
            if distance >= self.lookahead_desired:
                if wp_index_temp-2 >=0 and wp_index_temp+2 < len(self.waypoints)-1:
                    self.waypoints[wp_index_temp][2] = np.arctan((self.waypoints[wp_index_temp+2][1]-self.waypoints[wp_index_temp-2][1])/self.waypoints[wp_index_temp+2][0]-self.waypoints[wp_index_temp-2][0])
                self.desired_point = self.waypoints[wp_index_temp]
                self.actual_lookahead = distance
                break
            wp_index_temp += 1
        
    
    def find_path(self):
        #right cornering
        if self.transformed_desired_point[0] > 0:
            self.goal_path_radius = pow(self.actual_lookahead, 2)/(2*self.transformed_desired_point[0])
            self.goal_path_theta = np.arcsin(self.transformed_desired_point[1]/self.goal_path_radius)
            self.steering_direction = -1

        #left cornering
        else:
            self.goal_path_radius = pow(self.actual_lookahead, 2)/((-2)*self.transformed_desired_point[0])
            self.goal_path_theta = np.arcsin(self.transformed_desired_point[1]/self.goal_path_radius)
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

    def setSpeed(self):
        controlled_speed_max = self.SPEED_MAX
        controlled_speed_min = self.SPEED_MIN
        for i in range(0,self.MSC_MUXSIZE):
            if(self.wp_index_current > self.manualSpeedArray[i][0]) and (self.wp_index_current < self.manualSpeedArray[i][1]):
                controlled_speed_max = self.manualSpeedArray[i][2]
                controlled_speed_min = self.manualSpeedArray[i][3]
                break
        _speed = np.exp(-(self.DX_GAIN*np.fabs(self.dx)-np.log(controlled_speed_max - controlled_speed_min))) + controlled_speed_min
        return _speed

    def setSpeed_PossibleMaximumTest(self):
        test_speed = np.sqrt(self.MU*self.GRAVITY_ACCELERATION*self.goal_path_radius)
        if test_speed > 6:
            return self.setSpeed()
        else:
            return test_speed

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

    # def find_path(self):
    #     # right cornering
    #     if self.transformed_desired_point[0] > 0:
    #         self.goal_path_radius = pow(self.actual_lookahead, 2)/(2*self.transformed_desired_point[0])
    #         self.steering_direction = -1

    #     # left cornering
    #     else:
    #         self.goal_path_radius = pow(self.actual_lookahead, 2)/((-2)*self.transformed_desired_point[0])
    #         self.steering_direction = 1

    #     steering_angle = np.arctan2(self.RACECAR_LENGTH, self.goal_path_radius) * self.steering_direction

    #     speed = self.speed_controller()
    #     ackermann = [speed, steering_angle, self.idx_save]
    #     #print(self.scan_filtered)
    #     if self.global_q.full():
    #         self.global_q.get()
    #     self.global_q.put(ackermann)

        # test_speed = np.sqrt(self.MU * self.GRAVITY_ACCELERATION * self.goal_path_radius)
        # if test_speed > 6:
        #     controlled_speed_max = self.SPEED_MAX
        #     controlled_speed_min = self.SPEED_MIN
        #     for i in range(0, self.MSC_MUXSIZE):
        #         if (self.wp_index_current > self.manualSpeedArray[i][0]) and (
        #                 self.wp_index_current < self.manualSpeedArray[i][1]):
        #             controlled_speed_max = self.manualSpeedArray[i][2]
        #             controlled_speed_min = self.manualSpeedArray[i][3]
        #             break
        #     speed = np.exp(-(self.DX_GAIN * np.fabs(self.dx) - np.log(controlled_speed_max - controlled_speed_min))) + controlled_speed_min
        #     ackermann = [speed, steering_angle, self.idx_save]

        #     if self.global_q.full():
        #         self.global_q.get()
        #     self.global_q.put(ackermann)
        # else:
        #     speed = test_speed
        #     #speed, steering_angle
        #     ackermann = [speed, steering_angle, self.idx_save]
        #     if self.global_q.full():
        #         self.global_q.get()
        #     self.global_q.put(ackermann)

class local_fgm(threading.Thread):
    def __init__(self, local_q):
        super(local_fgm, self).__init__()
        #multiprocessing.Process.__init__(self)
        self.local_q = local_q
       
        self.rep_count = 0
        self.ackermann_data = AckermannDriveStamped()
        self.PI = 3.141592
        self.MU = 0.523   #1.0
        self.MASS = 3.47
        self.GRAVITY_ACC = 9.81
        self.LOOK = 5
        self.SPEED_MAX = 20.0
        self.SPEED_MIN = 4.0
        self.RATE = 100
        self.ROBOT_SCALE = 0.25
        self.ROBOT_LENGTH = 0.325
        self.THRESHOLD = 2.0
        self.FILTER_SCALE = 1.1
        self.scan_range = 0
        self.desired_wp_rt = [0,0]
        
        self.front_idx = 539
        self.detect_range_s = 359
        self.detect_range_e = 719
        self.safety_threshold = 0
        self.min_idx = 0
        self.f_rep_past_list =[0]*1080
        #self.start = time.time()
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

        self.wp_num = 1
        self.waypoints = self.get_waypoint()
        self.wp_index_current = 0
        #self.goal_point = [37.6,-19.1, 0]
        self.nearest_distance = 0

        self.current_position = [0,0,0]
        self.interval = 0.00435
        self.gamma = 1.0
        #self.a_k = 1.2
        self.current_speed = 1.0
        self.set_speed = 0.0
        self.alpha = 0.9

        self.mode = 0

        self.idx_temp = 0
        self.flag = False

        self.idx_save = 0

        self.dmin_past = 0
        rospy.Subscriber('/ICE/scan', LaserScan, self.subCallback_scan, queue_size=10)
        rospy.Subscriber('/ICE/odom', Odometry, self.Odome, queue_size=10)
        self.marker_pub = rospy.Publisher('/marker', Marker, queue_size=10)

    def run(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            if self.scan_range == 0: continue
         
            obstacles = self.define_obstacles(self.scan_filtered)
            #print(len(obstacles))
            rep_list = self.rep_field(obstacles)
            att_list = self.att_field(self.desired_wp_rt)
            total_list = self.total_field(rep_list, att_list)

            desired_angle = total_list#self.angle(total_list)
            # print(self.f_total_list[total_list])
            self.main_drive(desired_angle)
            rate.sleep()

    def get_waypoint(self):
    #file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_vegas.csv',delimiter=',',dtype='float')
        file_wps = np.genfromtxt('/home/lab/f1tenth_ws/src/car_duri/wp_vegas.csv', delimiter=',', dtype='float')
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
    
    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

        _vel = self.current_speed
        self.LOOK = 0.5 + (0.3 * _vel)
        
        while True:
            wp_index_temp+=1

            if wp_index_temp >= self.wp_num-1:
                wp_index_temp = 0
            temp_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

            if temp_distance < self.nearest_distance:
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
            elif ((temp_distance > (self.nearest_distance + self.LOOK*1.2)) or (wp_index_temp == self.wp_index_current)):
                break
        
        temp_distance = 0
        idx_temp = self.wp_index_current
        while True:
            if idx_temp >= self.wp_num-1:
                idx_temp = 0
            temp_distance = self.getDistance(self.waypoints[idx_temp], self.current_position)
            if temp_distance > self.LOOK: break
            idx_temp += 1

        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[idx_temp])
        self.desired_wp_rt = self.xyt2rt(transformed_nearest_point)

        #self.idx_temp = idx_temp

        # marker = Marker()
        # marker.header.frame_id = "map"
        # marker.header.stamp = rospy.Time.now()
        # marker.ns = "mpc"
        # marker.id = 2
        # marker.type = marker.CUBE
        # marker.action = marker.ADD
        # marker.pose.position.x = self.waypoints[idx_temp][0]
        # marker.pose.position.y = self.waypoints[idx_temp][1]
        # marker.pose.position.z = 0.1
        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # marker.pose.orientation.z = 0.0
        # marker.pose.orientation.w = 1.0
        # marker.scale.x = 0.2
        # marker.scale.y = 0.2
        # marker.scale.z = 0.1
        # marker.color.a = 1.0
        # marker.color.r = 1.0
        # marker.color.g = 0.0
        # marker.color.b = 0.0
        # self.marker_pub.publish(marker)

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
                a_k = ((max_temp - distance_obstacle)*np.exp(1/4))

                angle_obstacle = (end_idx_temp - start_idx_temp)*self.interval

                sigma_obstacle = np.arctan2((distance_obstacle * np.tan(angle_obstacle/2) + (self.ROBOT_SCALE/2)), distance_obstacle)

                angle_obstacle_center = (int)((end_idx_temp - start_idx_temp)/2) + start_idx_temp 
                angle_obstacle_center = angle_obstacle_center - self.front_idx

                obstacle_inf = [angle_obstacle_center, sigma_obstacle, a_k]
                #print('obstacle', obstacle_imf)
                obstacles.append(obstacle_inf)

            i += 1
        # if len(obstacles) == 1:
        
        #     print('obs_num:',len(obstacles))
        #     for j in obstacles:
            
        #         print('obs_info', j[2],j[3],end_idx_temp - start_idx_temp,j[4] )

        return obstacles

    def rep_field(self, obstacles):
        # print("obs_num",len(obstacles),"obs_inf",obstacles)
        f_rep_list = [0]*self.scan_range # np.zeros(self.scan_range)
        for i in range(len(obstacles)):
            for j in range(self.detect_range_s, self.detect_range_e):
                    f_rep_list[j] += obstacles[i][2] * np.exp((-0.5)*((((j-self.front_idx)*self.interval - obstacles[i][0]*self.interval)**2) / (obstacles[i][1])**2))                 
        #print("rep",f_rep_list,"front",f_rep_list[540],"left",f_rep_list[899],"right",f_rep_list[179])
        self.f_rep_list = f_rep_list
        return f_rep_list

    def att_field(self, goal_point):
        # transformed_nearest_point = self.transformPoint(self.current_position, self.goal_point)
        # self.desired_wp_rt = self.xyt2rt(transformed_nearest_point)
        # print('current_position',self.current_position, 'goal_theta',self.goal_theta[1])#'goal_point:',goal_point, 

        f_att_list = []
        for i in range(self.scan_range):
            idx2deg = (-self.front_idx+i)*self.interval
            f_att = self.gamma * np.fabs(goal_point[1] - idx2deg)
            f_att_list.append(f_att)
        # print('f_att_list:\n',f_att_list)
        return f_att_list 

    #
    def total_field(self, f_rep_list, f_att_list):
        
        f_total_list = [0]*self.scan_range

        for i in range(self.scan_range):
            f_total_list[i] = f_rep_list[i] + f_att_list[i]

        self.min_idx = np.argmin(f_total_list[self.detect_range_s:self.detect_range_e])+self.detect_range_s
        # print(f"tot:{f_total_list[min_idx]}, att : {f_att_list[min_idx]}, rep : {f_rep_list[min_idx]}\n")
        # print(f"min_idx:{min_idx}, att:{np.argmin(f_att_list)}, rep:{np.argmin(f_rep_list)}")
        # print("min_idx:",min_idx, "att:",f_att_list[min_idx], "rep:",f_rep_list[self.detect_range_s:self.detect_range_e])
        # print("rep_max:",f_rep_list[np.argmax(f_rep_list)])
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
            print("SCAN ERR")
            current_distance = 1.0
        
        if self.current_speed > 10:
            current_distance -= self.current_speed * 0.7
        
        maximum_speed = np.sqrt(2*self.MU * self.GRAVITY_ACC * current_distance) - 2

        if maximum_speed >= self.SPEED_MAX:
            maximum_speed = self.SPEED_MAX
        
        if self.current_speed <= maximum_speed:
            # ACC
            if self.current_speed >= 10:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed))
            else:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed) * 0.3)
        else:
            # set_speed = 0
            set_speed = self.current_speed - np.fabs((maximum_speed - self.current_speed) * 0.2)
        # print("speed :", set_speed, "current", maximum_speed)
        return set_speed
    
    # def steering_controller(self):
    #     _dx = self.current_position[0] - self.desired_wp_rt[0]
    #     _dy = self.current_position[1] - self.desired_wp_rt[1]

    #     theta = np.arctan2(_dy,_dx)
    #     delta = self.steering_angle

    #     # pid = self.steering_angle - self.d*(self.steering_angle - self.steering_angle_past) - self.i*(self.current_position[2] - self.current_position_past)
    #     # pid = delta - self.d * (delta - theta) - self.i * (delta - theta)
    #     pid = self.steering_angle
        
    #     return pid

    def main_drive(self, goal):

        self.steering_angle = (-self.front_idx+goal)*self.interval

        controlled_angle = self.steering_angle
        if controlled_angle == 0.0:
            controlled_angle = 0.001
        #distance = 1.5
        path_radius = self.LOOK**2 / (2 * np.sin(controlled_angle))
        steering_angle = np.arctan(self.ROBOT_LENGTH / path_radius)

        # if (np.fabs(self.steering_angle) > self.PI/6):
        #     speed = self.SPEED_MIN
        # else:
        #     # front_range = np.sum(self.scan_filtered[539:542])/len(self.scan_filtered[539:542])
        #     # speed = (float)(-(3/self.PI)*(self.SPEED_MAX-self.SPEED_MIN)*np.fabs(front_range) + self.SPEED_MAX)
        #     speed = (float)(-(3/self.PI)*(self.SPEED_MAX-self.SPEED_MIN)*np.fabs(self.steering_angle)+ self.SPEED_MAX)
        #     speed = np.fabs(speed)

        #sol_1 
        
        # steering_angle = self.steering_controller()
        # steering_angle = self.steering_angle - self.d*(self.steering_angle - self.current_position[2]) - self.i*(self.steering_angle - self.current_position[2])
        #steering_angle = self.steering_angle #- self.d*(self.steering_angle - self.steering_angle_past) - self.i*(self.current_position[2] - self.current_position_past)
        
        self.set_speed = self.speed_controller() # determin_speed

        ackermann=[self.set_speed, steering_angle, self.idx_save]
        if self.local_q.full():
                self.local_q.get()
        self.local_q.put(ackermann)

        self.current_position_past = self.current_position[2]
        self.steering_angle_past = steering_angle
        #self.error_past = error
        self.f_rep_past_list = self.f_rep_list

        self.ackermann_angle = steering_angle
        self.ackermann_speed = self.set_speed




    def Odome(self, odom_msg):
        qx = odom_msg.pose.pose.orientation.x 
        qy = odom_msg.pose.pose.orientation.y 
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w 

        siny_cosp = 2.0 * (qw*qz + qx*qy)
        cosy_cosp = 1.0-2.0*(qy*qy + qz*qz)

        current_position_theta = np.arctan2(siny_cosp, cosy_cosp)
        current_position_x = odom_msg.pose.pose.position.x
        current_position_y = odom_msg.pose.pose.position.y
        # print(current_position_theta)
        self.current_position = [current_position_x,current_position_y, current_position_theta]

        self.find_desired_wp()
        _speed = odom_msg.twist.twist.linear.x
        _steer = odom_msg.twist.twist.angular.z
        self.current_speed = _speed
        self.set_steering = _steer

    def subCallback_scan(self,msg_sub):
        self.scan_angle_min = msg_sub.angle_min
        self.scan_angle_max = msg_sub.angle_max
        self.interval = msg_sub.angle_increment
        self.scan_range = len(msg_sub.ranges)
        self.front_idx = (int)(self.scan_range/2)
        
        self.scan_origin = [0]*self.scan_range
        self.scan_filtered = [0]*self.scan_range
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
                self.scan_filtered[i] = sum/cont

        for i in range(self.scan_range - 1):
            if self.scan_origin[i]*self.FILTER_SCALE < self.scan_filtered[i+1]:
		#print('filter')
                unit_length = self.scan_origin[i]*self.interval 
                filter_num = self.ROBOT_SCALE/unit_length

                j = 1
                while j < filter_num + 1:
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
   

    # def main_drive(self, goal):
    #     # goal - [2] = max_idx,
    #     self.max_angle = (goal[2] - self.front_idx) * self.interval
    #     self.wp_angle = self.desired_wp_rt[1]

    #     # range_min_values = [0]*10
    #     temp_avg = 0
    #     dmin = 0
    #     for i in range(10):
            
    #         dmin += self.scan_filtered[i]

    #     dmin /= 10

    #     i = 0

    #     while i < self.scan_range-7:
    #         j = 0
    #         while j < 10:
    #             if i + j > 1079:
    #                 temp_avg += 0
    #             else:
    #                 temp_avg += self.scan_filtered[i+j]
    #             j += 1
                
    #         temp_avg /= 10
            
    #         if dmin > temp_avg:
    #             if temp_avg == 0:
    #                 temp_avg = dmin
    #             dmin = temp_avg
    #         temp_avg = 0
    #         i += 3

    #     if dmin == 0:
    #         dmin = self.dmin_past
        
    #     controlled_angle = ( (self.GAP_THETA_GAIN/dmin)*self.max_angle + self.REF_THETA_GAIN*self.wp_angle)/(self. GAP_THETA_GAIN/dmin + self.REF_THETA_GAIN)
    #     distance = 1.0
    #     path_radius = distance / (2 * np.sin(controlled_angle))
    #     steering_angle = np.arctan(self.RACECAR_LENGTH / path_radius)

    #     if (np.fabs(steering_angle) > self.PI / 8):
    #         speed = self.SPEED_MIN

    #     else:
    #         speed = (float)(-(3 / self.PI) * (self.SPEED_MAX - self.SPEED_MIN) * np.fabs(self.max_angle) + self.SPEED_MAX)
    #         speed = np.fabs(speed)

    #     if speed < self.SPEED_MIN:
    #         speed = self.SPEED_MIN

    #     ackermann=[speed, steering_angle, self.idx_save]
    #     if self.local_q.full():
    #             self.local_q.get()
    #     self.local_q.put(ackermann)
    #     #speed, steering_angle

    #     self.dmin_past = dmin

class Obstacle_detect(threading.Thread):
    def __init__(self, global_od_q, local_od_q):
        super(Obstacle_detect, self).__init__()
        #multiprocessing.Process.__init__(self)
        self.global_od_q = global_od_q
        self.local_od_q = local_od_q

        self.interval = 0.00435
        self.scan_range = 0
        self.front_idx = 0

        self.scan_origin = []
        self.gaps   = []
        self.scan_obs=[]
        self.dect_obs =[]
        self.len_obs = []
        self.obs= False
    
        self.PI = 3.141592
        self.RATE = 100

        self.current_position = [0]*5
        self.lidar_data = [0]*3

        self.current_speed = [0,0,0]
        self.set_steering = 1.0

        rospy.Subscriber('/ICE/scan', LaserScan, self.subCallback_od, queue_size=10)
        rospy.Subscriber('/ICE/odom', Odometry, self.Odome, queue_size = 10)
    
    def Odome(self, odom_msg):
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

    def subCallback_od(self, msg_sub):
        self.interval = msg_sub.angle_increment
        self.scan_range = len(msg_sub.ranges)
        self.front_idx = (int)(self.scan_range/2)
        
        self.scan_origin = [0]*self.scan_range
        self.scan_obs = []
        # temp    
        for i in range(self.scan_range):
            self.scan_origin[i] = msg_sub.ranges[i]
         
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
        self.lidar = [self.interval, self.scan_range, self.scan_origin]

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
            if self.scan_obs[i][3] < 5 and self.scan_obs[i][3] > 0:
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
            if self.len_obs[i][0] > 600 or self.len_obs[i][1] < 480:
                self.obs = False
            else:
                self.obs= True
                break
                
    def decision(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            if self.scan_range == 0: continue
            self.obs_dect()
            sensor_data = [self.current_position, self.lidar_data]
            if self.obs:
                self.global_od_q.put(sensor_data)
            else:
                self.local_od_q.put(sensor_data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("test")
    global_od_q = Queue(1)
    local_od_q = Queue(1)
    global_main_q = Queue(1)
    local_main_q = Queue(1)

    global_t = global_pure(global_od_q, global_main_q) 
    local_t = local_fgm(local_od_q, local_main_q)
    maindrive_t = maindrive(global_main_q, local_main_q)

    global_t.start()
    local_t.start()
    maindrive_t.start()

    A = Obstacle_detect(global_od_q, local_od_q)
    A.decision()
    rospy.spin()

