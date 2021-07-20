#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import queue
import rospy
import numpy as np
import threading
import matplotlib.pyplot as plt
from queue import Queue
import math
import time


from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

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
                print("Local")
                ackermann = self.local_q.get()
                self.ackermann_data.drive.steering_angle = ackermann[1]
                self.ackermann_data.drive.speed = ackermann[0]
                #print("local", ackermann[2])
                a=0
            else:           #global
                print("Global")
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
    def __init__(self, global_q):
        super(global_pure, self).__init__()
        self.global_q = global_q

        self.PI = 3.141592
        self.CURRENT_WP_CHECK_OFFSET = 2
        self.DX_GAIN = 2.5
        self.RACECAR_LENGTH = 0.3302
        self.ROBOT_SCALE = 0.25
        self.GRAVITY_ACCELERATION = 9.81
        self.FILTER_SCALE = 10000
        self.MASS = 3.47

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

        self.waypoint_real_path = rospy.get_param('wpt_path', '../f1tenth_ws/src/car_duri/wp_vegas_test.csv')
        self.waypoint_delimeter = rospy.get_param('wpt_delimeter', ',')
    
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
        rospy.Subscriber('/ICE/odom', Odometry, self.Odome, queue_size=10)
        rospy.Subscriber('/ICE/scan', LaserScan, self.subCallback_glob, queue_size=10)
        self.marker_pub = rospy.Publisher('/marker', Marker, queue_size=10)

        self.lap_time_flag = True
        self.lap_time_start = 0
        self.lap_time = 0
        
    def run(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
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
        # file_wps = np.genfromtxt('/home/lab/f1tenth_ws/src/car_duri/wp_vegas.csv', delimiter=',', dtype='float')
        file_wps = np.genfromtxt(self.waypoint_real_path, delimiter=self.waypoint_delimeter ,dtype='float')
        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0],i[1],i[2]]
            temp_waypoint.append(wps_point)

        return temp_waypoint

    def subCallback_glob(self, msg_sub):
        
        self.interval = msg_sub.angle_increment
        self.scan_range = len(msg_sub.ranges)
        self.front_idx = (int)(self.scan_range / 2)

        self.scan_origin = [0] * self.scan_range
        self.scan_filtered = [0] * self.scan_range
        for i in range(self.scan_range):
            self.scan_origin[i] = msg_sub.ranges[i]
            self.scan_filtered[i] = msg_sub.ranges[i]

        for i in range(self.scan_range):
            if self.scan_origin[i] == 0:
                cont = 0
                sum = 0
                for j in range(1, 21):
                    if i - j >= 0:
                        if self.scan_origin[i - j] != 0:
                            cont += 1
                            sum += self.scan_origin[i - j]
                    if i + j < self.scan_range:
                        if self.scan_origin[i + j] != 0:
                            cont += 1
                            sum += self.scan_origin[i + j]
                self.scan_origin[i] = sum / cont
                self.scan_filtered[i] = sum / cont

        for i in range(self.scan_range - 1):
            if self.scan_origin[i] * self.FILTER_SCALE < self.scan_filtered[i + 1]:
                unit_length = self.scan_origin[i] * self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 1
                while j < filter_num + 1:
                    if i + j < self.scan_range:
                        if self.scan_filtered[i + j] > self.scan_origin[i]:
                            self.scan_filtered[i + j] = self.scan_origin[i]
                        else:
                            break
                    else:
                        break
                    j += 1

            elif self.scan_filtered[i] > self.scan_origin[i + 1] * self.FILTER_SCALE:
                unit_length = self.scan_origin[i + 1] * self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 0
                while j < filter_num + 1:
                    if i - j > 0:
                        if self.scan_filtered[i - j] > self.scan_origin[i + 1]:
                            self.scan_filtered[i - j] = self.scan_origin[i + 1]
                        else:
                            break
                    else:
                        break
                    j += 1
                        
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
        self.current_position = [current_position_x,current_position_y, current_position_theta]

        self.current_speed = odom_msg.twist.twist.linear.x

        if self.current_speed > 1.0 and self.lap_time_flag:
            self.lap_time_flag = False
            self.lap_time_start = time.time()
        
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

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mpc"
        marker.id = 2
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position.x = self.waypoints[wp_index_temp][0]
        marker.pose.position.y = self.waypoints[wp_index_temp][1]
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
        self.marker_pub.publish(marker)
    
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
        current_distance = np.average(self.scan_filtered[499:580])
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
                set_speed = maximum_speed#self.current_speed + np.fabs((maximum_speed - self.current_speed))
            else:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed) * 0.4)
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
        self.local_q = local_q
        self.interval = 0.00435
        self.scan_range = 0
        self.front_idx = 0
        self.LOOK = 2
        self.GAP_THETA_GAIN = rospy.get_param('gap_theta_gain', 20.0)
        self.REF_THETA_GAIN = rospy.get_param('ref_theta_gain', 1.5)

        self.scan_origin = [0]*1080
        self.scan_filtered =[0]*1080
        self.FILTER_SCALE = 1.1
        self.gaps   = []
        self.GAP_SIZE = 1
        self.PI = 3.141595
        self.THRESHOLD = 3.0
        self.theta_for = self.PI/3
        self.for_point = 0
        self.for_gap = [0,0,0]
        self.RACECAR_LENGTH = 0.325
        self.ROBOT_SCALE = 0.35
        self.SPEED_MAX = 20.0
        self.SPEED_MIN = 1.9
        self.RATE= 100
        self.gap_cont = 0
        self.current_position = [0]*3
        self.nearest_distance = 0
        self.wp_index_current = 0
        self.wp_num = 1
        self.waypoints = self.get_waypoint()
        self.desired_wp_rt = [0,0]
        self.current_speed = 5.0
        self.MU = 0.523
        self.GRAVITY_ACC = 9.81

        self.idx_save = 0

        self.dmin_past = 0
        rospy.Subscriber('/ICE/scan', LaserScan, self.subCallback_scan, queue_size=10)
        rospy.Subscriber('/ICE/odom', Odometry, self.Odome_scan, queue_size=10)
        self.marker_pub = rospy.Publisher('/marker', Marker, queue_size=10)

    def run(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            if self.scan_range == 0: continue
            self.find_gap(self.scan_filtered)
            self.for_find_gap(self.scan_filtered)
            self.find_desired_wp()
            gap = self.find_best_gap(self.desired_wp_rt)
            self.main_drive(gap)
            rate.sleep()

    def get_waypoint(self):
    #file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_vegas.csv',delimiter=',',dtype='float')
        # file_wps = np.genfromtxt('/home/lab/f1tenth_ws/src/car_duri/wp_vegas.csv', delimiter=',', dtype='float')
        file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_vegas_test.csv',delimiter=',',dtype='float')
        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0],i[1],0]
            temp_waypoint.append(wps_point)
            self.wp_num += 1
        # print("wp_num",self.wp_num)
        return temp_waypoint

    def Odome_scan(self, odom_msg):
        qx = odom_msg.pose.pose.orientation.x 
        qy = odom_msg.pose.pose.orientation.y 
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w 

        siny_cosp = 2.0 * (qw*qz + qx*qy)
        cosy_cosp = 1.0-2.0*(qy*qy + qz*qz)

        current_position_theta = np.arctan2(siny_cosp, cosy_cosp)
        current_position_x = odom_msg.pose.pose.position.x
        current_position_y = odom_msg.pose.pose.position.y
        self.current_position = [current_position_x,current_position_y, current_position_theta]
        self.current_speed = odom_msg.twist.twist.linear.x

        #self.find_desired_wp()
    
    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)
        
        _vel = self.current_speed
        self.LOOK = 0.5+(_vel*0.3)

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

        self.idx_save = idx_temp
        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[idx_temp])
        self.desired_wp_rt = self.xyt2rt(transformed_nearest_point)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mpc"
        marker.id = 2
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position.x = self.waypoints[idx_temp][0]
        marker.pose.position.y = self.waypoints[idx_temp][1]
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
        self.marker_pub.publish(marker)

    def subCallback_scan(self, msg_sub):
        self.interval = msg_sub.angle_increment
        self.scan_range = len(msg_sub.ranges)
        self.front_idx = (int)(self.scan_range / 2)

        self.scan_origin = [0] * self.scan_range
        self.scan_filtered = [0] * self.scan_range
        for i in range(self.scan_range):
            self.scan_origin[i] = msg_sub.ranges[i]
            self.scan_filtered[i] = msg_sub.ranges[i]

        for i in range(self.scan_range):
            if self.scan_origin[i] == 0:
                cont = 0
                sum = 0
                for j in range(1, 21):
                    if i - j >= 0:
                        if self.scan_origin[i - j] != 0:
                            cont += 1
                            sum += self.scan_origin[i - j]
                    if i + j < self.scan_range:
                        if self.scan_origin[i + j] != 0:
                            cont += 1
                            sum += self.scan_origin[i + j]
                self.scan_origin[i] = sum / cont
                self.scan_filtered[i] = sum / cont

        for i in range(self.scan_range - 1):
            if self.scan_origin[i] * self.FILTER_SCALE < self.scan_filtered[i + 1]:
                unit_length = self.scan_origin[i] * self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 1
                while j < filter_num + 1:
                    if i + j < self.scan_range:
                        if self.scan_filtered[i + j] > self.scan_origin[i]:
                            self.scan_filtered[i + j] = self.scan_origin[i]
                        else:
                            break
                    else:
                        break
                    j += 1

            elif self.scan_filtered[i] > self.scan_origin[i + 1] * self.FILTER_SCALE:
                unit_length = self.scan_origin[i + 1] * self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 0
                while j < filter_num + 1:
                    if i - j > 0:
                        if self.scan_filtered[i - j] > self.scan_origin[i + 1]:
                            self.scan_filtered[i - j] = self.scan_origin[i + 1]
                        else:
                            break
                    else:
                        break
                    j += 1

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
                set_speed = maximum_speed #self.current_speed + np.fabs((maximum_speed - self.current_speed))
            else:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed) * 0.4)
        else:
            # set_speed = 0
            set_speed = self.current_speed - np.fabs((maximum_speed - self.current_speed) * 0.2)
        # print("speed :", set_speed, "current", maximum_speed)
        return set_speed
    
    def main_drive(self, goal):
        # goal - [2] = max_idx,
        self.max_angle = (goal[2] - self.front_idx) * self.interval
        self.wp_angle = self.desired_wp_rt[1]

        # range_min_values = [0]*10
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

        if dmin == 0:
            dmin = self.dmin_past
        
        controlled_angle = ( (self.GAP_THETA_GAIN/dmin)*self.max_angle + self.REF_THETA_GAIN*self.wp_angle)/(self. GAP_THETA_GAIN/dmin + self.REF_THETA_GAIN)
        distance = 1.0
        path_radius = distance / (2 * np.sin(controlled_angle))
        steering_angle = np.arctan(self.RACECAR_LENGTH / path_radius)

        # if (np.fabs(steering_angle) > self.PI / 8):
        #     speed = self.SPEED_MIN

        # else:
        #     speed = (float)(-(3 / self.PI) * (self.SPEED_MAX - self.SPEED_MIN) * np.fabs(self.max_angle) + self.SPEED_MAX)
        #     speed = np.fabs(speed)

        # if speed < self.SPEED_MIN:
        #     speed = self.SPEED_MIN

        speed = self.speed_controller()

        ackermann=[speed, steering_angle, self.idx_save]
        if self.local_q.full():
                self.local_q.get()
        self.local_q.put(ackermann)
        #speed, steering_angle

        self.dmin_past = dmin

class Obstacle_detect(threading.Thread):
    def __init__(self, obstacle_q):
        super(Obstacle_detect, self).__init__()
        self.obstacle_q = obstacle_q
        self.interval = 0.00435
        self.scan_range = 0
        self.front_idx = 0

        self.scan_origin = []
        self.gaps   = []
        self.scan_obs=[]
        self.dect_obs =[]
        self.len_obs = []
        self.obs= False

        self.wp_num = 0.5
        self.waypoints = self.get_waypoint()
        self.nearest_distance = 0
        self.current_position=[0]*3
        self.wp_index_current = 0
        self.LOOK = 1.5
        self.desired_wp_rt = [0,0]
        self.PI = 3.141592


        self.RATE = 100
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
        self.current_position = [current_position_x,current_position_y, current_position_theta]

        self.find_desired_wp()

    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

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
    
    def get_waypoint(self):
        file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_vegas_test.csv',delimiter=',',dtype='float')
        # file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_vegas.csv',delimiter=',',dtype='float')
        # file_wps = np.genfromtxt('~/f1tenth_ws/src/car_duri/wp_vegas.csv',delimiter=',',dtype='float')
        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0],i[1],0]
            temp_waypoint.append(wps_point)
            self.wp_num += 1
        print("wp_num",self.wp_num)
        return temp_waypoint
    
    def getDistance(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        
        return np.sqrt(dx**2 + dy**2)

    def xyt2rt(self, origin):
        rtpoint = []

        x = origin[0]
        y = origin[1]

        #rtpoint[0] = r, [1] = theta
        rtpoint.append(np.sqrt(x*x + y*y))
        rtpoint.append(np.arctan2(y, x) - (self.PI/2))

        return rtpoint
    
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
    
        # for i in range(1,self.scan_range - 1):
        #     if (self.scan_origin[i] > 0.3 and self.scan_origin[i]) < 4.0 and (self.scan_origin[i-1]*1.4 < self.scan_origin[i] or self.scan_origin[i-1]*0.6 > self.scan_origin[i]) and (i > 359 and i< 721):
        #         start_idx_temp = i
        #         end_idx_temp = i
        #         max_idx_temp = i
        #         i = i+1
        #         while self.scan_origin[i] > 0.3 and self.scan_origin[i-1]*1.1 > self.scan_origin[i] and self.scan_origin[i-1]*0.9 < self.scan_origin[i] and (i+1 < self.scan_range) and (i > 359 and i< 721):
        #             if self.scan_origin[i] > self.scan_origin[max_idx_temp]:
        #                 max_idx_temp = i
        #             i = i+1        
        #         i=i+1
        #         end_idx_temp = i
        #         obs_temp = [0]*4
        #         obs_temp[0] = start_idx_temp
        #         obs_temp[1] = end_idx_temp
        #         obs_temp[2] = max_idx_temp
        #         obs_temp[3] = self.scan_origin[max_idx_temp]
        #         self.scan_obs.append(obs_temp)
        #     i+=1    




    # #def obs_dect(self):
    #     #for i in range(1, self.scan_range - 1):
    #     self.scan_obs = []
    #     i=1
    #     while(self.scan_range - 1>i):
    #         start_idx_temp = i
    #         end_idx_temp = i
    #         max_idx_temp = i
    #         i = i+1
    #         while self.scan_origin[i-1]*1.4 > self.scan_origin[i] and self.scan_origin[i-1]*0.6 < self.scan_origin[i] and (i+1 < self.scan_range)  :
    #             if self.scan_origin[i] > self.scan_origin[max_idx_temp]:
    #                 max_idx_temp = i
    #             i = i+1
    #         i=i+1
    #         end_idx_temp = i
    #         obs_temp = [0]*4
    #         obs_temp[0] = start_idx_temp
    #         obs_temp[1] = end_idx_temp
    #         obs_temp[2] = max_idx_temp
    #         obs_temp[3] = self.scan_origin[max_idx_temp]
    #         self.scan_obs.append(obs_temp)
    #         i+=1  
        

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

        
        step = (int)(self.desired_wp_rt[1]/self.interval)
        ref_idx = self.front_idx + step
        self.obs = False
        for i in range(len(self.len_obs)):
            if self.len_obs[i][0] > 580 or self.len_obs[i][1] < 500:
                self.obs = False
            else:
                self.obs= True
                break


    def run(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            if self.scan_range == 0: continue
            self.obs_dect()
            #print(self.obs)
            if self.obs:
                self.obstacle_q.put(True)
            else:
                self.obstacle_q.put(False)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("test")
    global_q = Queue(1)
    local_q = Queue(1)
    od_q = Queue(1)

    global_p = global_pure(global_q)
    global_p.start()

    local_p = local_fgm(local_q)
    local_p.start()

    obstacle_p = Obstacle_detect(od_q)
    obstacle_p.start()

    A = maindrive(global_q, local_q, od_q)
    A.maindrives()
    rospy.spin()

