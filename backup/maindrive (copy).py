#!/usr/bin/env python

import queue
import rospy
import numpy as np
import threading
import matplotlib.pyplot as plt
from queue import Queue
import

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

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
            else:           #global
                print("Global")
                ackermann = self.global_q.get()
                self.ackermann_data.drive.steering_angle = ackermann[1]
                self.ackermann_data.drive.speed = ackermann[0]
            self.drive_pub.publish(self.ackermann_data)

            #s1.pop(0)
            #s2.pop(0)
            #s1.append(ackermann[0])
            #s2.append(ackermann[1])
            #plt.subplot(1,2,1)
            #plt.plot(s,s1,linewidth=3.0)
            #plt.ylim([0, 10])
            #plt.subplot(1,2,2)
            #plt.plot(s,s2,linewidth=3.0)
            #plt.ylim([-0.8, 0.8])
            #plt.pause(0.001)
            #plt.clf()

            rate.sleep()

class global_pure(threading.Thread):
    def __init__(self, global_q):
        super(global_pure, self).__init__()
        self.global_q = global_q
        self.wp_num = 1
        self.waypoints = self.get_waypoint()
        self.wp_index_current = 0
        self.current_position = [0] * 3
        self.lookahead_desired = 0
        self.steering_direction = 0
        self.actual_lookahead = 0
        self.transformed_desired_point = []
        self.desired_point = []
        self.nearest_distance = 0
        self.PI = 3.141592
        self.rf_distance = 2.5
        self.desired_wp_rt = [0, 0]
        self.tf_point = [0] * 3
        self.RACECAR_LENGTH = 0.325
        self.GRAVITY_ACCELERATION = 9.81
        self.MU = 1
        self.ackermann_data = AckermannDriveStamped()
        self.SPEED_MAX = rospy.get_param('max_speed', 9.0)
        self.SPEED_MIN = rospy.get_param('min_speed', 2.0)
        self.MSC_MUXSIZE = rospy.get_param('mux_size', 0)
        self.DX_GAIN = 2.5
        self.manualSpeedArray = []
        self.dx = 0
        self.CURRENT_WP_CHECK_OFFSET = 2
        self.LOOKAHEAD_MAX = rospy.get_param('max_look_ahead', 1.9)
        #self.LOOKAHEAD_MIN = rospy.get_param("/pure_pursuit/driving/min_look_ahead")
        self.LOOKAHEAD_MIN = rospy.get_param('min_look_ahead', 0.9)
        self.RATE = 100
        rospy.Subscriber('/ICE/odom', Odometry, self.Odome, queue_size=10)
        
    def run(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            self.find_nearest_wp()
            self.get_dx()

            self.get_lookahead_desired()
            self.find_desired_wp()

            self.transformed_desired_point = self.transformPoint(self.current_position, self.desired_point)

            self.find_path()
            rate.sleep()

    def find_lookahead_wp(self, length):
        wp_index_temp = self.wp_index_current
        while(1):
            if(wp_index_temp >= len(self.waypoints)-1): wp_index_temp = 0
            distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

            if(distance >= length): break
            wp_index_temp+=1
        return self.waypoints[wp_index_temp]

    def get_waypoint(self):
        #         file_wps = np.genfromtxt('/home/sumin/f1tenth_ws/src/sumim/wp_vegas.csv',delimiter=',',dtype='float')
        file_wps = np.genfromtxt('/home/lab/f1tenth_ws/src/car_duri/wp_vegas.csv', delimiter=',', dtype='float')
        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0], i[1], 0]
            # //print(wps_point)
            temp_waypoint.append(wps_point)
            self.wp_num += 1
        return temp_waypoint

    def Odome(self, odom_msg):
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        current_position_theta = np.arctan2(siny_cosp, cosy_cosp)
        current_position_x = odom_msg.pose.pose.position.x
        current_position_y = odom_msg.pose.pose.position.y
        self.current_position = [current_position_x, current_position_y, current_position_theta]


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
    
    def get_lookahead_desired(self):
        self.lookahead_desired = np.exp(-(self.DX_GAIN*np.fabs(self.dx) - np.log(self.LOOKAHEAD_MAX - self.LOOKAHEAD_MIN))) + self.LOOKAHEAD_MIN

    def get_dx(self):

        wp_min = self.find_lookahead_wp(self.LOOKAHEAD_MIN)
        wp_max = self.find_lookahead_wp(self.LOOKAHEAD_MAX)

        wp_min = self.transformPoint(self.current_position, wp_min)
        wp_max = self.transformPoint(self.current_position, wp_max)

        self.dx = wp_max[0] - wp_min[0]


    def getDistance(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]

        return np.sqrt(dx ** 2 + dy ** 2)

    def xyt2rt(self, origin):
        rtpoint = []

        x = origin[0]
        y = origin[1]

        # rtpoint[0] = r, [1] = theta
        rtpoint.append(np.sqrt(x * x + y * y))
        rtpoint.append(np.arctan2(y, x) - (self.PI / 2))

        return rtpoint

    def transformPoint(self, origin, target):
        theta = self.PI / 2 - origin[2]

        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        dtheta = target[2] + theta

        tf_point_x = dx * np.cos(theta) - dy * np.sin(theta)
        tf_point_y = dx * np.sin(theta) + dy * np.cos(theta)
        tf_point_theta = dtheta
        tf_point = [tf_point_x, tf_point_y, tf_point_theta]
        self.tf_point = [tf_point_x, tf_point_y, tf_point_theta]

        return tf_point

    def find_path(self):
        # right cornering
        if self.transformed_desired_point[0] > 0:
            self.goal_path_radius = pow(self.actual_lookahead, 2)/(2*self.transformed_desired_point[0])
            self.steering_direction = -1

        # left cornering
        else:
            self.goal_path_radius = pow(self.actual_lookahead, 2)/((-2)*self.transformed_desired_point[0])
            self.steering_direction = 1

        steering_angle = np.arctan2(self.RACECAR_LENGTH, self.goal_path_radius) * self.steering_direction

        test_speed = np.sqrt(self.MU * self.GRAVITY_ACCELERATION * self.goal_path_radius)
        if test_speed > 6:
            controlled_speed_max = self.SPEED_MAX
            controlled_speed_min = self.SPEED_MIN
            for i in range(0, self.MSC_MUXSIZE):
                if (self.wp_index_current > self.manualSpeedArray[i][0]) and (
                        self.wp_index_current < self.manualSpeedArray[i][1]):
                    controlled_speed_max = self.manualSpeedArray[i][2]
                    controlled_speed_min = self.manualSpeedArray[i][3]
                    break
            speed = np.exp(-(self.DX_GAIN * np.fabs(self.dx) - np.log(controlled_speed_max - controlled_speed_min))) + controlled_speed_min
            ackermann = [speed, steering_angle]
            self.global_q.put(ackermann)
        else:
            speed = test_speed
            #speed, steering_angle
            ackermann = [speed, steering_angle]
            self.global_q.put(ackermann)

class local_fgm(threading.Thread):
    def __init__(self, local_q):
        super(local_fgm, self).__init__()
        rospy.Subscriber('/ICE/scan', LaserScan, self.subCallback_scan, queue_size=10)
        rospy.Subscriber('/ICE/odom', Odometry, self.Odome, queue_size=10)
        self.local_q = local_q

        self.rep_count = 0
        self.ackermann_data = AckermannDriveStamped()
        self.PI = 3.141592
        self.MU = 0.523
        self.MASS = 3.47
        self.GRAVITY_ACC = 9.81
        self.LOOK = 5
        self.SPEED_MAX = 15.0
        self.SPEED_MIN = 4.0
        self.RATE = 100
        self.ROBOT_SCALE = 0.25
        self.THRESHOLD = 3.0
        self.FILTER_SCALE = 1.3
        self.scan_range = 0
        self.desired_wp_rt = [0,0]
        
        self.front_idx = 540
        self.detect_range_s = 479
        self.detect_range_e = 599
        self.safety_threshold = 0
        self.min_idx = 0
        self.f_rep_past_list =[0]*1080
        self.fullbraking_Flag = False
        self.start = time.time()
        #self.p = 0.1
        self.w = 0.9
        self.d = 0.05
        self.i = 1
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
        self.gamma = 0.5
        #self.a_k = 1.2
        self.current_speed = 1.0
        self.alpha = 0.9

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
        print("wp_num",self.wp_num)
        return temp_waypoint

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
        _speed = odom_msg.twist.twist.linear.x
        _steer = odom_msg.twist.twist.angular.z
        self.current_speed = _speed
        self.set_steering = _steer
    
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
        return self.min_idx

    def angle(self, f_total_list):

        min_f = f_total_list[0]*self.scan_range
        min_f_idx = self.detect_range_s

        for i in range(self.detect_range_s + 1, self.detect_range_e-1):
            if min_f > f_total_list[i]:
                min_f = f_total_list[i]
                min_f_idx = i

        return min_f_idx
    
    def speed_controller(self, scan, steering_angle):
        # steer_deg2idx = int(steer/self.interval) + 540
        current_distance = np.fabs(scan[539])
        
        if (np.fabs(self.steering_angle) > self.PI / 6):
            maximum_speed = (float)(-(3/self.PI)*(self.SPEED_MAX-self.SPEED_MIN)*np.fabs(self.steering_angle)+ self.SPEED_MAX)
        else:
            maximum_speed = np.sqrt(2*self.MU * self.GRAVITY_ACC * current_distance) - 2    

        if maximum_speed >= self.SPEED_MAX:
            maximum_speed = self.SPEED_MAX
        
        if self.current_speed <= maximum_speed:
            set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed) * 0.15)
        else:
            set_speed = self.current_speed - np.fabs((maximum_speed - self.current_speed) * 0.1)
        
        return set_speed
        
    def main_drive(self, goal):

        self.steering_angle = (-self.front_idx+goal)*self.interval

        # if (np.fabs(self.steering_angle) > self.PI/6):
        #     speed = self.SPEED_MIN
        # else:
        #     # front_range = np.sum(self.scan_filtered[539:542])/len(self.scan_filtered[539:542])
        #     # speed = (float)(-(3/self.PI)*(self.SPEED_MAX-self.SPEED_MIN)*np.fabs(front_range) + self.SPEED_MAX)
        #     speed = (float)(-(3/self.PI)*(self.SPEED_MAX-self.SPEED_MIN)*np.fabs(self.steering_angle)+ self.SPEED_MAX)
        #     speed = np.fabs(speed)

        steering_angle = self.steering_angle - self.d*(self.steering_angle - self.steering_angle_past) - self.i*(self.current_position[2] - self.current_position_past)#- self.d*(self.steering_angle - self.steering_angle_past)

        max_speed = self.speed_controller(self.scan_filtered, steering_angle) # determin_speed
        
        self.current_position_past = self.current_position[2]
        self.steering_angle_past = steering_angle
        #self.error_past = error
        self.f_rep_past_list = self.f_rep_list

        ackermann=[max_speed, steering_angle]
        self.local_q.put(ackermann)


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

        self.RATE = 100
        rospy.Subscriber('/ICE/scan', LaserScan, self.subCallback_od, queue_size=10)
    
    def subCallback_od(self, msg_sub):
        self.interval = msg_sub.angle_increment
        self.scan_range = len(msg_sub.ranges)
        self.front_idx = (int)(self.scan_range/2)
        
        self.scan_origin = [0]*self.scan_range
        self.scan_obs = []
    
    

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
            

        
        for i in range(1,self.scan_range - 1):
            if (self.scan_origin[i] > 0.3 and self.scan_origin[i]) < 4.0 and (self.scan_origin[i-1]*1.4 < self.scan_origin[i] or self.scan_origin[i-1]*0.6 > self.scan_origin[i]) and (i > 359 and i< 721):
                start_idx_temp = i
                end_idx_temp = i
                max_idx_temp = i
                i = i+1
                while self.scan_origin[i] > 0.3 and self.scan_origin[i-1]*1.1 > self.scan_origin[i] and self.scan_origin[i-1]*0.9 < self.scan_origin[i] and (i+1 < self.scan_range) and (i > 359 and i< 721):
                    if self.scan_origin[i] > self.scan_origin[max_idx_temp]:
                        max_idx_temp = i
                    i = i+1        
                i=i+1
                end_idx_temp = i
                obs_temp = [0]*4
                obs_temp[0] = start_idx_temp
                obs_temp[1] = end_idx_temp
                obs_temp[2] = max_idx_temp
                obs_temp[3] = self.scan_origin[max_idx_temp]
                self.scan_obs.append(obs_temp)
            i+=1    

    def run(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            if self.scan_range == 0: continue
            if len(self.scan_obs)==0:
                self.obstacle_q.put(False)
            else:
                self.obstacle_q.put(True)
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

