#!/usr/bin/env python

import queue
import rospy
import numpy as np
import threading
import matplotlib.pyplot as plt
from queue import Queue

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
        self.SPEED_MAX = rospy.get_param('max_speed', 10.1)
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
        self.local_q = local_q
        self.interval = 0.00435
        self.scan_range = 0
        self.front_idx = 0

        self.scan_origin = []
        self.scan_filtered =[]
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
        self.SPEED_MAX = 5.7
        self.SPEED_MIN = 1.9
        self.RATE= 100
        self.gap_cont = 0
        rospy.Subscriber('/ICE/scan', LaserScan, self.subCallback_scan, queue_size=10)

    def run(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            if self.scan_range == 0: continue
            self.find_gap(self.scan_filtered)
            self.for_find_gap(self.scan_filtered)
            gap = self.find_best_gap()
            self.main_drive(gap)
            rate.sleep()

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

    def find_best_gap(self):
        ##print(self.gaps)
        num = len(self.gaps)
        if num == 0:
            return self.for_gap
        else:
            ref_idx = self.front_idx 

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
            return self.gaps[gap_idx]

    def main_drive(self, goal):
        # goal - [2] = max_idx,
        self.max_angle = (goal[2] - self.front_idx) * self.interval

        # range_min_values = [0]*10

        controlled_angle = self.max_angle
        distance = 1.5
        path_radius = distance / (2 * np.sin(controlled_angle))
        steering_angle = np.arctan(self.RACECAR_LENGTH / path_radius)


        if (np.fabs(steering_angle) > self.PI / 8):
            speed = self.SPEED_MIN

        else:
            speed = (float)(-(3 / self.PI) * (self.SPEED_MAX - self.SPEED_MIN) * np.fabs(self.max_angle) + self.SPEED_MAX)
            speed = np.fabs(speed)

        if speed < self.SPEED_MIN:
            speed = self.SPEED_MIN

        ackermann=[speed, steering_angle]
        self.local_q.put(ackermann)
        #speed, steering_angle

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
