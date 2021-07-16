#!/usr/bin/env python

import rospy
import math
import numpy as np
import time

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

import matplotlib.pyplot as plt


class FGM:
    def __init__(self):

        self.ackermann_data = AckermannDriveStamped()

        self.PI = 3.141592
        self.LOOK = 2.5
        self.RACECAR_LENGTH = 0.325
        self.SPEED_MAX = rospy.get_param('max_speed', 11.0)
        self.SPEED_MIN = rospy.get_param('min_speed', 1.5)
        self.RATE = rospy.get_param('rate', 100)
        self.ROBOT_SCALE = rospy.get_param('robot_scale', 0.35)
        self.THRESHOLD = rospy.get_param('threshold', 3.0)
        self.GAP_SIZE = rospy.get_param('gap_size', 1)
        self.FILTER_SCALE = rospy.get_param('filter_scale', 1.1)
        self.GAP_THETA_GAIN = rospy.get_param('gap_theta_gain', 20.0)
        self.REF_THETA_GAIN = rospy.get_param('ref_theta_gain', 1.5)

        self.ackermann_data.drive.acceleration = 0
        self.ackermann_data.drive.jerk = 0
        self.ackermann_data.drive.steering_angle = 0
        self.ackermann_data.drive.steering_angle_velocity = 0

        self.scan_range = 0
        self.desired_gap = 0
        self.speed_gain = 0
        self.steering_gain = 0
        self.gain_cont = 0
        self.speed_cont = 0
        self.desired_wp_rt = [0, 0]

        self.speed_up = 0
        self.MU = 0.523
        self.MASS = 3.47
        self.GRAVITY_ACC = 9.81

        self.wp_num = 1
        self.waypoints = self.get_waypoint()
        self.wp_index_current = 0
        self.current_position = [0]*3
        self.nearest_distance = 0

        self.max_angle = 0
        self.wp_angle = 0

        self.gaps = []
        self.for_gap = [0, 0, 0]
        self.for_point = 0

        self.interval = 0.00435  # 1도 = 0.0175라디안
        self.front_idx = 0
        self.theta_for = self.PI/3
        self.gap_cont = 0
        self.wp_idx = 0
        self.gap_idx = 0

        self.current_speed = 1.0
        self.current_steer = 0.0
        self.start = time.time()
        

        rospy.Subscriber('/ICE/scan', LaserScan,
                         self.subCallback_scan, queue_size=10)
        rospy.Subscriber('/ICE/odom', Odometry, self.Odome, queue_size=10)
        self.drive_pub = rospy.Publisher(
            "/ICE/drive", AckermannDriveStamped, queue_size=10)

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

        # rtpoint[0] = r, [1] = theta
        rtpoint.append(np.sqrt(x*x + y*y))
        rtpoint.append(np.arctan2(y, x) - (self.PI/2))

        return rtpoint

    def get_waypoint(self):
        #file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_vegas.csv',delimiter=',',dtype='float')
        file_wps = np.genfromtxt(
            '/home/lab/f1tenth_ws/src/car_duri/wp_vegas.csv', delimiter=',', dtype='float')
        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0], i[1], 0]
            temp_waypoint.append(wps_point)
            self.wp_num += 1
        # print("wp_num",self.wp_num)
        return temp_waypoint

    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(
            self.waypoints[wp_index_temp], self.current_position)

        _vel = self.current_speed
        _LOOK = 1.0 + (_vel * 0.2)
        if _LOOK > self.LOOK:
            self.LOOK = _LOOK

        while True:
            wp_index_temp += 1

            if wp_index_temp >= self.wp_num-1:
                wp_index_temp = 0
            temp_distance = self.getDistance(
                self.waypoints[wp_index_temp], self.current_position)

            if temp_distance < self.nearest_distance:
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
            elif ((temp_distance > (self.nearest_distance + self.LOOK*1.2)) or (wp_index_temp == self.wp_index_current)):
                break

        temp_distance = self.getDistance(
            self.waypoints[1000], self.current_position)
        # if temp_distance < 1.5:
        #     #  print("time :", time.time() - self.start)

        temp_distance = 0
        idx_temp = self.wp_index_current
        while True:
            if idx_temp >= self.wp_num-1:
                idx_temp = 0
            temp_distance = self.getDistance(
                self.waypoints[idx_temp], self.current_position)
            if temp_distance > self.LOOK:
                break
            idx_temp += 1
        self.wp_idx = idx_temp
        transformed_nearest_point = self.transformPoint(
            self.current_position, self.waypoints[idx_temp])
        self.desired_wp_rt = self.xyt2rt(transformed_nearest_point)

        # marker_pub = rospy.Publisher('/marker', Marker, queue_size=10)
        # marker = Marker()
        # marker.header.frame_id = "map"
        # marker.header.stamp = rospy.Time.now()
        # marker.ns = "mpc"
        # marker.id = 2
        # marker.type = marker.CUBE
        # marker.action = marker.ADD
        # marker.pose.position.x = self.waypoints[self.desired_gap][0]
        # marker.pose.position.y = self.waypoints[self.desired_gap][1]
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
        # marker_pub.publish(marker)

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
        self.current_position = [current_position_x,
                                 current_position_y, current_position_theta]

        self.find_desired_wp()
        _speed = odom_msg.twist.twist.linear.x
        _steer = odom_msg.twist.twist.angular.z
        self.current_speed = _speed
        self.current_steer = _steer

    def subCallback_scan(self, msg_sub):
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
                for j in range(1, 21):
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
                unit_length = self.scan_origin[i]*self.interval
                filter_num = self.ROBOT_SCALE/unit_length

                j = 1
                while j < filter_num+1:
                    if i+j < self.scan_range:
                        if self.scan_filtered[i+j] > self.scan_origin[i]:
                            self.scan_filtered[i+j] = self.scan_origin[i]
                        else:
                            break
                    else:
                        break
                    j += 1

            elif self.scan_filtered[i] > self.scan_origin[i+1]*self.FILTER_SCALE:
                unit_length = self.scan_origin[i+1]*self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 0
                while j < filter_num + 1:
                    if i-j > 0:
                        if self.scan_filtered[i-j] > self.scan_origin[i+1]:
                            self.scan_filtered[i-j] = self.scan_origin[i+1]
                        else:
                            break
                    else:
                        break
                    j += 1

    def find_gap(self, scan):
        self.gaps = []
        i=359

        while True:
            if i > 819:
                break
            if scan[i] > self.THRESHOLD:
                start_idx_temp = i
                end_idx_temp = i
                max_temp = scan[i]
                max_idx_temp = i

                while ((scan[i] > self.THRESHOLD) and (i+1 < 819)):
                    i += 1
                    if scan[i] > max_temp:
                        max_temp = scan[i]
                        max_idx_temp = i
                if scan[i] > self.THRESHOLD:
                    i += 1
                end_idx_temp = i

                gap_temp = [0]*3
                gap_temp[0] = start_idx_temp
                gap_temp[1] = end_idx_temp
                gap_temp[2] = max_idx_temp
                self.gaps.append(gap_temp)
            i += 1

    def for_find_gap(self, scan):

        self.for_point = (int)(self.theta_for/self.interval)
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

    # ref - [0] = r, [1] = theta

    def find_best_gap(self, ref):
        step = (int)(ref[1]/self.interval)
        ref_idx = self.front_idx + step
        print(self.gaps, ref_idx)
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

            if self.gaps[gap_idx][0] < ref_idx and self.gaps[gap_idx][1] > ref_idx:
                self.gaps[gap_idx][2] = ref_idx
            # 가장 작은 distance를 갖는 gap만 return
            return self.gaps[gap_idx]
            # if self.gaps[0][2] > ref_idx:
            #     distance = self.gaps[0][2] - ref_idx
            # elif self.gaps[0][2] < ref_idx:
            #     distance = ref_idx - self.gaps[0][2]
            # else:
            #     distance = 0
            #     gap_idx = 0

            # i = 1
            # while (i < num):
            #     if self.gaps[i][2] > ref_idx:
            #         temp_distance = self.gaps[i][2] - ref_idx
            #         if temp_distance < distance:
            #             distance = temp_distance
            #             gap_idx = i
            #     elif self.gaps[i][2] < ref_idx:
            #         temp_distance = ref_idx - self.gaps[i][2]
            #         if temp_distance < distance:
            #             distance = temp_distance
            #             gap_idx = i

            #     else:
            #         temp_distance = 0
            #         distance = 0
            #         gap_idx = i
            #         break

            #     i += 1
            # #가장 작은 distance를 갖는 gap만 return
            # return self.gaps[gap_idx]

    def speed_controller(self, scan, steering_angle):
        # steer_deg2idx = int(steer/self.interval) + 540
        current_distance = np.fabs(np.average(scan[534:545]))

        if (np.fabs(self.steering_angle) > self.PI / 6):
            maximum_speed = (float)(-(3/self.PI)*(self.SPEED_MAX-self.SPEED_MIN)
                                    * np.fabs(self.steering_angle) + self.SPEED_MAX)
        else:
            maximum_speed = np.sqrt(
                2*self.MU * self.GRAVITY_ACC * current_distance) - 2
        # maximum_speed = np.sqrt(2*self.MU * self.GRAVITY_ACC * current_distance) - 2

        if maximum_speed >= self.SPEED_MAX:
            maximum_speed = self.SPEED_MAX

        if self.current_speed <= maximum_speed:
            set_speed = maximum_speed
        else:
            set_speed = self.current_speed - \
                np.fabs((maximum_speed - self.current_speed) * 0.05)

        return set_speed

    def main_drive(self, goal):
        self.max_angle = (goal[2] - self.front_idx)*self.interval

        controlled_angle = self.max_angle

        distance = 1.0
        # path_radius = 경로 반지름
        path_radius = distance / (2*np.sin(controlled_angle))
        #
        self.steering_angle = np.arctan(self.RACECAR_LENGTH/path_radius)

        speed = self.speed_controller(self.scan_filtered, self.steering_angle)

        self.ackermann_data.drive.steering_angle = self.steering_angle 
        self.ackermann_data.drive.steering_angle_velocity = 0
        self.ackermann_data.drive.speed = speed

        self.ackermann_data.drive.acceleration = 0
        self.ackermann_data.drive.jerk = 0

        #print("curr : ", self.current_speed, "input : ", speed, "steer :", self.steering_angle)

        self.drive_pub.publish(self.ackermann_data)
        self.speed_gain = 0
        self.steering_gain = 0
        self.gain_cont = 0

    def driving(self):
        rate = rospy.Rate(self.RATE)

        self.steering_input = [0]*100
        self.steering_output = [0]*100
        self.speed_input = [0]*100
        self.speed_output = [0]*100

        self.s = np.arange(100)

        while not rospy.is_shutdown():

            if self.scan_range == 0:
                continue

            self.find_gap(self.scan_filtered)
            self.for_find_gap(self.scan_filtered)

            self.desired_gap = self.find_best_gap(self.desired_wp_rt)

            self.main_drive(self.desired_gap)

            # ackermann_angle = self.ackermann_data.drive.steering_angle
            # ackermann_speed = self.ackermann_data.drive.speed
            # print("wp : ", self.wp_index_current)

            #marker_pub = rospy.Publisher('/marker', Marker, queue_size = 10)
            #marker = Marker()
            #marker.header.frame_id = "map"
            #marker.header.stamp = rospy.Time.now()
            #marker.ns = "mpc"
            #marker.id = 2
            #marker.type = marker.CUBE
            #marker.action = marker.ADD
            #marker.pose.position.x = self.waypoints[self.wp_idx][0]
            #marker.pose.position.y = self.waypoints[self.wp_idx][1]
            #marker.pose.position.z = 0.1
            #marker.pose.orientation.x = 0.0
            #marker.pose.orientation.y = 0.0
            #marker.pose.orientation.z = 0.0
            #marker.pose.orientation.w = 1.0
            #marker.scale.x = 0.2
            #marker.scale.y = 0.2
            #marker.scale.z = 0.1
            #marker.color.a = 1.0
            #marker.color.r = 1.0
            #marker.color.g = 0.0
            #marker.color.b = 0.0
            # marker_pub.publish(marker)

            # self.steering_input.pop(0)
            # self.steering_output.pop(0)
            # self.speed_input.pop(0)
            # self.speed_output.pop(0)
            # self.steering_input.append(ackermann_angle)
            # self.steering_output.append(self.current_steer)
            # self.speed_input.append(ackermann_speed)
            # self.speed_output.append(self.current_speed)

            # plt.subplot(1,2,1)
            # plt.plot(self.s,self.speed_input,linewidth=3.0, c='r',linestyle='-' ,label='input')
            # plt.plot(self.s,self.speed_output,linewidth=3.0, c='black', linestyle='dotted',label='output')
            # plt.ylim([0, 16])
            # plt.legend(loc='upper left')
            # plt.subplot(1,2,2)
            # plt.plot(self.s,self.steering_input,linewidth=3.0, c='r',linestyle='-',label='input')
            # plt.ylim([-0.8, 0.8])
            # plt.plot(self.s,self.steering_output,linewidth=3.0, c='black',linestyle='dotted',label='output')
            # plt.ylim([-0.8, 0.8])
            # plt.legend(loc='upper right')
            # plt.pause(0.0001)
            # plt.clf()

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("test")
    A = FGM()
    A.driving()
    rospy.spin()
