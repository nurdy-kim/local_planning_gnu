#!/usr/bin/env python

import rospy
import math
import numpy as np
import copy


from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class Pure_Pursuit:
    def __init__(self):
        self.sub = rospy.Subscriber('/odom', Odometry, self.Odome, queue_size = 1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 10 )
        # self.pub_dp_mark = rospy.Publisher("/marker", Marker, queue_size = 1)
        self.ackermann_data = AckermannDriveStamped()

        # self.ackermann_data.drive.steering_angle = 0
        self.ackermann_data.drive.steering_angle = 0

        self.PI = 3.141592
        self.CURRENT_WP_CHECK_OFFSET = 2
        self.DX_GAIN = 2.5
        self.RACECAR_LENGTH = 0.325
        self.GRAVITY_ACCELERATION = 9.81

        #self.LOOKAHEAD_MAX = rospy.get_param("/pure_pursuit/driving/max_look_ahead")
        self.LOOKAHEAD_MAX = rospy.get_param('max_look_ahead', 1.9)
        #self.LOOKAHEAD_MIN = rospy.get_param("/pure_pursuit/driving/min_look_ahead")
        self.LOOKAHEAD_MIN = rospy.get_param('min_look_ahead', 0.9)
        #self.SPEED_MAX = rospy.get_param("/pure_pursuit/driving/max_speed")
        self.SPEED_MAX = rospy.get_param('max_speed', 6.0)
        #self.SPEED_MIN = rospy.get_param("/pure_pursuit/driving/min_speed")
        self.SPEED_MIN = rospy.get_param('min_speed', 1.5)
        #self.DP_ANGLE_PROPORTION = rospy.get_param("/pure_pursuit/tuning/DP_angle_proportion")
        #self.MSC_MUXSIZE = rospy.get_param("/pure_pursuit/driving/manual_speed_control/mux_size")
        self.MSC_MUXSIZE = rospy.get_param('mux_size', 0)
        #self.MU = rospy.get_param("/pure_pursuit/driving/mu")
        self.MU = rospy.get_param('mu', 1)
        #self.RATE = rospy.get_param("/pure_pursuit/driving/rate")
        self.RATE = rospy.get_param('rate', 100)

        self.waypoints = self.get_waypoint()
        self.wp_index_current = 0
        self.current_position = [0,0,0]
        self.lookahead_desired = 0
        self.steering_direction = 0
        self.goal_path_radius = 0
        self.goal_path_theta = 0
        self.actual_lookahead = 0
        self.transformed_desired_point = []
        self.desired_point = []
        self.dx = 0
        self.nearest_distance = 0
        self.manualSpeedArray = []
        
    
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
        file_wps = np.genfromtxt('/home/lab/f1tenth_ws/src/car_duri/wp_vegas.csv',delimiter=',',dtype='float')
        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0],i[1],i[2]]
            temp_waypoint.append(wps_point)

        return temp_waypoint

        
    def get_manualspeed(self):
        for i in range(self.MSC_MUXSIZE):
            region = "/region_"+str(i)
            starting_wp = rospy.get_param("/pure_pursuit/driving/manual_speed_control"+region+"/starting_wp")
            ending_wp = rospy.get_param("/pure_pursuit/driving/manual_speed_control"+region+"/ending_wp")
            max_speed = rospy.get_param("/pure_pursuit/driving/manual_speed_control"+region+"/max_speed")
            min_speed = rospy.get_param("/pure_pursuit/driving/manual_speed_control"+region+"/min_speed")

            self.manualSpeedArray.append([starting_wp, ending_wp, max_speed, min_speed])
            
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
        self.lookahead_desired = np.exp(-(self.DX_GAIN*np.fabs(self.dx) - np.log(self.LOOKAHEAD_MAX - self.LOOKAHEAD_MIN))) + self.LOOKAHEAD_MIN

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
            if(wp_index_temp >= len(self.waypoints)-1): wp_index_temp = 0
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
        
        self.ackermann_data.drive.steering_angle = self.steering_direction * steering_angle

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
        self.ackermann_data.drive.speed = np.exp(-(self.DX_GAIN*np.fabs(self.dx)-np.log(controlled_speed_max - controlled_speed_min))) + controlled_speed_min

    def setSpeed_PossibleMaximumTest(self):
        test_speed = np.sqrt(self.MU*self.GRAVITY_ACCELERATION*self.goal_path_radius)
        if test_speed > 6:
            self.setSpeed()
        else:
            self.ackermann_data.drive.speed = test_speed


    def driving(self):
        #RATE = loop_late(100)
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            
            self.find_nearest_wp()
            self.get_dx()

            self.get_lookahead_desired()
            self.find_desired_wp()

            #publishDPmarker()

            self.transformed_desired_point = self.transformPoint(self.current_position, self.desired_point)

            self.find_path()
            self.setSteeringAngle()
            self.setSpeed_PossibleMaximumTest()
            self.drive_pub.publish(self.ackermann_data)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("O")
    A = Pure_Pursuit()
    A.driving()
    rospy.spin()