import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import matplotlib.pyplot as plt
import numpy as np

class Monitor:
    def __init__(self):
        self.current_speed = 0
        self.input_speed = 0
        rospy.Subscriber('/odom', Odometry, self.Odome)
        rospy.Subscriber('/drive',AckermannDriveStamped, self.drive_msg)
    
    def Odome(self,odom_msg):
        self.current_speed = odom_msg.twist.twist.linear.x
    
    def drive_msg(self,drive_msg):
        self.input_speed = drive_msg.drive.speed
    
    def plotting(self):
        rate = rospy.Rate(100)
        
        t = np.arange(0,750)
        in_list = [0] * 750
        out_list = [0] * 750
        
        while not rospy.is_shutdown():
        
            speed_input = self.input_speed
            speed_output = self.current_speed

            del in_list[0]
            del out_list[0]
            
            t[0:-1] = t[1:]
            t[-1] = t[-1] + 1

            in_list.append(speed_input)
            out_list.append(speed_output)

            plt.plot(t, in_list, color='black', label='input')
            plt.plot(t, out_list, color='red', label='output')
            plt.legend()
            plt.grid()
            plt.pause(0.001)
            plt.clf()
        
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("monitor")
    M = Monitor()
    M.plotting()
    rospy.spin()
