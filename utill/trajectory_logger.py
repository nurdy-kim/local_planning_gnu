import rospy
import numpy as np
import atexit
from nav_msgs.msg import Odometry

file = open('/home/nurdy/f1tenth_ws/src/local_planning_gnu/utill/trajectory.csv', 'w')

def save_trajectory(odom_msg):
    qx = odom_msg.pose.pose.orientation.x 
    qy = odom_msg.pose.pose.orientation.y 
    qz = odom_msg.pose.pose.orientation.z
    qw = odom_msg.pose.pose.orientation.w 
    
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0-2.0*(qy*qy + qz*qz)
    
    current_position_theta = np.arctan2(siny_cosp, cosy_cosp)
    current_position_x = odom_msg.pose.pose.position.x
    current_position_y = odom_msg.pose.pose.position.y

    file.write(f"{current_position_x},{current_position_y},{current_position_theta}\n")

def shutdown():
    file.close()
    print("saved!")

def listener():
    rospy.init_node('trajectory_logger', anonymous=True)
    rospy.Subscriber('/odom',Odometry, save_trajectory)
    rospy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving Trajectories..')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
