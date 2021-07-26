import numpy as np
import matplotlib.pyplot as plt
import time

import rospy

class logger:
    
    def __init__(self):
        self.wp = np.genfromtxt('wp_curve.csv',delimiter=',')
        self.tr = np.genfromtxt('trajectory.csv',delimiter=',')

        self.wp_list = self.wp[:,:2]
        self.tr_list = self.tr[:,:2]
        

        self.tr_idx_current = 0
        self.nearest_dist = 0


        self.Aver_FR = 0
        self.Sum_C_X = 0
        self.Sum_C_X = 0
        self.FR = 0
        self.C_X = 0
        self.C_Y = 0

        self.recording = open('recording.csv', 'a')
        
    def calc_theta(self):
        # Waypoints has no theta, so calculation theta using arctan2
        theta = []
        for i in range(len(self.wp)-1):
            _dx = self.wp[i,0] - self.wp[i+1,0]
            _dy = self.wp[i,1] - self.wp[i+1,1]
            theta.append(np.arctan2(_dy,_dx))
        
        _dx = self.wp[-1,0] - self.wp[0,0]
        _dy = self.wp[-1,1] - self.wp[0,1]
        theta.append(np.arctan2(_dy,_dx))

        return theta
    
    def calc_FR(self):
        # To Calculation FR 
        N = len(self.wp)
        theta_i = self.tr[:,2]
        theta_v = self.calc_theta()

        div_u = 0

        for i in range(N-1):
            self.find_nearest_on_trajectory(i)
            tr_idx = self.tr_idx_current

            div_u += np.fabs(theta_v[i] - theta_i[tr_idx])
        
        FR = div_u / ((N-1)*180)


        return FR

    def get_distance(self,x,y):
        dx = x[0] - y[1]
        dy = x[1] - y[1]
        
        return np.sqrt(dx**2 + dy**2)


    def find_nearest_on_trajectory(self,idx):
        # Waypoint and Trajectory is not matching. So Find nearest point on trajectory.
        point = self.wp_list[idx]
        # print(point)
        
        idx_tmp = self.tr_idx_current
        self.nearest_dist = self.get_distance(self.tr_list[idx_tmp], point)

        while True:
            idx_tmp += 1
            if idx_tmp >= len(self.wp_list) -1:
                idx_tmp = 0
            
            tmp_dist = self.get_distance(self.tr_list[idx_tmp],point)

            if tmp_dist < self.nearest_dist:
                self.nearest_dist = tmp_dist
                self.tr_idx_current = idx_tmp
                # print(1)
            elif tmp_dist > self.nearest_dist or idx_tmp == self.tr_idx_current:
                # print(2)
                break

    def writing_inf(self):

        FR = self.calc_FR()

        print("FR : ",FR, "C_X : ", self.C_X, "C_Y : ", self.C_Y)

        # self.recording.write(f"{FR},{self.C_X},{self.C_Y}\n")
        # self.recording.close()


    # def save_inf(self):


    #     self.r = np.genfromtxt('recording.csv',delimiter=',')

    #     print(self.r)


    #     #FR Average
    
    #     FR_list = self.r[:,0]
        
    #     self.Sum_FR = sum(self.r[:,0])

    #     if len(FR_list)-1 != 0:

    #         self.Aver_FR = self.Sum_FR / (len(FR_list)-1)


    #         if len(FR_list) % 11 == 0:

    #             print(f"It's the average of {len(FR_list)-1} times")
    #             print(self.Aver_FR) 

    #     else:
    #         pass

        


        

    # def average(self):


A = logger()
A.writing_inf()
# A.save_inf()
        

