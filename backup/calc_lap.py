import numpy as np

file_wps = np.genfromtxt('/home/lab/f1tenth_ws/src/car_duri/wp_vegas1.csv', delimiter=',', dtype='float')
past = [0,0,0]
range = 0
for i in file_wps:
    dx = i[0] - past[0]
    dy = i[1] - past[1]

    dist = np.sqrt(dx**2 + dy**2)

    range += dist
    #print(range)
    past = i

print(range)