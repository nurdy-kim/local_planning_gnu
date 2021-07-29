import numpy as np


class ODGPF:
    def __init__(self, params):

        self.RACECAR_LENGTH = params.robot_length
        self.ROBOT_LENGTH = params.robot_length
        self.SPEED_MAX = params.max_speed
        self.SPEED_MIN = params.min_speed

        self.MU = params.mu
        self.GRAVITY_ACC = params.g
        self.PI = params.pi
        self.ROBOT_SCALE = params.robot_scale

        self.THRESHOLD = params.odg['threshold']
        self.FILTER_SCALE = params.odg['filter_scale']

        self.waypoint_real_path = params.wpt_path
        self.waypoint_delimeter = params.wpt_delimeter

        self.rep_count = 0
        self.scan_range = 0
        self.desired_wp_rt = [0, 0]

        self.front_idx = 539
        self.detect_range_s = 299
        self.detect_range_e = 779
        self.detect_range = self.detect_range_e - self.detect_range_s
        self.detect_n = 5

        self.safety_threshold = 0
        self.min_idx = 0
        self.f_rep_past_list = [0] * 1080

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

        self.nearest_distance = 0

        self.current_position = [0, 0, 0]
        self.interval = 0.00435
        self.gamma = 0.5

        self.current_speed = 1.0
        self.set_speed = 0.0
        self.alpha = 0.9
        self.mode = 0
        self.idx_temp = 0
        self.lap = 0

    def getDistance(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]

        return np.sqrt(dx ** 2 + dy ** 2)

    def find_nearest_obs(self, obs):
        min_di = 0
        min_dv = 0
        if len(obs) <= 1:
            min_di = 20
            min_dv = 20
        else:
            min_di = self.getDistance(self.current_position, obs[0])
            for i in range(len(obs)):
                _dist = self.getDistance(self.current_position, obs[i])
                if _dist <= min_di:
                    min_di = _dist
                    min_dv = self.getDistance(self.waypoints[self.wp_index_current], obs[i])

        self.closest_obs_dist = min_di
        self.closest_wp_dist = min_dv

    def transformPoint(self, origin, target):
        theta = self.PI / 2 - origin[2]
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
        rtpoint.append(np.sqrt(x * x + y * y))
        rtpoint.append(np.arctan2(y, x) - (self.PI / 2))

        return rtpoint

    def get_waypoint(self):
        file_wps = np.genfromtxt(self.waypoint_real_path, delimiter=self.waypoint_delimeter, dtype='float')

        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0], i[1], 0]
            temp_waypoint.append(wps_point)
            self.wp_num += 1

        return temp_waypoint

    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

        _vel = self.current_speed

        # self.LOOK = 1.5 + (0.3 * _vel)
        self.LOOK = 0.5 + (0.5 * _vel)

        while True:
            wp_index_temp += 1

            if wp_index_temp >= self.wp_num - 1:
                wp_index_temp = 0
                # print(self.lap_time)

            temp_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

            if temp_distance < self.nearest_distance:
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
            elif ((temp_distance > (self.nearest_distance + self.LOOK * 1.2)) or (
                    wp_index_temp == self.wp_index_current)):
                break

        temp_distance = 0
        idx_temp = self.wp_index_current
        while True:
            if idx_temp >= self.wp_num - 1:
                idx_temp = 0
            temp_distance = self.getDistance(self.waypoints[idx_temp], self.current_position)
            if temp_distance > self.LOOK: break
            idx_temp += 1

        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[idx_temp])
        self.desired_wp_rt = self.xyt2rt(transformed_nearest_point)

        self.idx_temp = idx_temp

    def define_obstacles(self, scan):
        obstacles = []

        i = self.detect_range_s
        d_i = 0
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

                while ((scan[i] < self.THRESHOLD) and (i + 1 < self.detect_range_e)):  # self.scan_range
                    i += 1
                    end_temp += scan[i]
                    obstacle_count += 1
                    if scan[i] > max_temp:
                        max_temp = scan[i]
                        max_idx_temp = i
                if scan[i] < self.THRESHOLD:
                    i += 1
                end_idx_temp = i

                # print('start:', start_idx_temp,'end:',end_idx_temp, end=" ")

                distance_obstacle = end_temp / obstacle_count

                a_k = ((self.THRESHOLD - distance_obstacle) * np.exp(1 / 2))

                angle_obstacle = (end_idx_temp - start_idx_temp) * self.interval

                sigma_obstacle = np.arctan2((distance_obstacle * np.tan(angle_obstacle / 2) + (self.ROBOT_SCALE / 2)),
                                            distance_obstacle)

                angle_obstacle_center = (int)((end_idx_temp - start_idx_temp) / 2) + start_idx_temp
                angle_obstacle_center = angle_obstacle_center - self.front_idx

                obstacle_inf = [angle_obstacle_center, sigma_obstacle, a_k]

                # print('angle_center',angle_obstacle_center,end=' ')
                # print(sigma_obstacle)
                obstacles.append(obstacle_inf)

            i += 1

        # print(len(obstacles))
        # print()

        return obstacles

    def rep_field(self, obstacles):

        f_rep_list = [0] * self.scan_range  # np.zeros(self.scan_range)
        for i in range(len(obstacles)):
            for j in range(self.detect_range_s, self.front_idx):
                f_rep_list[j] += obstacles[i][2] * np.exp((-0.5) * (
                            (((j - self.front_idx) * self.interval - obstacles[i][0] * self.interval) ** 2) / (
                    obstacles[i][1]) ** 2))

            for k in range(self.detect_range_e, self.front_idx - 1, -1):
                f_rep_list[k] += obstacles[i][2] * np.exp((-0.5) * (
                            (((k - self.front_idx) * self.interval - obstacles[i][0] * self.interval) ** 2) / (
                    obstacles[i][1]) ** 2))

        self.f_rep_list = f_rep_list

        # reversed(f_rep_list)
        return f_rep_list

    def att_field(self, goal_point):

        f_att_list = []
        for i in range(self.scan_range):
            idx2deg = (-self.front_idx + i) * self.interval
            f_att = self.gamma * np.fabs(goal_point[1] - idx2deg)
            f_att_list.append(f_att)

        return f_att_list

    def total_field(self, f_rep_list, f_att_list):

        f_total_list = [0] * self.scan_range

        for i in range(self.scan_range):
            f_total_list[i] = f_rep_list[i] + f_att_list[i]

        self.min_idx = np.argmin(f_total_list[self.detect_range_s:self.detect_range_e]) + self.detect_range_s

        self.f_total_list = f_total_list
        return self.min_idx

    def angle(self, f_total_list):

        min_f = f_total_list[0] * self.scan_range
        min_f_idx = self.detect_range_s

        for i in range(self.detect_range_s + 1, self.detect_range_e - 1):
            if min_f > f_total_list[i]:
                min_f = f_total_list[i]
                min_f_idx = i

        return min_f_idx

    def speed_controller(self):
        current_distance = np.fabs(np.average(self.scan_filtered[499:580]))
        if np.isnan(current_distance):
            print("SCAN ERR")
            current_distance = 1.0

        if self.current_speed > 10:
            current_distance -= self.current_speed * 0.7

        maximum_speed = np.sqrt(2 * self.MU * self.GRAVITY_ACC * np.fabs(current_distance)) - 2

        if maximum_speed >= self.SPEED_MAX:
            maximum_speed = self.SPEED_MAX

        if self.current_speed <= maximum_speed:
            # ACC
            if self.current_speed >= 10:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed))
            else:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed) * self.ROBOT_LENGTH)
        else:
            # set_speed = 0
            set_speed = self.current_speed - np.fabs((maximum_speed - self.current_speed) * 0.2)

        return set_speed

    def main_drive(self, goal):

        self.steering_angle = (-self.front_idx + goal) * self.interval

        controlled_angle = self.steering_angle

        if controlled_angle == 0.0:
            controlled_angle = 0.001

        # LOOK : 0.5 + (0.3 * _vel)   (장애물 or 곡선 part) == 2
        # LOOK이 path_radius에 끼치는 영향
        # -> LOOK이 클수록 스티어링 앵글을 덜꺾음 
        path_radius = self.LOOK ** 1.25 / (2 * np.sin(controlled_angle))
        steering_angle = np.arctan(self.ROBOT_LENGTH / path_radius)
        # print("input",controlled_angle,"output",steering_angle)

        self.set_speed = self.speed_controller()  # determin_speed

        if np.fabs(self.steering_angle) > 0.5:
            # print("in")
            if np.fabs(self.steering_angle_past - self.steering_angle) > 0.5:
                steering_angle = self.steering_angle_past  # ((self.steering_angle+self.steering_angle_past*(0.5))/2)
                # print("to")

        self.current_position_past = self.current_position[2]
        self.steering_angle_past = steering_angle
        self.f_rep_past_list = self.f_rep_list

        return steering_angle, self.set_speed

    def subCallback_scan(self, scan_data):

        self.scan_range = len(scan_data)

        self.front_idx = (int(self.scan_range / 2))

        self.scan_origin = [0] * self.scan_range
        self.scan_filtered = [0] * self.scan_range

        for i in range(self.scan_range):
            self.scan_origin[i] = scan_data[i]
            self.scan_filtered[i] = scan_data[i]

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

    def driving(self, scan_data, odom_data):

        self.subCallback_scan(scan_data)
        self.current_position = [odom_data['x'], odom_data['y'], odom_data['theta']]
        self.current_speed = odom_data['linear_vel']
        self.find_desired_wp()

        obstacles = self.define_obstacles(self.scan_filtered)
        self.find_nearest_obs(obstacles)
        rep_list = self.rep_field(obstacles)
        att_list = self.att_field(self.desired_wp_rt)
        total_list = self.total_field(rep_list, att_list)

        desired_angle = total_list  # self.angle(total_list)
        steer, speed = self.main_drive(desired_angle)

        return speed, steer
