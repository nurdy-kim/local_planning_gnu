import numpy as np

class GapFollower:
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 100  # PREPROCESS_consecutive_SIZE
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees

    def __init__(self, params):
        self.radians_per_elem = None
        self.STRAIGHTS_SPEED = params.max_speed
        self.CORNERS_SPEED = params.min_speed

    def preprocess_lidar(self, ranges):

        self.radians_per_elem = (2 * np.pi) / len(ranges)
        proc_ranges = np.array(ranges[180:-180])  # 180도 봄
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)  # 오류 잡이용
        return proc_ranges

    def find_max_gap(self, free_space_ranges):

        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):

        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):

        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2

        return steering_angle

    def driving(self, scan_data, odom_data):
        ranges = scan_data
        proc_ranges = self.preprocess_lidar(ranges)
        closest = proc_ranges.argmin()

        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        gap_start, gap_end = self.find_max_gap(proc_ranges)

        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        steering_angle = self.get_angle(best, len(proc_ranges))
        if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
            speed = self.CORNERS_SPEED
        else:
            speed = self.STRAIGHTS_SPEED
        # print('Steering angle in degrees: {}'.format((steering_angle / (np.pi / 2)) * 90))
        # print(f"Speed: {speed}")
        return speed, steering_angle