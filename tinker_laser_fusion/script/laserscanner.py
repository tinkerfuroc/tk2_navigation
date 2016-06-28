import rospy
from threading import Lock
from numpy import inf, nan, isnan, fabs
import numpy as np
from math import sin, cos, sqrt, acos, floor, pi
from sensor_msgs.msg import LaserScan
from copy import copy


class LaserScanner:
    def __init__(self, name, local_laser_param, fake_laser_param):
        self.name = name
        self.min_fake_angle = fake_laser_param['angle_min']
        self.max_fake_angle = fake_laser_param['angle_max']
        self.fake_angle_step = fake_laser_param['angle_increment']
        self.len_fake_data = int((self.max_fake_angle - self.min_fake_angle) // self.fake_angle_step)
        self.x = local_laser_param['x']
        self.y = local_laser_param['y']
        self.theta = local_laser_param['theta']
        self.is_real_time = local_laser_param['is_real_time']
        self.need_filter = local_laser_param['need_filter']
        self.range_data = np.array([inf] * self.len_fake_data)
        self.angle_limited = 'angle_min' in local_laser_param
        if self.angle_limited:
            self.angle_min = local_laser_param['angle_min']
            self.angle_max = local_laser_param['angle_max']
        if 'reverse' in local_laser_param:
            self.reverse = local_laser_param['reverse']
        self.lock = Lock()
        rospy.Subscriber(local_laser_param['topic'], LaserScan, self.laser_data_callback)


    def get_range_data(self):
        with self.lock:
            return copy(self.range_data)

    def get_timestamp(self):
        with self.lock:
            return self.timestamp

    def get_fake_polar(self, real_angle, real_radius, reverse):
        real_angle = pi - real_angle if reverse else real_angle
        fake_x = cos(real_angle) * real_radius + self.x
        fake_y = sin(real_angle) * real_radius + self.y
        fake_radius = sqrt(fake_x * fake_x + fake_y * fake_y)
        fake_angle = acos(fake_x/fake_radius)
        if fake_y < 0:
            fake_angle = -fake_angle
        return fake_radius, fake_angle

    def filtered_val(self, val, neighbours):
        if val > 20:
            return inf
        near_cnt = sum(fabs(neighbours - val) < 0.1)
        if near_cnt < len(neighbours) * 0.5:
            return inf
        return val

    def filter_range(self, range_data):
        filtered_ranges = np.array([inf] * self.len_fake_data)
        for i in xrange(10, self.len_fake_data - 10):
            filtered_ranges[i] = self.filtered_val(range_data[i], range_data[i-5:i+5])
        return filtered_ranges

    def laser_data_callback(self, laser_scan):
        with self.lock:
            self.range_data = np.array([inf] * self.len_fake_data)
            angle_step = laser_scan.angle_increment
            angle = laser_scan.angle_min
            if self.angle_limited:
                start_i = (int)((self.angle_min - laser_scan.angle_min) / angle_step)
                end_i = (int)((self.angle_max - laser_scan.angle_min) / angle_step)
                laser_scan.ranges = laser_scan.ranges[start_i:end_i]
                angle = self.angle_min
            for radius in laser_scan.ranges:
                if not isnan(radius) and radius != inf:
                    fake_radius, fake_angle = self.get_fake_polar(angle + self.theta, radius, self.reverse)
                    if np.isnan(fake_angle):
                        continue
                    i = (fake_angle - self.min_fake_angle) / self.fake_angle_step
                    i = int(round(i))
                    i %= self.len_fake_data
                    if self.range_data[i] == nan or self.range_data[i] > fake_radius:
                        self.range_data[i] = fake_radius
                angle += angle_step
            if self.need_filter:
                self.range_data = self.filter_range(self.range_data)

