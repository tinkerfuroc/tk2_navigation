import rospy
from threading import Lock
from numpy import inf, nan, isnan
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
        self.range_data = [inf] * self.len_fake_data
        self.lock = Lock()
        rospy.Subscriber(local_laser_param['topic'], LaserScan, self.laser_data_callback)


    def get_range_data(self):
        with self.lock:
            return copy(self.range_data)

    def get_timestamp(self):
        with self.lock:
            return self.timestamp

    def get_fake_polar(self, real_angle, real_radius):
        fake_x = cos(real_angle) * real_radius + self.x
        fake_y = sin(real_angle) * real_radius + self.y
        fake_radius = sqrt(fake_x * fake_x + fake_y * fake_y)
        fake_angle = acos(fake_x/fake_radius)
        if fake_y < 0:
            fake_angle = -fake_angle
        return fake_radius, fake_angle


    def laser_data_callback(self, laser_scan):
        with self.lock:
            self.range_data = [inf] * self.len_fake_data
            angle_step = laser_scan.angle_increment
            angle = laser_scan.angle_min
            for radius in laser_scan.ranges:
                if not isnan(radius) and radius != inf:
                    fake_radius, fake_angle = self.get_fake_polar(angle + self.theta, radius)
                    i = (fake_angle - self.min_fake_angle) / self.fake_angle_step
                    i = int(floor(i + 0.5))
                    i %= self.len_fake_data
                    if self.range_data[i] == nan or self.range_data[i] > fake_radius:
                        self.range_data[i] = fake_radius
                angle += angle_step


