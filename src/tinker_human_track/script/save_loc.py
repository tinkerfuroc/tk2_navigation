#!/usr/bin/python

import sys
import rospy
import yaml
from sensor_msgs.msg import LaserScan
from numpy import array, save, arange
from math import sin, cos


frame_id = 0
def save_loc(laser_scan):
    global frame_id
    angles = arange(laser_scan.angle_min, laser_scan.angle_max - laser_scan.angle_increment, laser_scan.angle_increment)
    assert len(angles) == len(laser_scan.ranges)
    loc = [[radius * cos(angle), radius * sin(angle)] for radius, angle in zip(laser_scan.ranges, angles)]
    if frame_id % 100 == 0:
        save('frame%d' % (frame_id/100), array(loc))
    frame_id += 1


def init(argv):
    rospy.init_node('save_loc')
    rospy.Subscriber('/scan', LaserScan, save_loc)
    while not rospy.is_shutdown():
        pass
    

if __name__ == '__main__':
    init(sys.argv)
