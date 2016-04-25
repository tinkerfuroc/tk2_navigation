#!/usr/bin/python

import sys
import rospy
import yaml
from sensor_msgs.msg import LaserScan
from numpy import array, save, arange
from math import sin, cos
import numpy

def is_valid(loc):
    if -2.4 < loc[0]  < 2.4 and -0.4 < loc[1] < 0.4:
        return 1
    return 0

frame_id = 0
def save_loc(laser_scan):
    global frame_id
    angles = arange(laser_scan.angle_min, laser_scan.angle_max, laser_scan.angle_increment)
    assert len(angles) == len(laser_scan.ranges)
    ranges = [min(laser_scan.range_max, r) for r in laser_scan.ranges]
    locs = [[radius * cos(angle), radius * sin(angle)] for radius, angle in zip(ranges, angles)]
    result = [is_valid(loc) for loc in locs]
    print sum(result)
    if frame_id % 10 == 0:
        save('frame%d' % (frame_id/10), array(ranges, dtype='f'))
        save('result%d' % (frame_id/10), array(result, dtype=numpy.uint8))
    frame_id += 1


def init(argv):
    rospy.init_node('save_loc')
    rospy.Subscriber('/scan', LaserScan, save_loc)
    while not rospy.is_shutdown():
        pass
    

if __name__ == '__main__':
    init(sys.argv)
