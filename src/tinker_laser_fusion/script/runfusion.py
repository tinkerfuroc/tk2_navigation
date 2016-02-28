#!/usr/bin/python

import sys
import rospy
import yaml
from laserscanner import LaserScanner
from sensor_msgs.msg import LaserScan
from numpy import nan
from threading import Thread

RATE = 40.

def checker(fake_laser_param, laser_scanners):
    r = rospy.Rate(RATE)
    seq = 0
    laser_scan = LaserScan()
    laser_scan.header.seq = seq
    laser_scan.header.frame_id = fake_laser_param['frame_name']
    laser_scan.angle_min = fake_laser_param['angle_min']
    laser_scan.angle_max = fake_laser_param['angle_max']
    laser_scan.angle_increment = fake_laser_param['angle_increment']
    laser_scan.range_min = fake_laser_param['range_min']
    laser_scan.range_max = fake_laser_param['range_max']
    laser_scan.scan_time = 0
    laser_scan.time_increment = 0
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    while not rospy.is_shutdown():
        fake_laser_data = laser_scanners[0].get_range_data()
        for laser_scanner in laser_scanners[1:]:
            new_laser_data = laser_scanner.get_range_data()
            fake_laser_data = [min(r1, r2) for r1, r2 in zip(fake_laser_data, new_laser_data)]
        laser_scan.ranges = fake_laser_data
        laser_scan.header.stamp = rospy.Time.now()
        pub.publish(laser_scan)
        seq += 1
        r.sleep()


def init(argv):
    rospy.init_node('laser_fusion')
    with open(rospy.get_param('~fake_laser_param_file')) as f:
        fake_laser_param = yaml.load(f.read())
    with open(rospy.get_param('~real_laser_param_file')) as f:
        laser_param = yaml.load(f.read())
    laser_scanners =  [LaserScanner(name, laser_param, fake_laser_param) \
            for name, laser_param in laser_param.iteritems()]
    checker(fake_laser_param, laser_scanners)


if __name__ == '__main__':
    init(sys.argv)
