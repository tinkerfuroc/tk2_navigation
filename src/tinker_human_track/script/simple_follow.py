#!/usr/bin/python

import sys
import rospy
from people_msgs.msg import People
from geometry_msgs.msg import PoseStamped
import tf
from math import acos, sqrt


pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
seq = 0
def found_people_handler(people):
    global pub, seq
    if len(people.people) > 0:
        person = people.people[0]
        pose = PoseStamped()
        pose.header.seq = seq
        seq += 1
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = people.header.frame_id
        x = person.position.x
        y = person.position.y
        pose.pose.position.x = x / 2
        pose.pose.position.y = y / 2
        pose.pose.position.z = 0
        theta = acos(x / sqrt(x*x + y*y))
        if x < 0:
            theta = -theta
        quat= tf.transformations.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        pub.publish(pose)


def init(argv):
    rospy.init_node('simple_follow')
    rospy.Subscriber('tinker_laser_human_tracker/laser_found_people', People, found_people_handler)
    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    init(sys.argv)

