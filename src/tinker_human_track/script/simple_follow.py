#!/usr/bin/python

import sys
import rospy
from people_msgs.msg import People
from geometry_msgs.msg import PoseStamped
import tf
from math import acos, sqrt, cos, sin
from threading import Lock


pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
seq = 0
l = Lock()
pose = None
def found_people_handler(people):
    global seq, pose, l
    if len(people.people) > 0:
        l.acquire()
        person = people.people[0]
        x = person.position.x
        y = person.position.y
        if x*x + y*y < 4:
            rospy.loginfo('found human, too close')
            return
        pose = PoseStamped()
        pose.header.seq = seq
        seq += 1
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = people.header.frame_id

        pose.pose.position.z = 0
        distance = sqrt(x*x + y*y)
        theta = acos(x / distance)
        if x < 0:
            theta = -theta
        move_distance = distance - 2
        pose.pose.position.x = move_distance * cos(theta)
        pose.pose.position.y = move_distance * sin(theta)
        quat= tf.transformations.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        l.release()


def init(argv):
    global pub, pose, l
    rospy.init_node('simple_follow')
    rospy.Subscriber('tinker_laser_human_tracker/laser_found_people', People, found_people_handler)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        l.acquire()
        if pose != None:
            rospy.loginfo('move to target x %f y %f' % (pose.pose.position.x, pose.pose.position.y))
            pub.publish(pose)
            pose = None
        l.release() 
        rate.sleep()


if __name__ == '__main__':
    init(sys.argv)

