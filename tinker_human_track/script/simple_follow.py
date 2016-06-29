#!/usr/bin/python

import sys
import rospy
from people_msgs.msg import People
from geometry_msgs.msg import PoseStamped, PointStamped
from tinker_msgs.msg import FollowGoal
from tinker_msgs.msg import FollowAction
from tinker_msgs.msg import FollowResult
from actionlib_msgs.msg import GoalStatus
import tf
from math import acos, sqrt, cos, sin
from threading import Lock

import actionlib

class Follower:
    def __init__(self):
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.follow_server = rospy.actionlib.SimpleActionServer('follow_action',
                            FollowAction,
                            auto_start = False)
        self.follow_server.register_goal_callback(self.goal_cb)
        self.follow_server.register_preempt_callback(self.preempt_cb)
        self.human_sub = rospy.Subscriber('tinker_vision/found_people', PointStamped, 
                self.found_people_handler)
        self.is_following = False
        self.last_loc = PointStamped()
        self.follow_server.start()

    def found_people_handler(self, loc):
        if not self.is_following:
            return
        x = loc.point.x
        y = loc.point.y
        if x == 0 and y == 0:
            result = FollowResult()
            result.last_found_loc = last_loc
            self.follow_server.set_succeeded(result)
            self.is_following = False
        self.last_loc = loc
        rospy.logdebug('x at %f, y at %f' % (x, y))
        if x*x + y*y < 2:
            rospy.logdebug('found human, too close')
            return
        pose = PoseStamped()
        pose.header.seq = seq
        seq += 1
        pose.header = loc.header
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
        self.goal_pub.publish(pose)

    def goal_cb(self):
        self.follow_server.acceptNewGoal()
        self.is_following = True

    def preempt_cb(self):
        self.follow_server.set_preempted()
        self.is_following = False


def init(argv):
    global pub, pose, l
    rospy.init_node('simple_follow')
    follow_action = Follower()
    rospy.spin()


if __name__ == '__main__':
    init(sys.argv)

