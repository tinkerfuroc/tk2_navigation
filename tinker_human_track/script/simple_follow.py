#!/usr/bin/python

import sys
import rospy
import actionlib
from people_msgs.msg import People
from geometry_msgs.msg import PoseStamped, Point 
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
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.follow_server = actionlib.SimpleActionServer('follow_action',
                            FollowAction,
                            auto_start = False)
        self.follow_server.register_goal_callback(self.goal_cb)
        self.follow_server.register_preempt_callback(self.preempt_cb)
        self.human_sub = rospy.Subscriber('tk2_vision/human_center', Point, 
                self.found_people_handler)
        self.is_following = False
        self.last_loc = Point()
        self.last_time = rospy.Time.now()
        self.follow_server.start()
        self.seq = 0

    def move_to(self, x, y, dis):
        distance = sqrt(x*x + y*y)
        if distance < 0.001:
            theta = 0
        else:
            theta = acos(x / distance)
        if x < 0:
            theta = -theta
        move_distance = distance - dis
        x = move_distance * cos(theta)
        y = move_distance * sin(theta)
        pose = PoseStamped()
        pose.header.seq = self.seq
        self.seq += 1
        pose.header.frame_id = 'base_link'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.z = 0
        pose.pose.position.x = move_distance * cos(theta)
        pose.pose.position.y = move_distance * sin(theta)
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.goal_pub.publish(pose)

    def found_people_handler(self, loc):
        if not self.is_following:
            return
        x = loc.x
        y = loc.y
        if x == 0 and y == 0:
            result = FollowResult()
            rospy.loginfo('[Simple follow] lost')
            result.last_found_loc.point = self.last_loc
            result.last_found_loc.header.stamp = self.last_time
            result.last_found_loc.header.frame_id = 'base_link'
            self.follow_server.set_succeeded(result)
            self.is_following = False
            self.move_to(0, 0, 0)
            pose = PoseStamped()
            pose.header.seq = self.seq
            self.seq += 1
            pose.header.frame_id = 'base_link'
        self.last_loc = loc
        self.last_time = rospy.Time.now()
        rospy.logdebug('x at %f, y at %f' % (x, y))
        if x*x + y*y < 4:
            rospy.logdebug('found human, too close')
            return
        self.move_to(x, y, -2)

    def goal_cb(self):
        self.follow_server.accept_new_goal()
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

