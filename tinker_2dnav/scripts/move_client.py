#!/usr/bin/python

import rospy
from tinker_2dnav.msg._SimpleMoveGoal import SimpleMoveGoal
from tinker_2dnav.msg._SimpleMoveAction import SimpleMoveAction
import actionlib
import sys


def init():
    rospy.init_node('move_client')
    client = actionlib.SimpleActionClient('simple_move', SimpleMoveAction)
    client.wait_for_server()
    rospy.loginfo('server connected input x, y, theta')
    while True:
        line = sys.stdin.readline()
        if len(line) <= 1:
            break
        x, y, theta = tuple([float(w) for w in line.split()])
        goal = SimpleMoveGoal()
        goal.target.x = x
        goal.target.y = y
        goal.target.theta = theta
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo(str(client.get_result()))


if __name__ == '__main__':
    init()

