#!/usr/bin/env python

import rospy
import smach

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *

# define state MoveRobot
class MoveRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                outcomes = ['reached_kitchen', 'reached_table', 'failure'],
                output_keys = ['goal_pose'],
                input_keys = ['goal_pose'])
	rospy.loginfo("Creating base movement client.")
        self.baseClient = actionlib.SimpleActionClient(
            'move_base',
            MoveBaseAction
        )
        self.baseClient.wait_for_server()
        rospy.loginfo("Base client initialized")

    def execute(self, userdata):
        #TODO: Have a failure state
        rospy.loginfo('Moving robot to goal: %s\n%s', userdata.goal_pose[0], userdata.goal_pose[1])

        if rospy.is_shutdown(): # Exiting gracefully when ctrl-c is pressed
            return 'abort'

        self.baseClient.send_goal(userdata.goal_pose[1])
        self.baseClient.wait_for_result()
        if self.baseClient.get_state() != GoalStatus.SUCCEEDED:
            return 'failure'

        rospy.sleep(rospy.Duration.from_sec(0.3)) #avoid jumping out of a state immediately after entering it - actionlib bug

        if userdata.goal_pose[0] == 'kitchen':
            return 'reached_kitchen'
        else:
            return 'reached_table'
