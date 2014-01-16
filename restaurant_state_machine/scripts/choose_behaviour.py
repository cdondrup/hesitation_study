#!/usr/bin/env python

import rospy
import smach
import actionlib
from std_msgs.msg import String
from random import randint
from strands_human_aware_velocity.msg import *
from strands_gazing.msg import *

# define state ChooseBehaviour
class ChooseBehaviour(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['behaviour_set'])
        self.pub = rospy.Publisher('/study/behaviour', String)
        rospy.loginfo("Creating human_aware_planner_velocity client")
        self.behaviourClient = actionlib.SimpleActionClient('human_aware_planner_velocities', HumanAwareVelocityAction)
        self.behaviourClient.wait_for_server()
        rospy.loginfo(" ...done")
        rospy.loginfo("Creating gazing client")
        self.gazingClient = actionlib.SimpleActionClient('gaze_at_pose', GazeAtPoseAction)
        self.gazingClient.wait_for_server()
        rospy.loginfo(" ...done")

    def execute(self, usardata):
        goal = HumanAwareVelocityGoal
        toggle = randint(0,1)
        goal.seconds = 600.0
        if toggle == 0:
            rospy.loginfo("Choosing behaviour: stopping")
            self.pub.publish(String('stopping'))
            gaze = GazeAtPoseGoal
            gaze.runtime_sec   = 600.0
            self.gazingClient.send_goal(gaze)
            goal.time_to_reset = 3.0
            goal.max_vel_x     = 0.75
            goal.max_rot_vel   = 1.0
            goal.max_dist      = 6.0
            goal.min_dist      = 1.5
        else:
            rospy.loginfo("Choosing behaviour: non-stopping")
            self.pub.publish(String('non-stopping'))
            gaze = GazeAtPoseGoal
            gaze.runtime_sec   = 600.0
            self.gazingClient.send_goal(gaze)
            goal.time_to_reset = 2.0
            goal.max_vel_x     = 0.75
            goal.max_rot_vel   = 1.0
            goal.max_dist      = 0.5
            goal.min_dist      = 0.0
        self.behaviourClient.send_goal(goal)
        return 'behaviour_set'

    def request_preempt(self):
        """Overload the preempt request method."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")
        self.behaviourClient.cancel_all_goals()
