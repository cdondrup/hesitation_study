#!/usr/bin/env python

import rospy
import smach
import actionlib
from random import randint
from strands_human_aware_velocity.msg import *

# define state ChooseBehaviour
class ChooseBehaviour(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['behaviour_set'])
        rospy.loginfo("Creating human_aware_planner_velocity client")
        self.behaviourClient = actionlib.SimpleActionClient('human_aware_planner_velocities', HumanAwareVelocityAction)
        self.behaviourClient.wait_for_server()
        rospy.loginfo(" ...done")

    def execute(self, usardata):
        goal = HumanAwareVelocityGoal
        toggle = randint(0,1)
        goal.seconds = 300.0
        if toggle == 0:
            goal.time_to_reset = 5.0
            goal.max_vel_x = 0.55
            goal.max_dist = 6.0
            goal.min_dist = 1.5
        else:
            goal.time_to_reset = 2.0
            goal.max_vel_x = 0.75
            goal.max_dist = 1.0
            goal.min_dist = 0.2
        self.behaviourClient.send_goal(goal)
        rospy.loginfo("Choosing behaviour: %s", toggle)
        return 'behaviour_set'

    def request_preempt(self):
        """Overload the preempt request method."""
        State.request_preempt(self)
        rospy.logwarn("Preempted!")
        self.behaviourClient.cancelAllGoals()
