#!/usr/bin/env python

import rospy
import smach
from random import randint
import actionlib
from ros_mary_tts.msg import *

# define state TakeOrder
class TakeOrder(smach.State):
    """Just sleeping for a few seconds. Might be more sophisticated stuff coming."""
    def __init__(self):
       smach.State.__init__(self, outcomes=['ordered'])
       self.wait = rospy.Duration.from_sec(0.5)
       rospy.loginfo("Creating mary client")
       self.maryClient = actionlib.SimpleActionClient('speak', maryttsAction)
       self.maryClient.wait_for_server()
       rospy.loginfo(" ...done")

    def execute(self, userdata):
        rospy.loginfo("Taking order!")
        rospy.sleep(self.wait)
        goal = maryttsGoal
        toggle = randint(0,6)
        if toggle == 0:
            goal.text = "Your order please."
        if toggle == 1:
            goal.text = "What would you like?"
        if toggle == 2:
            goal.text = "Have you already decided?"
        if toggle == 3:
            goal.text = "Another drink for you?"
        if toggle == 4:
            goal.text = "What can I do you for?"
        if toggle == 5:
            goal.text = "Are you ready to order?"
        if toggle == 6:
            goal.text = "Fancy a drink?"
        if toggle == 7:
            goal.text = "Found something you like?"
        if toggle == 8:
            goal.text = "Drinks? Anyone?"
        if toggle == 9:
            goal.text = "More drinks?"
        self.maryClient.send_goal(goal)
        return 'ordered'
