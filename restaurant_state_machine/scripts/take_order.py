#!/usr/bin/env python

import rospy
import smach

# define state TakeOrder
class TakeOrder(smach.State):
    """Just sleeping for a few seconds. Might be more sophisticated stuff coming."""
    def __init__(self):
       smach.State.__init__(self, outcomes=['ordered'])
       self.wait = rospy.Duration.from_sec(2.0)

    def execute(self, userdata):
        rospy.loginfo("Taking order!")
        rospy.sleep(self.wait)
        return 'ordered'
