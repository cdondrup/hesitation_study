#!/usr/bin/env python

import rospy
import smach

# define state Placerder
class PlaceOrder(smach.State):
    """Just sleeping for a few seconds. Might be more sophisticated stuff coming."""
    def __init__(self):
       smach.State.__init__(self, outcomes=['order_placed'])
       self.wait = rospy.Duration.from_sec(0.5)

    def execute(self, userdata):
        rospy.loginfo("Placing order!")
        rospy.sleep(self.wait)
        return 'order_placed'
