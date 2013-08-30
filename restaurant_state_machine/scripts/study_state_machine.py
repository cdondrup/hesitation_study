#!/usr/bin/env python

import rospy
import smach
import smach_ros
from waypoint_reader import WaypointReader
from move_robot import MoveRobot
from take_order import TakeOrder
from place_order import PlaceOrder
from choose_behaviour import ChooseBehaviour

def main():
    rospy.init_node('study_smach')

    # Get parameters
    file_path = rospy.get_param("~waypoints", "")
    if file_path == "":
        rospy.logfatal("No waypoint file given. Please run with _waypoints:=<file_path>")
        return

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    sis = smach_ros.IntrospectionServer('hesitation_study_state_machine', sm, '/restaurant_study')
    sis.start()
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('POINT_READER', WaypointReader(file_path),
            transitions={'goto_point':'MOVE_ROBOT'},
            remapping={'goal_pose':'goal_pose'})
        smach.StateMachine.add('MOVE_ROBOT', MoveRobot(),
            transitions={'reached_table':'TAKE_ORDER', 'reached_kitchen':'CHOOSE_BEHAVIOUR' ,'failure':'aborted'})
        smach.StateMachine.add('TAKE_ORDER', TakeOrder(),
            transitions={'ordered':'POINT_READER'})
        smach.StateMachine.add('CHOOSE_BEHAVIOUR', ChooseBehaviour(),
            transitions={'behaviour_set':'PLACE_ORDER'})
        smach.StateMachine.add('PLACE_ORDER', PlaceOrder(),
            transitions={'order_placed':'POINT_READER'})

    # Execute SMACH plan
    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
