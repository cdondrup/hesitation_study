#!/usr/bin/env python

import rospy
import smach
import csv
from random import randint

from move_base_msgs.msg import *

class WaypointReader(smach.State):
    """This class reads the waypoints from a file"""
    def __init__(self, file_path):
        rospy.loginfo("Creating reader for: %s", file_path)
        self.file_path = file_path
        smach.State.__init__(self,
            outcomes = ['goto_point'],
            output_keys=['goal_pose'])

        rospy.loginfo("Reading points")
        self.table_points   = []
        self.kitchen_points = []
        with open(self.file_path, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                current_row = []
                current_row.append(str(row[0]))
                for element in row[1:]:
                    current_row.append(float(element))
                if current_row[0] == 'kitchen' :
                    self.kitchen_points.append(current_row)
                else:
                    self.table_points.append(current_row)

        self.current_point = 0
        self.n_points = len(self.table_points)
        self.toggle = True

    def execute(self, userdata):
        next_goal = []
        goal = move_base_msgs.msg.MoveBaseGoal()

        current_row = []
        if self.toggle:
            self.toggle = False
            current_row = self.kitchen_points[0]
        else:
            current_row = self.table_points[randint(0,self.n_points-1)]
            self.toggle = True

        rospy.loginfo("Reading point: %s", current_row[0])

        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x=current_row[1]
        goal.target_pose.pose.position.y=current_row[2]
        goal.target_pose.pose.position.z=current_row[3]
        goal.target_pose.pose.orientation.x=current_row[4]
        goal.target_pose.pose.orientation.y=current_row[5]
        goal.target_pose.pose.orientation.z=current_row[6]
        goal.target_pose.pose.orientation.w=current_row[7]

        next_goal.append(current_row[0])
        next_goal.append(goal)

        userdata.goal_pose=next_goal

        return 'goto_point'

