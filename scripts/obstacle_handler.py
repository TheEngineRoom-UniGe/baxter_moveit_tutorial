#!/usr/bin/env python

from __future__ import print_function

import rospy
import io
import argparse
import sys
import copy
import math
import moveit_commander
from geometry_msgs.msg import PoseStamped

class HandObstacleHandler:

    def __init__(self):

        move_group = moveit_commander.MoveGroupCommander("left_arm")
        self.ee_link = move_group.get_end_effector_link()
        self.robot =  moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous = True)
        self.timeout = 5

        
    def wait_for_update(self, box_is_attached = False, box_is_known = False):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < self.timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects(["hand"])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = "hand" in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_hand(self):
        rospy.sleep(2)
        box_pose = PoseStamped()
        box_pose.header.frame_id = "left_hand"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.1
        box_name = "hand"
        self.scene.add_box(box_name, box_pose, size=(0.1, 0.15, 0.2))
        if self.wait_for_update(box_is_known=True):
            grasping_group = "left_hand"
            touch_links = self.robot.get_link_names(grasping_group)
            touch_links.append('l_gripper_l_finger')
            touch_links.append('l_gripper_r_finger')
            touch_links.append('l_gripper_r_finger_tip')
            touch_links.append('l_gripper_l_finger_tip')
            print(touch_links)
            self.scene.attach_box(self.ee_link, box_name, touch_links=touch_links)


    def add_table(self):
        rospy.sleep(2)
        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.5
        box_pose.pose.position.y = -0.1
        box_pose.pose.position.z = -0.7

        box_name = "table1"
        self.scene.add_box(box_name, box_pose, size=(0.7, 0.7, 0.7))
        # rospy.sleep(2)
        # box_pose = PoseStamped()
        # box_pose.header.frame_id = "world"
        # box_pose.pose.orientation.w = 1.0
        # box_pose.pose.position.y = 1
        # box_pose.pose.position.z = -0.6

        # box_name = "table2"
        # self.scene.add_box(box_name, box_pose, size=(2, 1, 1))
  

    def add_wall(self):
        rospy.sleep(2)
        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = -0.85
        box_pose.pose.position.z = -0.7

        box_name = "wall"
        self.scene.add_box(box_name, box_pose, size=(1, 4, 4))


def main():
    # Initialize node
    rospy.init_node('hand_obstacle_handler')
    moveit_commander.roscpp_initialize(sys.argv) 

    rospy.sleep(2)
    print("Adding obstacles...")
    handler = HandObstacleHandler()
    # handler.add_hand()
    # print("Obstacle added")
    handler.add_table()
    print("Table added")
    handler.add_wall()
    print("Wall added")    
    rospy.spin()



if __name__ == "__main__":
    main()
