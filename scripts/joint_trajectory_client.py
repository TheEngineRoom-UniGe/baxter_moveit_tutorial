#!/usr/bin/env python

import argparse
import sys
from threading import Event
from copy import copy, deepcopy
import rospy
import actionlib
import time
import baxter_interface
from trajectoryPlanner import TrajectoryPlanner
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    GripperCommandActionGoal
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from geometry_msgs.msg import Pose

from baxter_moveit.msg import MoveitTrajectory

from baxter_interface import CHECK_VERSION

# Client class to provides trajectory execution service
class TrajectoryClient():

    def __init__(self, limb):
        self.limb = limb
        rospy.Subscriber("/right_arm/baxter_moveit_trajectory", MoveitTrajectory, self.callback)

        self.trajTime = 2.5 # Time to complete each trajectory 
        self.traj_p = TrajectoryPlanner()
        self.stop_sleeping_sig = Event()

    def callback(self, msg):
        trajectory = msg.trajectory
        n = len(trajectory)

        for i in range(len(trajectory)):
            traj = Trajectory(self.limb)

            rospy.on_shutdown(traj.stop)
            # Command Current Joint Positions first
            limb_interface = baxter_interface.limb.Limb(self.limb)

            t = 0
            for point in trajectory[i].joint_trajectory.points:
  
                t += self.trajTime/len(trajectory[i].joint_trajectory.points)      #prima c'era 0.1
                traj.add_point(point.positions, point.velocities, point.accelerations, t)
                
            traj.start()
            traj.wait()
        
        self.ex_complete_pub.publish(True)
        print("Action Complete")


    def execute_trajectory(self, trajectory, side, speed):

        traj = Trajectory(side)

        rospy.on_shutdown(traj.stop)
        # Command Current Joint Positions first
        # limb_interface = baxter_interface.limb.Limb(self.limb)

        t = 0
        counter = 0
        avg = self.trajTime/len(trajectory.joint_trajectory.points)
        third = len(trajectory.joint_trajectory.points) / 5
        for point in trajectory.joint_trajectory.points:

            if counter < third:
                t += (1.5*avg)-(counter * avg / (2*third))

            elif counter >= third and counter < 4 * third:
                t += avg

            elif counter >= 4 * third:
                t += avg + (counter * avg / (2*third))
            # t += avg     #prima c'era 0.1
            traj.add_point(point.positions, point.velocities, point.accelerations, t)
            
            counter += 1
            
        traj.start()
        traj.wait()
    
        print("Action Complete")


    def move_to(self, poses, side):
        failures = False
        first = True

        for pose in poses:
            rospy.logerr("Transfer function called")
            rospy.logerr(pose)
            rospy.logerr(side)
            temp_pose = deepcopy(pose)
            #reach XY location above object
            simple_traj, fract = self.traj_p.plan_cartesian_trajectory(temp_pose, side)
            
            print('fraction', fract)
            if fract > 0.80:
                if first:
                    self.execute_trajectory(simple_traj, side,2.5)
                else:
                    self.execute_trajectory(simple_traj, side,3.0)
            else:
                rospy.logerr(fract)
                failures = True
        return not failures
   

    def tuck(self):
        for limb in ['left', 'right']:
            if limb == 'left':
                m_group = self.traj_p.move_group_left
                joint_values = [-0.10392719837923678, 2.1563934925699204, 1.495247772991307, -0.6715000898968398, -0.7923010769428162, -0.07746602978821339, 3.051087787104088]
            else:
                m_group = self.traj_p.move_group_right
                joint_values = [-0.10316020798529407, 2.223505152039907, -1.4891118498397653, -0.6550097964270717, -2.2422964166915036, -0.11083011192472114, 3.017340209770609]
            joint_names = [limb + '_' + joint for joint in ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']]
            dict = {}
            for name, value in zip(joint_names, joint_values):
                dict[name] = value
            m_group.set_joint_value_target(dict)
            m_group.set_goal_tolerance(10e-3)
            
            plan = m_group.plan()
            self.execute_trajectory(plan[1], limb)


# Trajectory class to handle joint trajectory action
class Trajectory(object):

    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.05)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)
        gripper_topic = "robot/end_effector/" + limb + "_gripper/gripper_action/goal"
        self.gripper_publisher = rospy.Publisher(gripper_topic, GripperCommandActionGoal, queue_size=10)

    def add_point(self, positions, velocities, accelerations, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = copy(velocities)
        point.accelerations = copy(accelerations)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

    

def main():
    
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l',
        '--limb',
        required=True,
        choices=['left', 'right'],
        help='Send joint trajectory to which limb'
    )

    args = parser.parse_args(rospy.myargv()[1:])

    # Instantiate trajectory handler object
    client = TrajectoryClient(args.limb)

    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client_%s" % (args.limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running...")
    

    client.traj_p.open_gripper('left')
    client.traj_p.open_gripper('right')
    screwdriver_pose = Pose()
    screwdriver_pose.position.x = 0.61
    screwdriver_pose.position.y = -0.37
    screwdriver_pose.position.z = -0.26
    screwdriver_pose.orientation.x = 1.0
    screwdriver_pose.orientation.y = 0.0
    screwdriver_pose.orientation.z = 0.0
    screwdriver_pose.orientation.w = 0.0

    screwdriver_pose2 = Pose()
    screwdriver_pose2.position.x = 0.61 - 0.15
    screwdriver_pose2.position.y = -0.37
    screwdriver_pose2.position.z = -0.26 + 0.20
    screwdriver_pose2.orientation.x = 1.0
    screwdriver_pose2.orientation.y = 0.0
    screwdriver_pose2.orientation.z = 0.0
    screwdriver_pose2.orientation.w = 0.0

    # handover_location = Pose()
    # handover_location.position.x = 0.625
    # handover_location.position.y = -0.20
    # handover_location.position.z = -0.15
    # handover_location.orientation.x = 1.0
    # handover_location.orientation.y = 0.0
    # handover_location.orientation.z = 0.0
    # handover_location.orientation.w = 0.0

    while not rospy.is_shutdown():
        client.traj_p.open_gripper('right')
        client.move_to([screwdriver_pose], 'right')
        client.move_to([screwdriver_pose2], 'right')
        client.traj_p.close_gripper('right')
        # client.move_to([handover_location], 'right')
    
    rospy.spin()


if __name__ == "__main__":
    main()
