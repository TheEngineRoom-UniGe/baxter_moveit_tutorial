#!/usr/bin/env python

from __future__ import print_function
import yaml, os

import argparse
import math
from time import sleep
import rospy
import moveit_commander
from baxter_moveit.srv import MoveitService, JointStateService
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from control_msgs.msg import (
    GripperCommandActionGoal
)
from geometry_msgs.msg import Pose
import random
import tf_conversions
from std_msgs.msg import String
from baxter_moveit.msg import MoveitTrajectory

#   BOUNDARIES TO CHOOSE THE CARTESIAN COORDINATE OF THE TRAJECTORIES 
X_MIN, X_MAX = 0.4, 1           #prima era 0.1, 1.0
Y_MIN, Y_MAX = 0.4, 1         #prima era 0.6, 0.8
Z_MIN, Z_MAX = -0.1, 0.5       #prima era 0.1, 0.4

ROT_MIN, ROT_MAX = -0.18,0.18


#   THE NUMBER OF TRAJECTORIES YOU WANT TO PLAN
NUM_TRAJECTORIES = 10

#   HOW MANY SECONDS BETWEEN THE CURRENT TRAJECTORY AND THE NEXT ONE (must be equal to the time Baxter need to travel the whole path)
SLEEP_TIME = 10

#   EXPERIMENT LENGTH = SLEEP_TIME * NUM_TRAJECTORIES

poseArray = []

class TrajectoryPlanner():

   
    def __init__(self):
        
        self.trajectoryList = []
        self.limb = "right"
        self.group_name = 'right_arm'
        self.publisher = rospy.Publisher('/right_arm/baxter_moveit_trajectory', MoveitTrajectory, queue_size=10)
        self.move_group_right = moveit_commander.MoveGroupCommander('right_arm')
        self.move_group_left = moveit_commander.MoveGroupCommander('left_arm')
        self.get_planning_scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        gripper_topic_right = "robot/end_effector/right_gripper/gripper_action/goal"
        gripper_topic_left = "robot/end_effector/left_gripper/gripper_action/goal"
        self.gripper_publisher_right = rospy.Publisher(gripper_topic_right, GripperCommandActionGoal, queue_size=10)
        self.gripper_publisher_left = rospy.Publisher(gripper_topic_left, GripperCommandActionGoal, queue_size=10)
        self.initialState()
        self.waiting4completion = False
        

    def initialState(self):
        # Initial joint configuration

        rospy.wait_for_service('baxter_joint_states')

        try:
            joint_state_service = rospy.ServiceProxy('baxter_joint_states', JointStateService)
            joint_response = joint_state_service(String(""))

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        arm_joints = joint_response.joint_state_msg.position[2:9]
        start_joint_angles = [arm_joints[2], arm_joints[3], arm_joints[0], arm_joints[1], arm_joints[4], arm_joints[5], arm_joints[6]]
        current_joint_state = JointState()
        joint_names = [self.limb + '_' + joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_angles

        self.move_group_right.set_start_state_to_current_state()
        self.move_group_left.set_start_state_to_current_state()
        self.move_group_left.set_goal_tolerance(10e-4)
        self.move_group_right.set_goal_tolerance(10e-4)
        # self.obstacles.add_hand()


    def plan_wrapper(self, pose):

        simple_traj, fract = self.plan_cartesian_trajectory(pose)#, current_robot_joint_configuration)


        return simple_traj, fract


    def plan_cartesian_trajectory(self, destination_pose, side):#, start_joint_angles):
        move_group = None
        if side == 'right':
            move_group = self.move_group_right
        elif side == 'left':
            move_group = self.move_group_left
        
        (plan, fraction) = move_group.compute_cartesian_path([destination_pose], 0.002, 0.0) #TODO MAX STEP
        # mg.set_pose_target(destination_pose)
        # plan = self.move_group.plan()

        # fraction = 1.0
        # print(plan)
        # print(len(plan[1].joint_trajectory.points))
        # if len(plan[1].joint_trajectory.points) == 0:
        #     fraction = 0.5
            
        return plan, fraction


    def newStartState(self, joints):
        joint_state = JointState()
        joint_state.name = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        joint_state.position = joints
        # moveit_robot_state = RobotState()
        # moveit_robot_state.joint_state = joint_state
        resp = self.get_planning_scene_srv(components = PlanningSceneComponents(components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS))
        moveit_robot_state = resp.scene.robot_state
        moveit_robot_state.joint_state = joint_state
        return moveit_robot_state


    def experimentCycle(self, n):
        #Message contruction
        for i in range(n):
            trajectory, frac = 0, 0
            x, y, z = randomPoseGenerator(X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX)
            xr, yr, zr = randomPoseGenerator(ROT_MIN, ROT_MAX, ROT_MIN, ROT_MAX, ROT_MIN, ROT_MAX)
            xr, yr, zr = 0.0 + xr, 1.5707963267948963 + yr, 0.0 + zr
            posa = messageConstruction(x, y, z, *quatFromEuler(xr, yr, zr))
            trajectory, frac = self.plan_wrapper(posa)

            if len(poseArray) == 0:
                while len(trajectory.joint_trajectory.points) == 0:
                    x, y, z = randomPoseGenerator(X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX)
                    xr, yr, zr = randomPoseGenerator(ROT_MIN, ROT_MAX, ROT_MIN, ROT_MAX, ROT_MIN, ROT_MAX)
                    xr, yr, zr = 0.0 + xr, 1.5707963267948963 + yr, 0.0 + zr
                    posa = messageConstruction(x, y, z, *quatFromEuler(xr, yr, zr))
                    trajectory, frac = self.plan_wrapper(posa)
                print('\n############################################\n')
                print("First pose selected")
                print('\n############################################\n')



            elif len(poseArray) != 0:
                while cartesianDistance(poseArray[-1].position.x, x, poseArray[-1].position.y, y, poseArray[-1].position.z, z) < 0.2  or frac < 1.0:
                    x, y, z = randomPoseGenerator(X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX)
                    xr, yr, zr = randomPoseGenerator(ROT_MIN, ROT_MAX, ROT_MIN, ROT_MAX, ROT_MIN, ROT_MAX)
                    xr, yr, zr = 0.0 + xr, 1.5707963267948963 + yr, 0.0 + zr
                    posa = messageConstruction(x, y, z, *quatFromEuler(xr, yr, zr))
                    trajectory, frac = self.plan_wrapper(posa)

                    
            start = self.newStartState(trajectory.joint_trajectory.points[-1].positions)
            self.move_group.set_start_state(start)
            # self.obstacles.add_hand()
            poseArray.append(posa)
            print('\n############################################\n')
            print("Found a plan for the trajectory #" + str(self.currentTrajIdx))
            print('\n############################################\n')
            self.currentTrajIdx += 1
            self.trajectoryList.append(trajectory)

        #print(len(self.trajectoryList))
        return poseArray


    def sendTrajectories(self):
        for traj in self.trajectoryList:
            traj_msg = MoveitTrajectory()
            traj_msg.trajectory.append(traj)
            self.publisher.publish(traj_msg)
            sleep(SLEEP_TIME)


    def sendSingleTrajectory(self, trajectory):
        traj_msg = MoveitTrajectory()
        traj_msg.trajectory.append(trajectory)
        # print(traj_msg)

        self.publisher.publish(traj_msg)


    def saveTrajectories(self):
        data = {}
        i = 0
        for traj in self.trajectoryList:
            data[i] = traj
            i+=1
        yaml.dump(data, self.f)
        #print("Dumping trajectory data to /home/simone/.ros/myPoses.txt")


    def fileOpener(self, file):
            if file is None:
                print("File not specified. Default one will be used")
                file = "/home/simone/.ros/myPoses2.txt"
                print('\n')

            if file[0] != '/':
                print("A relative path was used. Adding current working directory.")
                print('\n')

                file = os.getcwd() + '/' + file

            print("Opening: " + file)
            print('\n')

            self.f = open(file, "w")

    def open_gripper(self, side): 
        # sleep(1)
        gripperCommandMsg = GripperCommandActionGoal()
        gripperCommandMsg.goal.command.position = 100.0
        if side == 'right': 
            self.gripper_publisher_right.publish(gripperCommandMsg)
        elif side == 'left':
            self.gripper_publisher_left.publish(gripperCommandMsg)
    
    def close_gripper(self, side):
        sleep(1)
        gripperCommandMsg = GripperCommandActionGoal()
        gripperCommandMsg.goal.command.position = 10
        if side == 'right': 
            self.gripper_publisher_right.publish(gripperCommandMsg)
        elif side == 'left':
            # gripperCommandMsg.goal.command.position = 0
            self.gripper_publisher_left.publish(gripperCommandMsg)

        

def cartesianDistance(x1, x2, y1, y2, z1, z2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)


def quatFromEuler(x, y, z):
    x1, y1, z1, w1 = tf_conversions.transformations.quaternion_from_euler(x, y, z)
    return x1, y1, z1, w1


def eulerFromQuat(x, y, z, w):
    return tf_conversions.transformations.euler_from_quaternion([x, y, z, w])


def randomPoseGenerator(xmax, xmin, ymax, ymin, zmax, zmin):
    x = random.uniform(xmax, xmin)
    y = random.uniform(ymax, ymin)
    z = random.uniform(zmax, zmin)
    return x, y, z


def messageConstruction(px, py, pz, rx, ry, rz, rw):    #, rx, ry, rz, rw
    posa = Pose()
    positionConstructor(posa, px, py, pz)           #   0.0, 0.7071068, 0.0, 0.7071068
    orientationConstructor(posa, rx, ry, rz, rw)    #   THIS IS THE QUATERNION OF THE HORIZONTAL HAND (EULER = -0.0, 1.5707963267948963, 0.0)           
    return posa


def radToDeg(tupla):
    lista = list(tupla)
    for i in range(len(lista)):
        lista[i] *= 180/math.pi
    return tuple(lista)


def positionConstructor(posa, x, y, z):
    posa.position.x = x
    posa.position.y = y
    posa.position.z = z
    return posa


def orientationConstructor(posa, x, y, z, w):
    posa.orientation.x = x
    posa.orientation.y = y
    posa.orientation.z = z
    posa.orientation.w = w
    return posa

def serviceCall(posa):
    rospy.wait_for_service('/baxter_motion_planner')
    try:
        service = rospy.ServiceProxy('/baxter_motion_planner', MoveitService)
        response = service(posa)

        if(float(response.test)) < 1.0:
            print('##################')
            print(float(response.test))
            print('##################')

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def checkParamN(n):
    if n == None:
        return NUM_TRAJECTORIES
    else:
        return int(n)
    

def getParams():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-f', 
        '--filename', 
        required=True, 
        type=str,
        help='Absolute path of trajectory file')
    parser.add_argument(
        '-t', 
        '--tpoints', 
        required=True, 
        type=int,
        help='Number of trajectory points')
    args = parser.parse_args(rospy.myargv()[1:])



    print('\n')
    #print("Current working directory: " + os.getcwd())
    print(args)
    print('\n')
    return args



def main(): 
    rospy.init_node('experimentScript')
    # rospy.sleep(10)
    # args = getParams()
    traj_p = TrajectoryPlanner()
    traj_p.open_gripper()
    sleep(2)
    pose = Pose()
    pose.position.x = 0.61
    pose.position.y = -0.37
    pose.position.z = -0.26
    pose.orientation.x = 1.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0

    simple_traj, fract = traj_p.plan_cartesian_trajectory(pose)
    # print(simple_traj)
    print('fraction', fract)
    traj_p.sendSingleTrajectory(simple_traj)
    sleep(10)
    pose.position.z = -0.315
    simple_traj, fract = traj_p.plan_cartesian_trajectory(pose)
    print('fraction', fract)
    traj_p.sendSingleTrajectory(simple_traj)
    # traj_p.close_gripper()
    # traj_p.sendSingleTrajectory(simple_traj)
    # traj_p.sendSingleTrajectory(simple_traj)
    # myPoses = experiment.experimentCycle(checkParamN(args.tpoints)) 
    # experiment.fileOpener(args.filename)

    # #ENABLE THIS TO DIRECTLY SEND THE TRAJECTORIES TO BAXTER
    # #experiment.sendTrajectories()

    # #ENABLE THIS TO SAVE THE TRAJECTORIES ON A FILE
    # experiment.saveTrajectories()


if __name__ == "__main__":
    main()
