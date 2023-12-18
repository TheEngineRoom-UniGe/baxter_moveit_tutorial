#!/usr/bin/env python

from __future__ import print_function
import json, yaml, os, sys

from sys import argv
import argparse
from io import StringIO
import math
from time import sleep
import rospy
import moveit_commander
from baxter_moveit.srv import MoveitService, MoveitServiceResponse, JointStateService
from rospy.topics import Publisher
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, AttachedCollisionObject, CollisionObject
from geometry_msgs.msg import Pose, Vector3
from shape_msgs.msg import SolidPrimitive
import random
import tf_conversions
from rospy import service
from std_msgs.msg import String
from baxter_moveit.msg import MoveitTrajectory

#   BOUNDARIES TO CHOOSE THE CARTESIAN COORDINATE OF THE TRAJECTORIES 
X_MIN, X_MAX = 0, 1           #prima era 0.1, 1.0
Y_MIN, Y_MAX = 0, 1         #prima era 0.6, 0.8
Z_MIN, Z_MAX = -0.1, 0.8        #prima era 0.1, 0.4

ROT_MIN, ROT_MAX = -10, 10


#   THE NUMBER OF TRAJECTORIES YOU WANT TO PLAN
NUM_TRAJECTORIES = 10

#   HOW MANY SECONDS BETWEEN THE CURRENT TRAJECTORY AND THE NEXT ONE (must be equal to the time Baxter need to travel the whole path)
SLEEP_TIME = 10

#   EXPERIMENT LENGTH = SLEEP_TIME * NUM_TRAJECTORIES

poseArray = []

class Experiment:

   
    def __init__(self):
        
        self.trajectoryList = []
        self.limb = "left"
        self.group_name = 'left_arm'
        self.publisher = rospy.Publisher('baxter_moveit_trajectory', MoveitTrajectory, queue_size=10)
        self.attach_obj_pub = rospy.Publisher('/attached_collision_object', AttachedCollisionObject, queue_size=10)
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.robot = moveit_commander.RobotCommander()
        self.initialState()
        self.attach_tool_to_end_effector()
        self.attach_tool_to_end_effector()
        self.attach_tool_to_end_effector()
        #self.f = open("./myPoses21.txt", "r")
        self.currentTrajIdx = 1

    def form_attach_collision_object_msg(self, link_name, obj_name, size, pose):
        obj = AttachedCollisionObject()
        obj.link_name = link_name
        touch_links = self.robot.get_link_names('left_hand')
        touch_links.append('l_gripper_l_finger')
        touch_links.append('l_gripper_r_finger')
        touch_links.append('l_gripper_r_finger_tip')
        touch_links.append('l_gripper_l_finger_tip')
        # obj.touch_links = self.touch_links
        col_obj = CollisionObject()
        col_obj.id = obj_name
        col_obj.header.frame_id = link_name
        col_obj.header.stamp = rospy.Time.now()
        sp = SolidPrimitive()
        sp.type = sp.BOX
        sp.dimensions = [0.0]*3
        sp.dimensions[0] = size.x
        sp.dimensions[1] = size.y
        sp.dimensions[2] = size.z
        col_obj.primitives = [sp]
        col_obj.primitive_poses = [pose]
        col_obj.operation = col_obj.ADD
        obj.object = col_obj
        obj.weight = 0.0
        return obj

    def attach_tool_to_end_effector(self):
        tool_pose = Pose()
        tool_pose.position.x = 0.17
        tool_size = Vector3(0.1,0.2,0.1)
        ee = self.move_group.get_end_effector_link()
        print('end_effector ----', ee)
        attach_msg = self.form_attach_collision_object_msg(link_name=ee, obj_name='ar10_BOX', size=tool_size, pose=tool_pose)
        self.attach_obj_pub.publish(attach_msg)

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

        self.move_group.set_start_state_to_current_state()
        
        #moveit_robot_state = RobotState()
        #moveit_robot_state.joint_state = current_joint_state
        #self.move_group.set_start_state(moveit_robot_state)
        #self.move_group.set_goal_tolerance(10e-3)


    def plan_wrapper(self, pose):
        simple_traj, fract = self.plan_cartesian_trajectory(pose)#, current_robot_joint_configuration)


        return simple_traj, fract


    def plan_cartesian_trajectory(self, destination_pose):#, start_joint_angles):
        
        #(plan, fraction) = self.move_group.compute_cartesian_path([destination_pose], 0.01, 0.0) #TODO MAX STEP
        self.move_group.set_pose_target(destination_pose)
        plan = self.move_group.plan()

        fraction = 1.0
        print(len(plan[1].joint_trajectory.points))
        if len(plan[1].joint_trajectory.points) == 0:
            fraction = 0.5
            
        return plan[1], fraction


    def newStartState(self, joints):
        joint_state = JointState()
        joint_state.name = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        joint_state.position = joints
        moveit_robot_state = RobotState()
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
                print("First pose selected")


            elif len(poseArray) != 0:
                while cartesianDistance(poseArray[-1].position.x, x, poseArray[-1].position.y, y, poseArray[-1].position.z, z) < 0.2  or frac < 1.0:
                    x, y, z = randomPoseGenerator(X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX)
                    xr, yr, zr = randomPoseGenerator(ROT_MIN, ROT_MAX, ROT_MIN, ROT_MAX, ROT_MIN, ROT_MAX)
                    xr, yr, zr = 0.0 + xr, 1.5707963267948963 + yr, 0.0 + zr
                    posa = messageConstruction(x, y, z, *quatFromEuler(xr, yr, zr))
                    trajectory, frac = self.plan_wrapper(posa)

                    
            start = self.newStartState(trajectory.joint_trajectory.points[-1].positions)
            self.move_group.set_start_state(start)
            poseArray.append(posa)
            print("Found a plan for the trajectory #" + str(self.currentTrajIdx))
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
        print(traj_msg)

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
    # args = getParams()
    # rospy.sleep(10)
    experiment = Experiment()
    # myPoses = experiment.experimentCycle(checkParamN(args.tpoints)) 
    # experiment.fileOpener(args.filename)

    # #ENABLE THIS TO DIRECTLY SEND THE TRAJECTORIES TO BAXTER
    # #experiment.sendTrajectories()

    # #ENABLE THIS TO SAVE THE TRAJECTORIES ON A FILE
    # experiment.saveTrajectories()


if __name__ == "__main__":
    main()
