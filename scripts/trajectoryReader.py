#!/usr/bin/env python


from time import sleep
import rospy
import argparse
import yaml, os
from threading import Event
import signal
from baxter_moveit.msg import MoveitTrajectory

SLEEP_TIME = 18.0 #THIS MUST BE EQUAL TO THE TIME BETWEEN EACH TRAJECTORY (SEE JOINT_TRAJECTORY_CLIENT.PY)

exit = Event()

class TrajectoryReader:
        #moveit_commander.PlanningSceneInterface()        
    def __init__(self):
        self.publisher = rospy.Publisher('/left_arm/baxter_moveit_trajectory', MoveitTrajectory, queue_size=10)
        rospy.init_node('readerScript')


    def read(self):
        print("Reading from the file...\n")
        return yaml.load(self.f, Loader=yaml.Loader)


    def publishAll(self, trajList):
        for traj in trajList:
            if not exit.is_set():
                # print("TRAJ_READER ----> Awake")
                if traj == 0:   print("TRAJ_READER ----> I'm sending the " + str(traj+1) + "st trajectory to the robot!")
                if traj == 1:   print("TRAJ_READER ----> I'm sending the " + str(traj+1) + "nd trajectory to the robot!")
                if traj == 2:   print("TRAJ_READER ----> I'm sending the " + str(traj+1) + "rd trajectory to the robot!")
                if traj > 2:    print("TRAJ_READER ----> I'm sending the " + str(traj+1) + "th trajectory to the robot!")
                traj_msg = MoveitTrajectory()
                traj_msg.trajectory.append(trajList[traj])
                self.publisher.publish(traj_msg)
                # print("TRAJ_READER ----> Sleeping...")
                if traj == 99:  return      # WANT TO LIMIT THE YAML TO 100 TRAJECTORIES    
                exit.wait(SLEEP_TIME)
            #sleep(SLEEP_TIME)
        if exit.is_set():
            print("TRAJ_READER ----> Program interrupted by the user!\n")
    

    def fileOpener(self, file):
        if file is None:
            print("File not specified. Default one will be used\n")
            file = "/home/simone/.ros/myPoses2.txt"

        
        if file[0] != '/':
            print("A relative path was used. Adding current working directory.\n")
            file = os.getcwd() + '/' + file

        print("Opening: " + file + '\n')

        self.f = open(file, "r")


def quit(signo, frame):
    print("\nInterrupted by %d, shutting down\n" % signo)
    exit.set()


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
    args = parser.parse_args(rospy.myargv()[1:])
    print("\nCurrent working directory: " + os.getcwd() + '\n')
    return args


def main():
    for sig in ('TERM', 'HUP', 'INT'):
        signal.signal(getattr(signal, 'SIG'+sig), quit)
    args = getParams()
    reader = TrajectoryReader()
    reader.fileOpener(args.filename)
    data = reader.read()
    for i in range(2):  # REPEAT TWO TIMES BECAUSE I PLANNED 100 POINTS AND I NEED 200
        reader.publishAll(data)
    os.system("rosnode kill joint_action_server_left joint_action_left hand_driver hand_motion rviz")
    rospy.signal_shutdown("Adios Amigo!")


if __name__ == "__main__":
    main()
    
