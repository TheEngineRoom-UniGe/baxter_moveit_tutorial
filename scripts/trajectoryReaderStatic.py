#!/usr/bin/env python


from time import sleep
import rospy
import argparse
import yaml, os
from threading import Event
import signal
from baxter_moveit.msg import MoveitTrajectory

SLEEP_TIME = 10

exit = Event()

class TrajectoryReader:
        #moveit_commander.PlanningSceneInterface()        
    def __init__(self):
        self.publisher = rospy.Publisher('/left_arm/baxter_moveit_trajectory', MoveitTrajectory, queue_size=10)
        rospy.init_node('readerScript')
        self.sleepTime = SLEEP_TIME


    def read(self):
        print("Reading from the file...\n")
        return yaml.load(self.f, Loader=yaml.Loader)


    def publishAll(self, trajList):
        for traj in trajList:
            if not exit.is_set():
                print("Awake")
                if traj == 0:   print("I'm sending the " + str(traj+1) + "st trajectory to the robot!")
                if traj == 1:   print("I'm sending the " + str(traj+1) + "nd trajectory to the robot!")
                if traj == 2:   print("I'm sending the " + str(traj+1) + "rd trajectory to the robot!")
                if traj > 2:    print("I'm sending the " + str(traj+1) + "th trajectory to the robot!")
                traj_msg = MoveitTrajectory()
                traj_msg.trajectory.append(trajList[traj])
                self.publisher.publish(traj_msg)
                print("Sleeping...")
                exit.wait(SLEEP_TIME)
            #sleep(SLEEP_TIME)
        if exit.is_set():
            print("Program interrupted by the user!\n")
    

    def fileOpener(self, file):
        if file is None:
            print("File not specified. Default one will be used\n")
            file = "/home/simone/.ros/myPoses2.txt"

        
        if file[0] != '/':
            print("A relative path was used. Adding current working directory.\n")
            file = os.getcwd() + '/' + file

        print("Opening: " + file + '\n')

        self.f = open(file, "r")

    def updateSleepTime(self, t):
        if t is not None:
            self.sleepTime = SLEEP_TIME + int(t)
            print("\nNew sleep time: " + str(SLEEP_TIME + int(t)) + '\n')
        else:
            print("Current sleep time: ", SLEEP_TIME.__str__() + '\n')



def quit(signo, frame):
    print("\nInterrupted by %d, shutting down\n" % signo)
    exit.set()


def getParams():
    parser = argparse.ArgumentParser(description='Read Trajectories from a file and send them to the robot.')
    parser.add_argument('-f', '--filename', help="File containing the trajectories (absolute path)")
    parser.add_argument('-t', '--time', help="How long is baxter staying in this pose")

    args = parser.parse_args()
    print("\nCurrent working directory: " + os.getcwd() + '\n')
    return args


def main():
    for sig in ('TERM', 'HUP', 'INT'):
        signal.signal(getattr(signal, 'SIG'+sig), quit)
    args = getParams()
    reader = TrajectoryReader()
    reader.updateSleepTime(args.time)
    reader.fileOpener(args.filename)
    data = reader.read()
    reader.publishAll(data)



if __name__ == "__main__":
    main()
    
