#!/usr/bin/python3
import rospy, os

from std_msgs.msg import (
    Empty,
    Bool,
)

import baxter_interface

from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import JointState

def main():
    
    rospy.init_node("arms_positioner")
    handpub = rospy.Publisher('/joint_control', JointState, queue_size=10)
    # FINGERS NAMES
    fingers = ['Index', 'Middle', 'Ring', 'Pinkie', 'Thumb']

    # JOINTS NAMES
    names = ['servo0', 'servo1', 'servo2', 'servo3', 'servo4',
             'servo5', 'servo6', 'servo7', 'servo8', 'servo9']
    openPos = [0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34]

    opened = JointState()
    opened.name = names
    opened.position = openPos[:]

    
    left_interface = baxter_interface.limb.Limb("left")
    angles = dict(zip(left_interface.joint_names(),
                          [-0.96, 0.2, 1.13, 0.3, -0.965640905973868, -0.33709227813781967, -0.14611167004608566]))
    left_interface.move_to_joint_positions(angles)

    right_interface = baxter_interface.limb.Limb("right")
    angles = dict(zip(right_interface.joint_names(),
                          [-1.5880536106583745, -1.309636097657172, -0.05445631796993219, 2.5920440363293777, -0.07516505860638527, 0.37045636027432743, -0.16451943950071063]))
    right_interface.move_to_joint_positions(angles)

    opened.header.stamp = rospy.Time.now()
    handpub.publish(opened)
    os.system("rosnode kill hand_driver")
    rospy.signal_shutdown("Adios Amigo!")

if __name__ == "__main__":
    main()
