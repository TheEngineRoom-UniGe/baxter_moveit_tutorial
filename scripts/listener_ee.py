#!/usr/bin/env python  
import roslib
import rospy
import math
import tf2_ros
import rospkg
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('baxter_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(20.0)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('baxter_velocity')

    f = open(pkg_path + "/config/NC.txt", "w")
    
    

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'reference/right_gripper', rospy.Time(0))
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        f.write("%i,%i,%f,%f,%f\n" % (trans.header.stamp.secs, trans.header.stamp.nsecs, trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z))
        print(type(trans.header.stamp.secs))
        print(trans.header.stamp.nsecs)
        print(trans.transform.translation.x)
        print(trans.transform.translation.y)
        print(trans.transform.translation.z)
        
        rate.sleep()

    f.close()