#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import numpy as np
from IPython import embed
import time
import math

from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Empty, Header
from geometry_msgs.msg import Vector3Stamped, Vector3, WrenchStamped, Wrench
from std_srvs.srv import Trigger, SetBool, SetBoolRequest
from spinal.msg import FourAxisCommand
from tf.transformations import quaternion_multiply

if __name__ == '__main__':
    rospy.init_node('target_error')
    rospy.sleep(0.2)
    buf = tf2_ros.Buffer()
    tfl = tf2_ros.TransformListener(buf)
    r=rospy.Rate(10.0)
    rospy.sleep(2)

    while (not rospy.is_shutdown()):
        try:
            end = buf.lookup_transform('world', 'hydrus_xi/end', rospy.Time()).transform
            tar = buf.lookup_transform('world', 'hydrus_xi/manipulation_target', rospy.Time()).transform
            print "%lf, %lf, %lf, %lf" % (end.translation.x, end.translation.y, tar.translation.x, tar.translation.y)
        except:
            a=1
            #rospy.loginfo_throttle(1, 'tf lookup failed')
        r.sleep()