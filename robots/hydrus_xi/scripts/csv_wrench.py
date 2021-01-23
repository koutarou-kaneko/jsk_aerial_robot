#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from IPython import embed
import time
import math

from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Empty, Header
from geometry_msgs.msg import Vector3Stamped, Vector3, WrenchStamped, Wrench, TransformStamped
from std_srvs.srv import Trigger, SetBool, SetBoolRequest
from spinal.msg import FourAxisCommand
from tf.transformations import quaternion_multiply

tf = TransformStamped()

def wrench_cb(msg):
    global tf
    wr = tf2_geometry_msgs.do_transform_wrench(msg, tf).wrench.force
    print "%lf, %lf, %lf" % (wr.x, wr.y, wr.z)

if __name__ == '__main__':
    rospy.init_node('target_error')
    rospy.sleep(0.2)
    buf = tf2_ros.Buffer()
    tfl = tf2_ros.TransformListener(buf)
    sub = rospy.Subscriber('hydrus_xi/thrust_wrench', WrenchStamped, wrench_cb)
    r=rospy.Rate(10.0)
    rospy.sleep(2)

    while (not rospy.is_shutdown()):
        try:
            global tf
            tf = buf.lookup_transform('world', 'hydrus_xi/cog', rospy.Time())
        except:
            a=1
            #rospy.loginfo_throttle(1, 'tf lookup failed')
        r.sleep()