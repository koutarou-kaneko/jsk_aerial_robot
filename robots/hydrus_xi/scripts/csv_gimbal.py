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
from sensor_msgs.msg import JointState

def wrench_cb(msg):
    print "%lf, %lf, %lf, %lf, %lf" % ((msg.header.stamp).to_sec(), msg.position[0], msg.position[1], msg.position[2], msg.position[3])

if __name__ == '__main__':
    rospy.init_node('target_error')
    rospy.sleep(0.2)
    buf = tf2_ros.Buffer()
    tfl = tf2_ros.TransformListener(buf)
    sub = rospy.Subscriber('hydrus_xi/joint_states', JointState, wrench_cb)
    r=rospy.Rate(10.0)
    rospy.sleep(2)

    while (not rospy.is_shutdown()):
        r.sleep()