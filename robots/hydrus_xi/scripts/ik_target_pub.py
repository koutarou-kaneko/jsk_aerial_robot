#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf_conversions
import numpy as np
from IPython import embed
import time
import threading
import math

from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Empty, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from std_srvs.srv import Trigger, SetBool, SetBoolRequest
from spinal.msg import FourAxisCommand
from tf.transformations import quaternion_multiply

x=0
y=0
theta=0

def threadfunc():
    def w():
        global y
        y+=0.05

    def a():
        global x
        x-=0.05

    def s():
        global y
        y-=0.05

    def d():
        global x
        x+=0.05

    def q():
        global theta
        theta+=0.1

    def e():
        global theta
        theta-=0.1
    
    embed()

if __name__ == '__main__':
    th=threading.Thread(target=threadfunc)
    rospy.init_node('ik_target_pub')
    rospy.sleep(0.2)
    x=rospy.get_param('~manip_x')
    y=rospy.get_param('~manip_y')
    theta=rospy.get_param('~manip_theta')
    br=tf2_ros.TransformBroadcaster()

    r=rospy.Rate(10.0)
    rospy.sleep(1)
    th.start()

    while (not rospy.is_shutdown()):
        t=TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "hydrus_xi/manipulation_target"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        r.sleep()
    th.joint()