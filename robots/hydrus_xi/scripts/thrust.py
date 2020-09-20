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

v1=Vector3(x=0, y=0, z=0)
v2=Vector3(x=0, y=0, z=0)
v3=Vector3(x=0, y=0, z=0)
v4=Vector3(x=0, y=0, z=0)

def quat_mul_vec(q, v):
    qq = [q.x, q.y, q.z, q.w]
    q_c = [-q.x, -q.y, -q.z, q.w]
    return quaternion_multiply(quaternion_multiply(q_c, [v.x, v.y, v.z, 0]), qq)

def thrust_cb(msg):
    v1.z=-msg.base_thrust[0]
    v2.z=-msg.base_thrust[1]
    v3.z=-msg.base_thrust[2]
    v4.z=-msg.base_thrust[3]

if __name__ == '__main__':
    rospy.init_node('thrust_viz')
    pub = rospy.Publisher('hydrus_xi/thrust_wrench', WrenchStamped, queue_size=1)
    sub = rospy.Subscriber('hydrus_xi/four_axes/command', FourAxisCommand, thrust_cb)
    rospy.sleep(1.0)
    buf = tf2_ros.Buffer()
    tfl = tf2_ros.TransformListener(buf)
    r=rospy.Rate(10.0)
    rospy.sleep(1.0)

    while (not rospy.is_shutdown()):
        try:
            q1 = buf.lookup_transform('hydrus_xi/thrust1', 'hydrus_xi/cog', rospy.Time()).transform.rotation
            q2 = buf.lookup_transform('hydrus_xi/thrust2', 'hydrus_xi/cog', rospy.Time()).transform.rotation
            q3 = buf.lookup_transform('hydrus_xi/thrust3', 'hydrus_xi/cog', rospy.Time()).transform.rotation
            q4 = buf.lookup_transform('hydrus_xi/thrust4', 'hydrus_xi/cog', rospy.Time()).transform.rotation
            th1=quat_mul_vec(q1, v1)
            th2=quat_mul_vec(q2, v2)
            th3=quat_mul_vec(q3, v3)
            th4=quat_mul_vec(q4, v4)
            sumx=th1[0]+th2[0]+th3[0]+th4[0]
            sumy=th1[1]+th2[1]+th3[1]+th4[1]
            sumz=th1[2]+th2[2]+th3[2]+th4[2]
            pub.publish(WrenchStamped(header=Header(stamp=rospy.Time.now(), frame_id='hydrus_xi/cog') ,wrench=Wrench(force=Vector3(x=sumx, y=sumy, z=sumz))))
        except:
            rospy.loginfo_throttle(1, 'exception thrown')
            embed()
        #r.sleep()