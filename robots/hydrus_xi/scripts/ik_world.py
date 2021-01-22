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
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_srvs.srv import Trigger, SetBool, SetBoolRequest
from spinal.msg import FourAxisCommand
from tf.transformations import quaternion_multiply

if __name__ == '__main__':
    rospy.init_node('ik_world')
    pub = rospy.Publisher('hydrus_xi/manipulation_target', PoseStamped, queue_size=1)
    #sub = rospy.Subscriber('hydrus_xi/four_axes/command', FourAxisCommand, thrust_cb)
    rospy.sleep(0.2)
    buf = tf2_ros.Buffer()
    tfl = tf2_ros.TransformListener(buf)
    r=rospy.Rate(3.0)
    rospy.sleep(1)

    while (not rospy.is_shutdown()):
        try:
            world_tf = buf.lookup_transform('hydrus_xi/link4', 'hydrus_xi/manipulation_target', rospy.Time.now(), rospy.Duration(0.2)).transform
            pub.publish(PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='hydrus_xi/link4') ,pose=Pose(position=Point(x=world_tf.translation.x, y=world_tf.translation.y, z=world_tf.translation.z), orientation=world_tf.rotation)))
        except:
            rospy.loginfo_throttle(1, 'tf lookup failed')
        r.sleep()