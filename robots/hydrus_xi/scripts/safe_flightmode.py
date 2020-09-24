#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import math
import numpy

from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_multiply, euler_from_quaternion

e = [0,0,0]
tw = 0

def odom_cb(msg):
    global e, tw
    qq = msg.pose.pose.orientation
    q = [qq.x, qq.y, qq.z, qq.w]
    e = euler_from_quaternion(q)
    tw = numpy.linalg.norm(numpy.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]))

if __name__ == '__main__':
    rospy.init_node('safe_flightmode')
    sub = rospy.Subscriber('hydrus_xi/uav/cog/odom', Odometry, odom_cb, queue_size=1)
    rate=rospy.Rate(10.0)
    safe_mode = False

    while (not rospy.is_shutdown()):
        rospy.loginfo_throttle(1, "ex:{}, ey:{}, ez:{}, twist_norm:{}".format(e[0], e[1], e[2], tw))
        if abs(e[0]) > 0.2 or abs(e[1]) > 0.2:
            rospy.wait_for_service('hydrus_xi/reset_horizontal_force_mode')
            try:
                reset_srv = rospy.ServiceProxy('hydrus_xi/reset_horizontal_force_mode', Empty)
                reset_srv()
                safe_mode = True
                rospy.loginfo("Safe mode ON, waiting for 1 sec")
                rospy.sleep(1)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        if safe_mode and abs(e[0]) < 0.005 and abs(e[1]) < 0.005 and tw < 0.03:
            rospy.wait_for_service('hydrus_xi/set_horizontal_force_mode')
            try:
                set_srv = rospy.ServiceProxy('hydrus_xi/set_horizontal_force_mode', Empty)
                set_srv()
                safe_mode = False
                rospy.loginfo("Safe mode OFF")
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        rate.sleep()