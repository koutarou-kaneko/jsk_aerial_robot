#!/usr/bin/env python
import tf
import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def cb(msg):
    print(quaternion_to_euler(msg.pose.pose.orientation).z)

if __name__ == '__main__':
    rospy.init_node("yaw")
    rospy.Subscriber("/hydrus_xi/uav/cog/odom", Odometry, cb)
    rospy.spin()
