#!/usr/bin/env python

import sys
import time
import rospy
import math
import signal
import tf.transformations as tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from aerial_robot_msgs.msg import FlightNav, PoseControlPid

class mocap_control():
  def __init__(self):
    self.period = rospy.get_param("~period", 40.0)
    self.radius = rospy.get_param("~radius", 1.0)
    self.init_theta = rospy.get_param("~init_theta", 0.0)
    self.yaw = rospy.get_param("~yaw", True)
    self.loop = rospy.get_param("~loop", False)
    self.link_num = rospy.get_param("~link_num", 4)
    self.duration = rospy.get_param("~duration", 8)
    joint_control_topic_name = rospy.get_param("/hydrus/joint_control_topic_name", "/hydrus/joints_ctrl")
    #rospy.set_param("~/test_topic_name", "hogehogehoge")
    self.joint_pub = rospy.Publisher(joint_control_topic_name, JointState, queue_size=10)
    self.nav_pub = rospy.Publisher("~uav/nav", FlightNav, queue_size=1)
    #self.control_sub = rospy.Subscriber("~debug/pose/pid", PoseControlPid, self.mocapCb)
    self.mocap_sub = rospy.Subscriber("mocap/pose", PoseStamped, self.mocapCb)

    self.omega = 2 * math.pi / self.period
    self.velocity = self.omega * self.radius
    #self.velocity = 0.15
    self.nav_rate = rospy.get_param("~nav_rate", 20.0) # hz
    self.nav_rate = 1 / self.nav_rate
    self.joint = JointState()
    self.joint.position = []


    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.pos_z_nav_mode = FlightNav.POS_VEL_MODE
    if self.yaw:
      self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE

    signal.signal(signal.SIGINT, self.stopRequest)

    time.sleep(0.5)    

  def mocapCb(self, msg):
    self.mocap_pos = msg.pose.position
    orientation = msg.pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    self.mocap_euler = tf.euler_from_quaternion(quaternion)

    #rospy.loginfo("mocap position is [%f, %f, %f]", self.mocap_pos.x, self.mocap_pos.y, self.mocap_pos.z)
    #rospy.loginfo("mocap euler is [%f, %f, %f]", self.mocap_euler.x, self.mocap_euler.y, self.mocap_euler.z)


  def stopRequest(self, signal, frame):
    rospy.loginfo("stop following")
    self.flight_nav.target_vel_x = 0
    self.flight_nav.target_vel_y = 0
    self.flight_nav.target_omega_z = 0
    self.nav_pub.publish(self.flight_nav)

    sys.exit(0)

  def main(self):

    cnt = 0
    for i in range(0, self.link_num - 1):
      self.joint.position.append(2 * math.pi / self.link_num)

    while not rospy.is_shutdown():
      
      for i in range(0, self.link_num - 1):
        self.joint.position[i] = -self.joint.position[i]
        self.joint_pub.publish(self.joint)
        print("joint angles: {}".format(self.joint.position))
        time.sleep(self.duration)

      if self.mocap_pos == None:
        rospy.loginfo_throttle(1.0, "not yet receive the controller message")
        time.sleep(self.nav_rate)
        continue
      
      self.flight_nav.target_pos_x = self.mocap_pos.x
      self.flight_nav.target_pos_y = self.mocap_pos.y
      self.flight_nav.target_pos_z = self.mocap_pos.z
      self.flight_nav.target_vel_x = self.velocity
      self.flight_nav.target_vel_y = self.velocity
      self.flight_nav.target_vel_z = self.velocity
      self.flight_nav.target_yaw = self.mocap_euler[2]
      self.flight_nav.target_omega_z = self.omega
      #rospy.loginfo("target_pos is [%f, %f]", self.flight_nav.target_pos_x, self.flight_nav.target_pos_y)
      rospy.loginfo("target_pos is [%f, %f, %f]", self.flight_nav.target_pos_x, self.flight_nav.target_pos_y, self.flight_nav.target_pos_z)
      rospy.loginfo("target_yaw is [%f]", self.flight_nav.target_yaw)
      self.nav_pub.publish(self.flight_nav)

      cnt += 0.1

      if cnt == self.period // self.nav_rate:
        if self.loop:
          cnt = 0
        else:

          time.sleep(0.1)
          self.flight_nav.target_vel_x = 0
          self.flight_nav.target_vel_y = 0
          self.flight_nav.target_vel_z = 0
          self.flight_nav.target_omega_z = 0
          self.nav_pub.publish(self.flight_nav)

          break # only one loop

      time.sleep(self.nav_rate)

if __name__ == "__main__":

  rospy.init_node("hydrus_mocap_ctrl")

  Tracker = mocap_control()
  Tracker.main()

