#!/usr/bin/env python

#from operator import truediv
import sys
import time
import rospy
import math
import signal
from std_msgs.msg import UInt8
import tf.transformations as tf
from geometry_msgs.msg import PoseStamped
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord

class mocap_control():
  def __init__(self):
    
    self.period = rospy.get_param("~period", 40.0)
    self.radius = rospy.get_param("~radius", 1.0)
    self.init_theta = rospy.get_param("~init_theta", 0.0)
    self.yaw = rospy.get_param("~yaw", True)
    self.loop = rospy.get_param("~loop", False)
    self.duration = rospy.get_param("~duration", 8)
    self.nav_pub = rospy.Publisher("/dragon/uav/nav", FlightNav, queue_size=1)
    self.att_control_pub = rospy.Publisher("/dragon/final_target_baselink_rot", DesireCoord, queue_size=1)
    self.mocap_sub = rospy.Subscriber("/mocap_node/avatar/pose", PoseStamped, self.mocapCb)
    self.flight_state_sub = rospy.Subscriber("/dragon/flight_state", UInt8, self.flight_stateCb)

    self.omega = 2 * math.pi / self.period
    self.velocity = self.omega * self.radius
    #self.velocity = 0.15
    self.nav_rate = rospy.get_param("/dragon/nav_rate", 20.0) # hz
    self.nav_rate = 1 / self.nav_rate
    self.Hovering = False
    self.mocap_pos = None
    self.flight_state = 0

    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.pos_z_nav_mode = FlightNav.POS_VEL_MODE
    if self.yaw:
      self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE
    self.desire_att = DesireCoord()

    signal.signal(signal.SIGINT, self.stopRequest)

    time.sleep(0.5)    

  def mocapCb(self, msg):
    self.mocap_pos = msg.pose.position
    orientation = msg.pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    self.mocap_euler = tf.euler_from_quaternion(quaternion)

  def flight_stateCb(self, msg):
    self.flight_state = msg.data
    #rospy.loginfo("flight_state is %s", self.flight_state)
    if self.flight_state == 5:
      #rospy.loginfo("Hovering")
      self.Hovering = True  

  def stopRequest(self, signal, frame):
    rospy.loginfo("stop following")
    self.flight_nav.target_vel_x = 0
    self.flight_nav.target_vel_y = 0
    self.flight_nav.target_omega_z = 0
    self.nav_pub.publish(self.flight_nav)
    sys.exit(0)

  def main(self):
    while not rospy.is_shutdown():

      if self.mocap_pos == None:
          rospy.loginfo_throttle(1.0, "not yet receive the mocap controller message")
          time.sleep(self.nav_rate)
          continue
      self.flight_nav.target_pos_x = self.mocap_pos.x
      self.flight_nav.target_pos_y = self.mocap_pos.y
      self.flight_nav.target_pos_z = self.mocap_pos.z
      self.flight_nav.target_vel_x = self.velocity
      self.flight_nav.target_vel_y = self.velocity
      self.flight_nav.target_vel_z = self.velocity
      self.desire_att.roll = self.mocap_euler[0]
      self.desire_att.pitch = self.mocap_euler[1]
      self.flight_nav.target_yaw = self.mocap_euler[2]
      self.flight_nav.target_omega_z = self.omega
      #rospy.loginfo("target_pos is [%f, %f, %f]", self.flight_nav.target_pos_x, self.flight_nav.target_pos_y, self.flight_nav.target_pos_z)
      #rospy.loginfo("target_rot is [%f, %f, %f]", self.desire_att.roll, self.desire_att.pitch, self.flight_nav.target_yaw)

      if self.Hovering == True:
        self.nav_pub.publish(self.flight_nav)
        self.att_control_pub.publish(self.desire_att)

      if self.flight_state == 4:
        self.flight_nav.target_pos_x = 0.0
        self.flight_nav.target_pos_y = 0.0
        self.flight_nav.target_pos_z = 0.0
        self.flight_nav.target_vel_x = 0.0
        self.flight_nav.target_vel_y = 0.0
        self.flight_nav.target_vel_z = 0.0
        self.desire_att.roll = 0.0
        self.desire_att.pitch = 0.0
        self.flight_nav.target_yaw = 0.0
        self.flight_nav.target_omega_z = self.omega
        self.nav_pub.publish(self.flight_nav)
        self.att_control_pub.publish(self.desire_att)

      time.sleep(self.nav_rate)

if __name__ == "__main__":

  rospy.init_node("dragon_mocap_ctrl")

  Tracker = mocap_control()
  Tracker.main()

