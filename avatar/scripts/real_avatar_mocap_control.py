#!/usr/bin/env python

#from operator import truediv
import sys
import time
import rospy
import math
import signal
import copy
from std_msgs.msg import UInt8
import tf.transformations as tf
from geometry_msgs.msg import PoseStamped
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord

class mocap_control():
  def __init__(self):

    self.real_machine = rospy.get_param("~real_machine", False)
    self.robot_name = rospy.get_param("~robot_ns", "hydrus_xi")
    topic_name = '/mocap_node/avatar/pose'
    if self.real_machine == True:
      topic_name = '/avatar_mocap_node/avatar/pose'

    self.pos_scaling = rospy.get_param("~pos_scaling", 1.5)
    self.period = rospy.get_param("~period", 40.0)
    self.radius = rospy.get_param("~radius", 1.0)
    self.init_theta = rospy.get_param("~init_theta", 0.0)
    self.yaw = rospy.get_param("~yaw", True)
    self.loop = rospy.get_param("~loop", False)
    self.duration = rospy.get_param("~duration", 8)
    self.nav_pub = rospy.Publisher('/'+self.robot_name+'/uav/nav', FlightNav, queue_size=1)
    self.att_control_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rot', DesireCoord, queue_size=1)
    self.mocap_sub = rospy.Subscriber(topic_name, PoseStamped, self.mocapCb)
    self.flight_state_sub = rospy.Subscriber('/'+self.robot_name+'/flight_state', UInt8, self.flight_stateCb)
    self.robot_pos_sub = rospy.Subscriber('/'+self.robot_name+'/mocap/pose',PoseStamped, self.robot_posCb)

    self.omega = 2 * math.pi / self.period
    self.velocity = self.omega * self.radius
    #self.velocity = 0.15
    self.nav_rate = rospy.get_param('/'+self.robot_name+'/nav_rate', 20.0) # hz
    self.nav_rate = 1 / self.nav_rate
    self.Hovering = False
    self.flight_state = 0
    self.mocap_pos = None
    self.mocap_init_flag = False
    self.robot_init_flag = False
    self.robot_init_pos = None

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
    if self.flight_state == 5:
      #rospy.loginfo("Hovering")
      self.Hovering = True  
  
  def robot_posCb(self,msg):
    if self.robot_init_flag == False and self.Hovering == True:
      self.robot_init_pos = msg.pose.position
      self.robot_init_flag = True
      rospy.loginfo("robot_init_pos is [%f, %f, %f]",self.robot_init_pos.x, self.robot_init_pos.y, self.robot_init_pos.z)
     
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
          self.mocap_init_flag = False
          continue
      
      if self.mocap_init_flag == False and self.Hovering == True:
        mocap_init_pos = copy.deepcopy(self.mocap_pos)
        rospy.loginfo("mocap_init_pos is [%f, %f, %f]",mocap_init_pos.x, mocap_init_pos.y, mocap_init_pos.z)
        self.mocap_init_flag = True

      if self.mocap_init_flag==True and self.robot_init_flag==True:
        self.flight_nav.target_pos_x = (self.mocap_pos.x - mocap_init_pos.x + self.robot_init_pos.x) * self.pos_scaling
        self.flight_nav.target_pos_y = (self.mocap_pos.y - mocap_init_pos.y + self.robot_init_pos.y) * self.pos_scaling
        self.flight_nav.target_pos_z = (self.mocap_pos.z - mocap_init_pos.z + self.robot_init_pos.z)
        '''
        self.flight_nav.target_vel_x = self.velocity
        self.flight_nav.target_vel_y = self.velocity
        self.flight_nav.target_vel_z = self.velocity
        '''
        self.flight_nav.target_yaw = self.mocap_euler[2]
        #self.flight_nav.target_omega_z = self.omega
        #if self.flight_nav.target_pos_z <= 0.2:
          #self.flight_nav.target_pos_z = 0.2
      #rospy.loginfo("target_pos is [%f, %f, %f]", self.flight_nav.target_pos_x, self.flight_nav.target_pos_y, self.flight_nav.target_pos_z)

      if self.mocap_init_flag==True and self.robot_init_flag==True and self.Hovering == True:
        self.nav_pub.publish(self.flight_nav)
        #self.att_control_pub.publish(self.desire_att)

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
        self.flight_nav.target_omega_z = 0.0
        self.nav_pub.publish(self.flight_nav)
        #self.att_control_pub.publish(self.desire_att)

      time.sleep(self.nav_rate)

if __name__ == "__main__":

  rospy.init_node("dragon_mocap_ctrl")

  Tracker = mocap_control()
  Tracker.main()

