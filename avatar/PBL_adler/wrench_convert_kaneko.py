#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header
from std_srvs.srv import Empty

class wrench_converter():

  def __init__(self):

    self.hight = rospy.get_param("~grip_hight", 0.445)
    self.sensor2_flag = rospy.get_param("~use_2_sensors", True)
    
    self.ft_sensor_1_sub = rospy.Subscriber('/force1/cfs/data', WrenchStamped, self.ft_sensor_1_Cb) # upper 
    self.ft_sensor_2_sub = rospy.Subscriber('/force2/cfs/data', WrenchStamped, self.ft_sensor_2_Cb) # lower
    self.moment_pub = rospy.Publisher('/moment', Float32MultiArray, queue_size=10)
    self.pulldown_force_pub = rospy.Publisher('/pulldown_force', Float32, queue_size=10)
    self.abs_force_pub = rospy.Publisher('/abs_force', Float32, queue_size=10)
    self.error_pub = rospy.Publisher('/error', WrenchStamped, queue_size=10)
    self.converted_val_pub = rospy.Publisher('/converted_val', WrenchStamped, queue_size=10)
    self.error_ratio_pub = rospy.Publisher('/error_ratio', WrenchStamped, queue_size=10)

    self.ft_sensor_1_val = np.zeros(6)
    self.ft_sensor_2_val = np.zeros(6)

  def ft_sensor_1_Cb(self,msg):
    force_x = msg.wrench.force.x
    force_y = msg.wrench.force.y
    force_z = msg.wrench.force.z
    torque_x = msg.wrench.torque.x
    torque_y = msg.wrench.torque.y
    torque_z = msg.wrench.torque.z
    self.ft_sensor_1_val = np.array([force_x, force_y, force_z, torque_x, torque_y, torque_z])

  def ft_sensor_2_Cb(self,msg):
    force_x = msg.wrench.force.x
    force_y = msg.wrench.force.y
    force_z = msg.wrench.force.z
    torque_x = msg.wrench.torque.x
    torque_y = msg.wrench.torque.y
    torque_z = msg.wrench.torque.z
    self.ft_sensor_2_val = np.array([force_x, force_y, force_z, torque_x, torque_y, torque_z]) # true value

  # def delay_param_Cb(self,msg):
  #   self.delay_param = msg.data
    
  def call_calib(self):
    rospy.wait_for_service('/force1/cfs_sensor_calib')
    try:
      service_1 = rospy.ServiceProxy('/force1/cfs_sensor_calib', Empty)
      response = service_1()
    except rospy.ServiceException:
      print('sensor1 cfs_calib service calling failed')

    if self.sensor2_flag:
      rospy.wait_for_service('/force2/cfs_sensor_calib')
      try:
        service_2 = rospy.ServiceProxy('/force2/cfs_sensor_calib', Empty)
        response = service_2()
      except rospy.ServiceException:
        print('sensor2 cfs_calib service calling failed')


  def main(self):
    r = rospy.Rate(100)

    # calibration
    self.call_calib()

    R = np.array([0, 0, self.hight])
    r_m = np.zeros(3)
    r_fxy = 0.0
    r_f = 0.0
    converted_val = np.zeros(6)
    error = np.zeros(6)
    error_ratio = np.zeros(6)
    r_m_msg = Float32MultiArray()
    r_fxy_msg = Float32()
    r_f_msg = Float32()
    error_msg = WrenchStamped()
    converted_val_msg = WrenchStamped()
    error_ratio_msg = WrenchStamped()
    
    while not rospy.is_shutdown():

      r_m = self.ft_sensor_1_val[-3:] + np.cross(R, self.ft_sensor_1_val[0:3]) # torque
      converted_val = np.concatenate((self.ft_sensor_1_val[0:3], r_m))
      r_fxy = np.sqrt(self.ft_sensor_1_val[0]**2 + self.ft_sensor_1_val[1]**2) # pull-down force
      r_f = np.sqrt(np.dot(self.ft_sensor_1_val[0:3], self.ft_sensor_1_val[0:3].T)) # abs force
      if self.sensor2_flag:
        error = np.subtract(converted_val, self.ft_sensor_2_val)
        for i in range(len(self.ft_sensor_2_val)):
          if self.ft_sensor_2_val[i] != 0:
            error_ratio[i] = error[i]/self.ft_sensor_2_val[i]

      r_m_msg.data = r_m
      r_fxy_msg = r_fxy
      r_f_msg = r_f
      converted_val_msg.wrench.force.x = converted_val[0]
      converted_val_msg.wrench.force.y = converted_val[1]
      converted_val_msg.wrench.force.z = converted_val[2]
      converted_val_msg.wrench.torque.x = converted_val[3]
      converted_val_msg.wrench.torque.y = converted_val[4]
      converted_val_msg.wrench.torque.z = converted_val[5]
      if self.sensor2_flag:
        error_msg.wrench.force.x = error[0]
        error_msg.wrench.force.y = error[1]
        error_msg.wrench.force.z = error[2]
        error_msg.wrench.torque.x = error[3]
        error_msg.wrench.torque.y = error[4]
        error_msg.wrench.torque.z = error[5]
        error_ratio_msg.wrench.force.x = error_ratio[0]
        error_ratio_msg.wrench.force.y = error_ratio[1]
        error_ratio_msg.wrench.force.z = error_ratio[2]
        error_ratio_msg.wrench.torque.x = error_ratio[3]
        error_ratio_msg.wrench.torque.y = error_ratio[4]
        error_ratio_msg.wrench.torque.z = error_ratio[5]

      self.moment_pub.publish(r_m_msg)
      self.pulldown_force_pub.publish(r_fxy_msg)
      self.abs_force_pub.publish(r_f_msg)
      self.converted_val_pub.publish(converted_val_msg)
      if self.sensor2_flag:
        self.error_pub.publish(error_msg)
        self.error_ratio_pub.publish(error_ratio_msg)

      r.sleep()

if __name__ == "__main__":
  rospy.init_node("wrench_converter")
  Tracker = wrench_converter()
  Tracker.main()
