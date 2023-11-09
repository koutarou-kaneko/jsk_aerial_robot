#!/usr/bin/env python

import sys
import time
import rospy
import math
import signal
import copy
from sensor_msgs.msg import JointState

class fusion_avatar_optimize():
  
  def __init__(self):
    self.debug = rospy.get_param("~debug", False)
    topic_name = '/hydrus/joints_ctrl'
    if self.debug:
      topic_name = '/hydrus/joint_states'

    self.avatar_pos_sub = rospy.Subscriber('/hydrus/avatar_pos', JointState, self.avatar_pos_Cb)
    self.optimize_pos_sub = rospy.Subscriber('/hydrus/opt_joint_angles', JointState, self.optimize_pos_Cb)
    self.target_pos_pub = rospy.Publisher(topic_name, JointState, queue_size=10)

    self.avatar_joint_angles = JointState()
    self.opt_joint_angles = JointState()
    self.target_joint_angles = JointState()
    self.target_joint_pos_list = []
    self.A = 0

  def avatar_pos_Cb(self,msg):
    self.avatar_joint_angles = msg
    self.target_joint_angles = copy.deepcopy(self.avatar_joint_angles)
    self.target_joint_pos_list = list(self.target_joint_angles.position)

  def optimize_pos_Cb(self,msg):
    self.opt_joint_angles = msg

  def balance_decision(self):
    self.A = 0.5

  def main(self):
    r = rospy.Rate(40)
    while not rospy.is_shutdown():  
      self.balance_decision()

      '''
      print("avatar = ", self.avatar_joint_angles)
      print("optimized = ", self.opt_joint_angles)
      print(len(self.opt_joint_angles.position))
      '''
      for i in range(len(self.opt_joint_angles.position)):
        avatar_pos = self.avatar_joint_angles.position[i]
        opt_pos = self.opt_joint_angles.position[i]
        target_pos = avatar_pos*self.A + opt_pos*(1-self.A)
        self.target_joint_pos_list[i] = target_pos
      self.target_joint_angles.position = tuple(self.target_joint_pos_list)

      self.target_pos_pub.publish(self.target_joint_angles)
      r.sleep()

if __name__ == "__main__":
  rospy.init_node("fusion_avatar_optimize")
  Tracker = fusion_avatar_optimize()
  Tracker.main()  

