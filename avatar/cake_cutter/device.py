#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from dynamixel_workbench_msgs.srv import *
from spinal.msg import ServoControlCmd
from std_msgs.msg import String

class device():

  def __init__(self):
    self.device_sub = rospy.Subscriber('/dynamixel_workbench/joint_states', JointState, self.device_Cb)
    self.joint_servo_pub = rospy.Publisher('/servo_cmd_to_VIM4', ServoControlCmd, queue_size=10) # TODO rewrite for controling from spinal
    self.test_pub = rospy.Publisher('/test', String, queue_size=10)
    self.joint_angle = 0.0
    self.init_angle = 200
    self.scale = 1.5
    self.servo_cmd_msg = ServoControlCmd()
    self.test_msg = String()

  def device_Cb(self, msg):
    self.joint_angle = msg.position

  def servo_switch(self,switch,servo_number):
      rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
      try:
          client = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
          resp = client('', servo_number, 'Torque_Enable', switch)
      except rospy.ServiceException as e:
          print("Service call failed: {}".format(e))
      self.servo_Switch_state = switch

  def main(self):
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
      self.servo_switch(switch=False,servo_number=1)
      angle = self.joint_angle[0] * 2048/3.1415 * self.scale
      angle += self.init_angle
      # print(self.joint_angle)
      # self.servo_cmd_msg.angles = [self.joint_angle]
      self.servo_cmd_msg.index = [0]
      self.servo_cmd_msg.angles = [int(angle)]
      self.joint_servo_pub.publish(self.servo_cmd_msg)

      self.test_msg.data = 'test'
      self.test_pub.publish(self.test_msg)
      r.sleep()

if __name__ == "__main__":
  rospy.init_node("teleop")
  Tracker = device()
  Tracker.main()



