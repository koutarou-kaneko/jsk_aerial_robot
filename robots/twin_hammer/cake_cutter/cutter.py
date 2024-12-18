#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from spinal.msg import ServoControlCmd, ServoStates
from std_msgs.msg import String
class cutter():

  def __init__(self):
    self.device_angle_sub = rospy.Subscriber('/servo_cmd_to_VIM4', ServoControlCmd, self.device_angle_cb)
    self.robot_angle_sub = rospy.Subscriber('/servo/states', ServoStates, self.robot_angle_cb)
    self.test_sub = rospy.Subscriber('test', String, self.test_cb)
    self.servo_pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=10)
    self.servo_msg = ServoControlCmd()
    self.robot_servo_angle = 0
    self.device_servo_angle = 0
    self.init_angle = 0
    self.init_flag = True

  def test_cb(self,msg):
    str = msg.data
    # print(str)
  
  def device_angle_cb(self,msg):
    self.servo_msg.index = msg.index
    self.device_servo_angle = int(msg.angles[0])
  
  def robot_angle_cb(self,msg):
    # self.robot_servo_angle = msg.servos.angle
    a = 0
    
  def main(self):
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
      if self.init_flag == True:
        self.init_angle = self.robot_servo_angle
        self.init_flag = False
      # print("robot", self.robot_servo_angle)
      self.servo_msg.angles = (self.device_servo_angle,)
      # self.servo_msg.angles = (self.device_servo_angle - self.init_angle,)
      self.servo_pub.publish(self.servo_msg)
      r.sleep()

if __name__ == "__main__":
  rospy.init_node("cutter")
  Tracker = cutter()
  Tracker.main()
