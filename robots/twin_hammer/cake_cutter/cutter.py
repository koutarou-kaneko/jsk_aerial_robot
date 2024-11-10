#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from spinal.msg import ServoControlCmd

class cutter():

  def __init__(self):
    self.angle_sub = rospy.Subscriber('/servo_cmd_to_VIM4', ServoControlCmd, self.angle_cb)
    self.servo_pub = rospy.Publisher('/servo_cmd', ServoControlCmd, queue_size=10)
    self.servo_msg = ServoControlCmd()
  
  def angle_cb(self,msg):
    self.servo_msg.index = msg.index
    self.servo_msg.angles = msg.angles
    self.servo_pub.publish(self.servo_msg)
  
  def main(self):
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
      self.servo_pub.publish(self.servo_msg)
      r.sleep()

if __name__ == "__main__":
  rospy.init_node("cutter")
  Tracker = cutter()
  Tracker.main()
