#!/usr/bin/env python

from http import server
import sys
import time
import rospy
import math
import copy
import select, termios, tty
from std_msgs.msg import UInt8
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from spinal.msg import DesireCoord
from std_msgs.msg import Empty
from dynamixel_workbench_msgs.srv import *

class avatar_control():

    def __init__(self):

        self.debug = rospy.get_param("~debug", False)
        topic_name = '/dragon/joints_ctrl'

        if self.debug:
            topic_name = '/dragon/joint_states'

        self.joint_servo_pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory', JointTrajectory, queue_size=10)
        self.joint_control_pub = rospy.Publisher(topic_name, JointState, queue_size=10)
        self.avatar_sub = rospy.Subscriber('/dynamixel_workbench/joint_states', JointState, self.avatarCb)
        self.flight_state_sub = rospy.Subscriber("/dragon/flight_state", UInt8, self.flight_stateCb)

        self.raw_servo_position = None
        self.Hovering = False
        self.servo_Switch_state = True

        self.angle_limit = rospy.get_param("angle_limit", 1.56) # the limitation of the joint
        self.min_yaw_angle = rospy.get_param("min_yaw_angle", 0.3)
        self.yaw_sum_threshold = rospy.get_param("yaw_sum_threshold", 0.9)
        self.servo_init_time = 0.5

    def servo_switch(self, msg, Switch):
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
        for i in range(len(msg.name)):
            try:
                client = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
                resp = client('', i+1, 'Torque_Enable', Switch)
            except rospy.ServiceException as e:
                print("Service call failed: {}".format(e))
        self.servo_Switch_state = Switch

    def set_servo_init(self,msg):
        # set the joint servo to havering
        self.servo_init_position = JointTrajectory()
        self.servo_init_position.joint_names = ["joint1_pitch", "joint1_yaw", "joint2_pitch", "joint2_yaw", "joint3_pitch", "joint3_yaw"]
        self.servo_init_position.points = [JointTrajectoryPoint()]
        self.servo_init_position.points[0].positions = [0.0, 1.56, 0.0, 1.56, 0.0, 1.56]
        self.servo_init_position.points[0].time_from_start = rospy.Time(self.servo_init_time)
        self.joint_servo_pub.publish(self.servo_init_position)
        rospy.sleep(self.servo_init_time * 10)
        rospy.loginfo("joint servo init DONE!")


    def flight_stateCb(self,msg):
        self.flight_state = msg.data
        #rospy.loginfo("%s", self.Hovering)
        #rospy.loginfo("flight_state is %s", self.flight_state)
        if self.flight_state == 5:
            self.Hovering = True 
        '''
        if self.flight_state == 4:
            self.servo_switch(msg, Switch=True)
            self.set_servo_init(msg)
        '''

    def avatarCb(self,msg):

        if self.raw_servo_position is None:
            # set the joint servo on
            self.servo_switch(msg, Switch=True)
            # set the joint servo to havering
            self.set_servo_init(msg)

        # set the joint servo off
        if self.debug==False and self.Hovering==True and self.servo_Switch_state==True:
            self.servo_switch(msg, Switch=False)
            rospy.loginfo("servo off")
        if self.debug==True and self.servo_Switch_state==True:
            self.servo_switch(msg, Switch=False)
            rospy.loginfo("servo off")
        
        self.raw_servo_position = msg
        self.raw_servo_position.name = list(self.raw_servo_position.name)
        self.raw_servo_position.position = list(self.raw_servo_position.position)


    def main(self):

        r = rospy.Rate(40)
        while not rospy.is_shutdown():

            if self.raw_servo_position is None:
                continue

            #avoid over angle 
            desire_joint = copy.deepcopy(self.raw_servo_position)
            for i in range(len(desire_joint.position)):
                if desire_joint.position[i] > self.angle_limit:
                    desire_joint.position[i] = self.angle_limit
                if desire_joint.position[i] < -self.angle_limit:
                    desire_joint.position[i] = -self.angle_limit

            # avoid stright line configuration
            sum = 0
            for i, n in enumerate(desire_joint.name):
                if 'yaw' in n:
                    '''
                    if abs(desire_joint.position[i]) < self.min_yaw_angle:
                        if desire_joint.position[i] >0:
                            desire_joint.position[i] = self.min_yaw_angle
                        if desire_joint.position[i] <0:
                            desire_joint.position[i] = -self.min_yaw_angle
                    '''
                    sum += abs(desire_joint.position[i])


            if sum < self.yaw_sum_threshold:
                #rospy.loginfo("%f", sum)
                '''
                if desire_joint.position[1] >=0:
                    desire_joint.position[1] = 1.2
                else:
                    desire_joint.position[1] = -1.2
                '''
                '''
                for i, n in enumerate(desire_joint.name):
                    if 'yaw' in n:
                        if desire_joint.position[i] >0:
                            desire_joint.position[i] = self.min_yaw_angle
                        if desire_joint.position[i] <0:
                            desire_joint.position[i] = -self.min_yaw_angle
                '''

            # add gimbal angle if necessary
            if self.debug:
                desire_joint.name.extend(['gimbal1_roll', 'gimbal1_pitch', 'gimbal2_roll', 'gimbal2_pitch', 'gimbal3_roll', 'gimbal3_pitch', 'gimbal4_roll', 'gimbal4_pitch'])
                desire_joint.position.extend([0] * 8)

            # send joint angles
            self.joint_control_pub.publish(desire_joint)

            r.sleep()

if __name__ == "__main__":
  rospy.init_node("avatar_control")
  Tracker = avatar_control()
  Tracker.main()


