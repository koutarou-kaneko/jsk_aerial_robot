#!/usr/bin/env python

import os
import rospy
import xacro
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler, quaternion_multiply
from control_msgs.msg import FollowJointTrajectoryActionGoal

script_dir = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(script_dir, "../../TCP-IP-ROS-6AXis/dobot_gazebo/urdf/me6_robot.xacro")

class MocapTeleopME6Test:
    def __init__(self, urdf_path):
        rospy.init_node("me6_teleop_from_mocap")

        try:
            robot_description_config = xacro.process_file(urdf_path)
            robot_description_xml = robot_description_config.toxml()
            rospy.set_param("/robot_description", robot_description_xml)
            rospy.loginfo(f"URDF loaded to /robot_description from {urdf_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load URDF: {e}")
            rospy.signal_shutdown("URDF load failed")
            return

        self.ik_solver = IK("base_link", "Link6")
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        self.initial_joint_received = False
        self.listener = tf.TransformListener()
        self.gazebo_joint_pub = rospy.Publisher("/me6_robot/joint_controller/command", JointTrajectory, queue_size=1)
        self.real_machine_joint_pub = rospy.Publisher("/me6_robot/joint_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=10)

        rospy.Subscriber("/twin_hammer/mocap/pose", PoseStamped, self.mocap_cb)
        rospy.Subscriber("/me6_robot/joint_states", JointState, self.jointstate_cb)

    def jointstate_cb(self, msg: JointState):
        if not self.initial_joint_received:
            joint_positions = []
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    joint_positions.append(msg.position[idx])
                else:
                    joint_positions.append(0.0)
            self.last_joint_positions = joint_positions

            try:
                rospy.sleep(0.5)
                (trans, rot) = self.listener.lookupTransform("base_link", "Link6", rospy.Time(0))
                self.center_pos = np.array(trans)
                self.center_quat = np.array(rot)
                rospy.loginfo(f"Initial end-effector position from TF: {self.center_pos}")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to get initial end-effector pose from TF")
                self.center_pos = np.zeros(3)
                self.center_quat = np.array([0,0,0,1])

            self.initial_joint_received = True

    def mocap_cb(self, msg: PoseStamped):
        if not self.initial_joint_received:
          rospy.logwarn("Failed to initialize")
          return

        target_pos = ([0.2,0.2,0.3])
        target_quat = np.array([0.0, 0.0, 0.0, 1.0])

        sol = self.ik_solver.get_ik(
            self.last_joint_positions,
            target_pos[0], target_pos[1], target_pos[2],
            target_quat[0], target_quat[1], target_quat[2], target_quat[3]
        )

        if sol:
            point = JointTrajectoryPoint()
            point.positions = sol
            point.time_from_start = rospy.Duration(1.0)
            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            traj.points.append(point)
            self.gazebo_joint_pub.publish(traj)
            goal_msg = FollowJointTrajectoryActionGoal()
            goal_msg.goal.trajectory.joint_names = self.joint_names
            goal_msg.goal.trajectory.points.append(point)
            self.real_machine_joint_pub.publish(goal_msg)
            
        else:
            rospy.logwarn_throttle(1.0, "IK solution not found for given pose.")

if __name__ == "__main__":
    try:
        teleop = MocapTeleopME6Test(urdf_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
