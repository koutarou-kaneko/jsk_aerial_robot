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

class MocapTeleopME6:
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
        self.center_pos = None
        self.device_init_pos = None
        self.device_init_quat = None
        self.last_joint_positions = [0.0] * self.ik_solver.number_of_joints
        self.last_valid_ee_pos = None
        self.teleop_scale = 0.4
        self.wrench_scale = 1.0
        self.offset_pos = np.array([0.0, 0.0, 0.5])
        self.offset_quat = quaternion_from_euler(0, np.pi/2, 0)

        self.listener = tf.TransformListener()
        self.gazebo_joint_pub = rospy.Publisher("/me6_robot/joint_controller/command", JointTrajectory, queue_size=1)
        self.error_feedback_pub = rospy.Publisher("/twin_hammer/haptics_wrench", WrenchStamped, queue_size=1)
        self.real_machine_joint_pub = rospy.Publisher("/me6_robot/joint_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=10)
        self.debug_target_pos_pub = rospy.Publisher("/debug_target_pos", PoseStamped, queue_size=1)

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

        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) 
        quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

        if self.device_init_pos is None:
          self.device_init_pos = pos + self.offset_pos
          self.device_init_quat = quat

        delta_pos = (pos - self.device_init_pos) * self.teleop_scale
        target_pos = self.center_pos + delta_pos
        target_quat = quaternion_multiply(quat, self.offset_quat)

        sol = self.ik_solver.get_ik(
            self.last_joint_positions,
            target_pos[0], target_pos[1], target_pos[2],
            target_quat[0], target_quat[1], target_quat[2], target_quat[3]
        )

        if sol:
            self.last_joint_positions = sol
            self.last_valid_ee_pos = target_pos.copy()
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
            debug_target_pos_msg = PoseStamped()
            debug_target_pos_msg.pose.position.x = target_pos[0]
            debug_target_pos_msg.pose.position.y = target_pos[1]
            debug_target_pos_msg.pose.position.z = target_pos[2]
            self.debug_target_pos_pub.publish(debug_target_pos_msg)

        else:
            rospy.logwarn_throttle(1.0, "IK solution not found for given mocap pose.")
            if self.last_valid_ee_pos is not None:
              error_vec = (target_pos - self.last_valid_ee_pos) * self.wrench_scale
              wrench_msg = WrenchStamped()
              wrench_msg.wrench.force.x = error_vec[0]
              wrench_msg.wrench.force.y = error_vec[1]
              wrench_msg.wrench.force.z = error_vec[2]
              self.error_feedback_pub.publish(wrench_msg)

if __name__ == "__main__":
    try:
        teleop = MocapTeleopME6(urdf_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
