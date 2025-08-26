#!/usr/bin/env python

import os
import rospy
import xacro
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
URDF_PATH = os.path.join(SCRIPT_DIR, "../../TCP-IP-ROS-6AXis/dobot_gazebo/urdf/me6_robot.xacro")

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

        self.center_pos = None
        self.device_init_pos = None
        self.device_init_quat = None
        self.last_joint_positions = [0.0] * self.ik_solver.number_of_joints
        self.scale = 0.5

        self.listener = tf.TransformListener()
        self.joint_pub = rospy.Publisher("/me6_robot/joint_controller/command", JointTrajectory, queue_size=1)
        rospy.Subscriber("/twin_hammer/mocap/pose", PoseStamped, self.mocap_cb)
        rospy.Subscriber("/me6_robot/joint_states", JointState, self.jointstate_cb)
        self.initial_joint_received = False
        rospy.loginfo("ME6 teleoperation node initialized.")

    def jointstate_cb(self, msg: JointState):
        if not self.initial_joint_received:
            # 初回受信時に直前関節角として保持
            joint_positions = []
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    joint_positions.append(msg.position[idx])
                else:
                    joint_positions.append(0.0)
            self.last_joint_positions = joint_positions

            # TFからエンドエフェクタ位置を取得
            try:
                rospy.sleep(0.5)  # TFが安定するまで待機
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
            return  # 初期位置未取得なら動作しない

        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) 
        quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

        if self.device_init_pos is None:
          self.device_init_pos = pos
          self.device_init_quat = quat

        delta_pos = (pos - self.device_init_pos) * self.scale
        target_pos = self.center_pos + delta_pos
        target_quat = quat

        sol = self.ik_solver.get_ik(
            self.last_joint_positions,
            target_pos[0], target_pos[1], target_pos[2],
            target_quat[0], target_quat[1], target_quat[2], target_quat[3]
        )

        if sol:
            self.last_joint_positions = sol
            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = sol
            point.time_from_start = rospy.Duration(0.1)
            traj.points.append(point)
            self.joint_pub.publish(traj)
        else:
            rospy.logwarn_throttle(1.0, "IK solution not found for given mocap pose.")

if __name__ == "__main__":
    try:
        teleop = MocapTeleopME6(URDF_PATH)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
