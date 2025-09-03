#!/usr/bin/env python3
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

if __name__ == "__main__":
    rospy.init_node("me6_direct_send_joint_angles")

    client = actionlib.SimpleActionClient(
        "/me6_robot/joint_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction
    )
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to server")

    # テスト用の関節角度（ラジアン）
    joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]
    # joint_positions = [3.14, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_positions = [1.57, 0.5, -2.5, 2.0, 1.57, 0.0]
    # joint_positions = [1.57, -1.8, 2.2, -0.4, 1.57, 0.0]

    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = rospy.Duration(3.0)  # 3秒かけて移動

    traj = JointTrajectory()
    traj.joint_names = joint_names
    traj.points.append(point)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj

    # ゴール送信
    client.send_goal(goal)
    rospy.loginfo("Goal sent, waiting for result...")
    client.wait_for_result()
    rospy.loginfo("Done!")
