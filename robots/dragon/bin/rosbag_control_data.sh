#!/bin/sh

rosrun aerial_robot_base rosbag_control_data.sh /dragon/joints_ctrl /dragon/joint_states /controller/gimbals_target_force /controller/roll_pitch_gimbal_control /realsense1/odom/throttle /realsense1/odom
