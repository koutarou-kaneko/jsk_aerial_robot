#!/bin/sh

rosrun aerial_robot_base rosbag_raw_sensors.sh /zed/odom /realsense1/odom/throttle /realsense1/odom /realsense2/odom/throttle /realsense2/odom /camera/hsv_color_filter/image/compressed /camera/usb_cam/image_raw/compressed
