#!/usr/bin/env python

from __future__ import print_function
import queue # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty, Int8
from geometry_msgs.msg import WrenchStamped
from aerial_robot_msgs.msg import FlightNav
import rosgraph




msg = """
Instruction:

---------------------------

r:  arming motor (please do before takeoff)
t:  takeoff
l:  land
f:  force landing
h:  halt (force stop motor)

     q           w           e           [
(turn left)  (forward)  (turn right)  (move up)

     a           s           d           ]
(move left)  (backward) (move right) (move down)

     m                       n
(hand force on)         (hand force off)  


Please don't have caps lock on.
CTRL+c to quit
---------------------------
"""

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def printMsg(msg, msg_len = 50):
        print(msg.ljust(msg_len) + "\r", end="")

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("keyboard_command")
        robot_ns = rospy.get_param("~robot_ns", "");
        print(msg)

        if not robot_ns:
                master = rosgraph.Master('/rostopic')
                try:
                        _, subs, _ = master.getSystemState()

                except socket.error:
                        raise ROSTopicIOException("Unable to communicate with master!")

                teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
                if len(teleop_topics) == 1:
                        robot_ns = teleop_topics[0].split('/teleop')[0]

        ns = robot_ns + "/teleop_command"
        land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)
        halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
        start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
        takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
        force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)
        nav_pub = rospy.Publisher(robot_ns + '/uav/nav', FlightNav, queue_size=1)
        hand_force_switch_pub = rospy.Publisher(robot_ns + '/hand_force_switch', Int8, queue_size=1)
        hand_force_pub = rospy.Publisher(robot_ns + '/desire_wrench', WrenchStamped, queue_size=1)
        xy_vel   = rospy.get_param("xy_vel", 0.2)
        z_vel    = rospy.get_param("z_vel", 0.2)
        yaw_vel  = rospy.get_param("yaw_vel", 0.2)

        motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)

        try:
                while(True):
                        nav_msg = FlightNav()
                        nav_msg.control_frame = FlightNav.WORLD_FRAME
                        nav_msg.target = FlightNav.COG
                        hand_force_switch_msg = Int8()
                        hand_force_msg = WrenchStamped()


                        key = getKey()

                        msg = ""

                        if key == 'l':
                                land_pub.publish(Empty())
                                msg = "send land command"
                        if key == 'r':
                                start_pub.publish(Empty())
                                msg = "send motor-arming command"
                        if key == 'h':
                                halt_pub.publish(Empty())
                                msg = "send motor-disarming (halt) command"
                        if key == 'f':
                                force_landing_pub.publish(Empty())
                                msg = "send force landing command"
                        if key == 't':
                                takeoff_pub.publish(Empty())
                                msg = "send takeoff command"
                        if key == 'x':
                                motion_start_pub.publish()
                                msg = "send task-start command"
                        if key == 'w':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_x = xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +x vel command"
                        if key == 's':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_x = -xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -x vel command"
                        if key == 'a':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_y = xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +y vel command"
                        if key == 'd':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_y = -xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -y vel command"
                        if key == 'q':
                                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_omega_z = yaw_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +yaw vel command"
                        if key == 'e':
                                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_omega_z = -yaw_vel
                                msg = "send -yaw vel command"
                                nav_pub.publish(nav_msg)
                        if key == '[':
                                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_z = z_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +z vel command"
                        if key == ']':
                                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_z = -z_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -z vel command"
                        if key == 'm':
                                hand_force_switch_msg.data = 1
                                hand_force_switch_pub.publish(hand_force_switch_msg)
                                msg = "hand force ON"
                        if key == 'n':
                                hand_force_switch_msg.data = 0
                                hand_force_switch_pub.publish(hand_force_switch_msg)
                                msg = "hand force OFF"
                        if key == '1':
                                hand_force_msg.wrench.force.x = 0.1
                                hand_force_pub.publish(hand_force_msg)
                                msg = "hand froce = 0.1N"
                        if key == '2':
                                hand_force_msg.wrench.force.x = 0.2
                                hand_force_pub.publish(hand_force_msg)
                                msg = "hand froce = 0.2N"
                        if key == '3':
                                hand_force_msg.wrench.force.x = 0.3
                                hand_force_pub.publish(hand_force_msg)
                                msg = "hand froce = 0.3N"
                        if key == '4':
                                hand_force_msg.wrench.force.x = 0.4
                                hand_force_pub.publish(hand_force_msg)
                                msg = "hand froce = 0.4N"
                        if key == '5':
                                hand_force_msg.wrench.force.x = 0.5
                                hand_force_pub.publish(hand_force_msg)
                                msg = "hand froce = 0.5N"
                        if key == '6':
                                hand_force_msg.wrench.force.x = 0.6
                                hand_force_pub.publish(hand_force_msg)
                                msg = "hand froce = 0.6N"
                        if key == '7':
                                hand_force_msg.wrench.force.x = 0.7
                                hand_force_pub.publish(hand_force_msg)
                                msg = "hand froce = 0.7N"
                        if key == '8':
                                hand_force_msg.wrench.force.x = 0.8
                                hand_force_pub.publish(hand_force_msg)
                                msg = "hand froce = 0.8N"
                        if key == '9':
                                hand_force_msg.wrench.force.x = 0.9
                                hand_force_pub.publish(hand_force_msg)
                                msg = "hand froce = 0.9N"
                        if key == '0':
                                hand_force_msg.wrench.force.x = 1.0
                                hand_force_pub.publish(hand_force_msg)
                                msg = "hand froce = 1.0N"
        


                        if key == '\x03':
                                break

                        printMsg(msg)
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


