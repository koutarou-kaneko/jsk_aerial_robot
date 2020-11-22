#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import numpy as np
from IPython import embed
import time
import math

from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Empty, Header
from geometry_msgs.msg import Vector3Stamped, Vector3, PoseStamped, Pose, Point, Quaternion
from std_srvs.srv import Trigger, SetBool, SetBoolRequest

def RotMat(t):
    return np.matrix([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])

class HydrusCommander():
    def __init__(self, nav_mode=2, name="hydrus_commander"):
        # define publisher
        self.start_pub = rospy.Publisher("/hydrus_xi/teleop_command/start", Empty, queue_size=1)
        self.nav_control_pub = rospy.Publisher("/hydrus_xi/uav/nav", FlightNav, queue_size=1)
        self.takeoff_pub = rospy.Publisher("/hydrus_xi/teleop_command/takeoff", Empty,  queue_size=1)
        self.land_pub = rospy.Publisher("/hydrus_xi/teleop_command/land", Empty,  queue_size=1)
        self.halt_pub = rospy.Publisher("/hydrus_xi/teleop_command/halt", Empty, queue_size=1)
        self.manip_pub = rospy.Publisher("/hydrus_xi/manipulation_end", PoseStamped, queue_size=1)

        self.set_joint_torque_client = rospy.ServiceProxy("/hydrus_xi/joints/torque_enable", SetBool)
        self.extra_servos_ctrl_pub = rospy.Publisher("/hydrus_xi/extra_servos_ctrl", JointState, queue_size=1)
        self.joints_ctrl_pub = rospy.Publisher("/hydrus_xi/joints_ctrl", JointState, queue_size=1)

        #self.cover_pose = rospy.get_param('~cover_pose')
        #self.close_pose = rospy.get_param('~close_pose')
        self.nav_mode = nav_mode

        # constants
        self.WAIT_TIME = 0.5

    def arm_and_takeoff(self):
        # send arm and takeoff
        time.sleep(self.WAIT_TIME)
        self.start_pub.publish(Empty())
        time.sleep(self.WAIT_TIME)
        self.takeoff_pub.publish(Empty())
        time.sleep(self.WAIT_TIME)

    def arm(self):
        # send arm
        time.sleep(self.WAIT_TIME)
        self.start_pub.publish(Empty())
        time.sleep(self.WAIT_TIME)

    def takeoff(self):
        # send arm
        time.sleep(self.WAIT_TIME)
        self.takeoff_pub.publish(Empty())
        time.sleep(self.WAIT_TIME)

    def land(self):
        time.sleep(self.WAIT_TIME)
        self.land_pub.publish(Empty())
        time.sleep(self.WAIT_TIME)

    def halt(self):
        time.sleep(self.WAIT_TIME)
        self.halt_pub.publish(Empty())
        time.sleep(self.WAIT_TIME)

    def set_joint_torque(self, state):
        req = SetBoolRequest()
        req.data = state
        try:
            self.set_joint_torque_client(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        time.sleep(self.WAIT_TIME)

    def move_to(self, pos_x, pos_y, override_nav_mode=None):
        """move to target x, y position"""
        # send desired position
        nav_msg = FlightNav()

        if override_nav_mode is None:
            nav_mode = self.nav_mode
        else:
            nav_mode = override_nav_mode

        nav_msg.target = nav_msg.COG
        nav_msg.pos_xy_nav_mode = nav_mode
        nav_msg.target_pos_x = pos_x
        nav_msg.target_pos_y = pos_y
        time.sleep(self.WAIT_TIME)
        self.nav_control_pub.publish(nav_msg)
        time.sleep(self.WAIT_TIME)

    '''
    def target_pos_error(self):
        #return target pos error [x,y,z]
        controller_debug_msg = rospy.wait_for_message("/controller/debug", FlatnessPid)
        return [controller_debug_msg.pitch.pos_err,
                controller_debug_msg.roll.pos_err,
                controller_debug_msg.throttle.pos_err]
    '''

    def change_height(self, pos_z):
        """ change height """
        nav_msg = FlightNav()
        nav_msg.target = nav_msg.COG
        nav_msg.pos_z_nav_mode = nav_msg.POS_MODE
        nav_msg.target_pos_z = pos_z
        time.sleep(self.WAIT_TIME)
        self.nav_control_pub.publish(nav_msg)
        time.sleep(self.WAIT_TIME)

    def change_yaw(self, yaw):
        """ change yaw """
        nav_msg = FlightNav()
        nav_msg.yaw_nav_mode = nav_msg.POS_MODE
        nav_msg.target_yaw = yaw
        time.sleep(self.WAIT_TIME)
        self.nav_control_pub.publish(nav_msg)
        time.sleep(self.WAIT_TIME)

    def move_camera(self, angle_rad):
        """ move camera to angle_rad """
        joint_msg = JointState()
        joint_msg.position = [angle_rad]
        time.sleep(self.WAIT_TIME)
        self.extra_servos_ctrl_pub.publish(joint_msg)
        time.sleep(self.WAIT_TIME)

    def joint_publish(self, joint_state):
        """publish joint_state list"""
        joint_msg = JointState()
        joint_msg.position = joint_state
        time.sleep(self.WAIT_TIME)
        self.joints_ctrl_pub.publish(joint_msg)
        time.sleep(self.WAIT_TIME)

    def door_pose(self):
        self.joint_publish([0.78, 1.57, 1.57])

    '''
    def open_joints(self):
        """open links"""
        self.joint_publish(self.cover_pose)

    def close_joints(self):
        """close links"""
        self.joint_publish(self.close_pose)
    '''

    def covering_motion(self,pos_x, pos_y, cog_yaw, covering_pre_height, covering_post_height, covering_move_dist):
        dest_yaw = cog_yaw + 0.785 # correct forward angle of open form
        forward_dir = [-covering_move_dist*math.sin(dest_yaw), covering_move_dist*math.cos(dest_yaw)]
        self.change_height(covering_pre_height)
        self.move_to(pos_x-forward_dir[0], pos_y-forward_dir[1], override_nav_mode=2)
        time.sleep(5)
        self.open_joints()
#        time.sleep(1)
#        self.change_height(covering_pre_height)
#        time.sleep(1)

        # back = 1.2
        # self.move_to(pos_x-back*forward_dir[0], pos_y-back*forward_dir[1], override_nav_mode=2)
        # self.change_height(covering_post_height)
        self.land()
        time.sleep(10)
        self.halt()

    def ik(self, des_x, des_y, des_yaw):
        #todo: use tf
        l1=0.67
        l2=0.6
        l3=0.6
        l4=0.9
        solfound=True
        joints=np.array([1.57,1.57,1.57])
        link1=np.array([[l1], [0]])
        link2=np.array([[l2], [0]])
        link3=np.array([[l3], [0]])
        link4=np.array([[l4], [0]])
        des=np.array([[des_x], [des_y]])
        d=0.02 #[rad]

        for i in range(26):
            l23=des+RotMat(des_yaw+i*d)*link4-link1
            if (l2+l3)>=np.linalg.norm(l23):
                theta=np.arccos(np.linalg.norm(l23)/2/l2)
                joints[0]=np.arctan2(l23[1], l23[0])+theta
                joints[1]=-2*theta
                joints[2]=des_yaw+i*d-np.pi-joints[0]-joints[1]
                if (joints < 1.57).all() and (joints > -1.57).all():
                    q=tf.transformations.quaternion_about_axis(des_yaw+i*d, (0,0,1))
                    self.manip_pub.publish(PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='hydrus_xi/link4'),pose=Pose(position=Point(x=l1-des_x, y=-des_y, z=0), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))))
                    print "Found solution: tol +%d" % i
                    break
                joints[0]=np.arctan2(l23[1], l23[0])-theta
                joints[1]=2*theta
                joints[2]=des_yaw+i*d-np.pi-joints[0]-joints[1]
                if (joints < 1.57).all() and (joints > -1.57).all():
                    q=tf.transformations.quaternion_about_axis(des_yaw+i*d, (0,0,1))
                    self.manip_pub.publish(PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='hydrus_xi/link4'),pose=Pose(position=Point(x=l1-des_x, y=-des_y, z=0), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))))
                    print "Found solution: tol +%d" % i
                    break
            l23=des+RotMat(des_yaw-i*d)*link4-link1
            if (l2+l3)>=np.linalg.norm(l23):
                theta=np.arccos(np.linalg.norm(l23)/2/l2)
                joints[0]=np.arctan2(l23[1], l23[0])+theta
                joints[1]=-2*theta
                joints[2]=des_yaw-i*d-np.pi-joints[0]-joints[1]
                if (joints < 1.57).all() and (joints > -1.57).all():
                    q=tf.transformations.quaternion_about_axis(des_yaw-i*d, (0,0,1))
                    self.manip_pub.publish(PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='hydrus_xi/link4'),pose=Pose(position=Point(x=l1-des_x, y=-des_y, z=0), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))))
                    print "Found solution: tol -%d" % i
                    break
                joints[0]=np.arctan2(l23[1], l23[0])-theta
                joints[1]=2*theta
                joints[2]=des_yaw-i*d-np.pi-joints[0]-joints[1]
                if (joints < 1.57).all() and (joints > -1.57).all():
                    q=tf.transformations.quaternion_about_axis(des_yaw-i*d, (0,0,1))
                    self.manip_pub.publish(PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='hydrus_xi/link4'),pose=Pose(position=Point(x=l1-des_x, y=-des_y, z=0), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))))
                    print "Found solution: tol -%d" % i
                    break
            if i==25:
                print "Could not find solution"
                solfound=False
        if solfound:
            #Reversed since link order is different
            self.joint_publish([-joints[2], -joints[1], -joints[0]])
        
    def ik_array(self, start, end, n, dt):
        dx   = (end[0]-start[0])/n
        dy   = (end[1]-start[1])/n
        dyaw = (end[2]-start[2])/n
        for i in range(n+1):
            self.ik(start[0]+i*dx, start[1]+i*dy, start[2]+i*dyaw)
            rospy.sleep(rospy.Duration(dt))

def sendFFWrench(pub, fx, fy, tz):
    pub.publish(Vector3(x=fx, y=fy, z=tz))

if __name__ == '__main__':
    rospy.init_node("interactive_test")
    pub = rospy.Publisher("/hydrus_xi/ff_wrench", Vector3, queue_size=10)
    rospy.sleep(rospy.Duration(5.0))
    hyd = HydrusCommander()
    rospy.sleep(rospy.Duration(1.0))
    '''
    hyd.arm_and_takeoff()
    rospy.sleep(rospy.Duration(15.0))
    hyd.change_yaw(1.57)
    rospy.sleep(rospy.Duration(2.0))
    hyd.move_to(1,1)
    rospy.sleep(rospy.Duration(2.0))
    hyd.joint_publish([0, 1.57, 1.57])
    rospy.sleep(rospy.Duration(1.0))
    hyd.move_to(1,-0.5)
    '''
    embed()
