#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from IPython import embed
import time
import math

from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import ServoStates
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
        self.force_landing_pub = rospy.Publisher("/hydrus_xi/teleop_command/force_landing", Empty,  queue_size=1)
        self.halt_pub = rospy.Publisher("/hydrus_xi/teleop_command/halt", Empty, queue_size=1)
        #self.manip_pub = rospy.Publisher("/hydrus_xi/manipulation_end", PoseStamped, queue_size=1)
        self.manip_sub = rospy.Subscriber('/hydrus_xi/manipulation_target', PoseStamped, self.manip_cb)

        self.set_joint_torque_client = rospy.ServiceProxy("/hydrus_xi/joints/torque_enable", SetBool)
        self.extra_servos_ctrl_pub = rospy.Publisher("/hydrus_xi/extra_servos_ctrl", JointState, queue_size=1)
        self.joints_ctrl_pub = rospy.Publisher("/hydrus_xi/joints_ctrl", JointState, queue_size=1)
        self.servo_states_sub = rospy.Subscriber('/hydrus_xi/servo/states', ServoStates, self.servo_states_cb)
        self.joint_states_sub = rospy.Subscriber('/hydrus_xi/joint_states', JointState, self.joint_states_cb)

        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf)

        self.test_mode = rospy.get_param('~test_mode', default='F')
        print "Test Mode: %s" % self.test_mode
        #self.cover_pose = rospy.get_param('~cover_pose')
        #self.close_pose = rospy.get_param('~close_pose')
        self.nav_mode = nav_mode
        self.errors = [False, False, False]
        self.joint_angles_now = [1.4, 1.57, 1.57]
        self.last_pose = [0.05, -0.25, 1.57]

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

    def joint_publish(self, joint_state, short_waittime=False):
        """publish joint_state list"""
        joint_msg = JointState()
        joint_msg.position = joint_state
        if short_waittime:
            time.sleep(0.05)
        else:
            time.sleep(self.WAIT_TIME)
        self.joints_ctrl_pub.publish(joint_msg)
        if short_waittime:
            time.sleep(0.05)
        else:
            time.sleep(self.WAIT_TIME)

    def servo_states_cb(self, msg):
        for i, servo in enumerate(msg.servos):
            #self.joint_angles_now[i] = 1.5708 * (servo.angle/2047.0 - 1)
            if servo.error != 0:
                print "Servo error, force landing, index: %d" % servo.index
                self.force_landing_pub.publish(Empty())
                time.sleep(self.WAIT_TIME)
                self.errors[i] = True
            else:
                self.errors[i] = False

    def joint_states_cb(self, msg):
        self.joint_angles_now[0] = msg.position[4]
        self.joint_angles_now[1] = msg.position[5]
        self.joint_angles_now[2] = msg.position[6]

    def covering_motion(self,pos_x, pos_y, cog_yaw, covering_pre_height, covering_post_height, covering_move_dist):
        dest_yaw = cog_yaw + 0.785 # correct forward angle of open form
        forward_dir = [-covering_move_dist*math.sin(dest_yaw), covering_move_dist*math.cos(dest_yaw)]
        self.change_height(covering_pre_height)
        self.move_to(pos_x-forward_dir[0], pos_y-forward_dir[1], override_nav_mode=2)
        time.sleep(5)
        self.open_joints()
        self.land()
        time.sleep(10)
        self.halt()

    def joint_sanitize(self, angle):
        if angle > np.pi:
            return angle-2*np.pi
        elif angle < -np.pi:
            return angle+2*np.pi
        else:
            return angle

    def fk(self, joint_angles):
        l1=0.68
        l2=0.6
        l3=0.6
        l4=0.9
        link1=np.array([[l1], [0]])
        link2=np.array([[l2], [0]])
        link3=np.array([[l3], [0]])
        link4=np.array([[l4], [0]])
        end = link1 + RotMat(joint_angles[0])*link2 + RotMat(joint_angles[0]+joint_angles[1])*link3 + RotMat(joint_angles[0]+joint_angles[1]+joint_angles[2])*link4
        return [end[0,0], end[1,0], self.joint_sanitize(joint_angles[0]+joint_angles[1]+joint_angles[2])]

    def ik(self, des_x, des_y, des_yaw):
        #todo: use tf
        l1=0.68
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
            l23=des-RotMat(des_yaw+i*d)*link4-link1
            if (l2+l3)>=np.linalg.norm(l23):
                theta=np.arccos(np.linalg.norm(l23)/2/l2)
                joints[0]=self.joint_sanitize(np.arctan2(l23[1], l23[0])+theta)
                joints[1]=self.joint_sanitize(-2*theta)
                joints[2]=self.joint_sanitize(des_yaw+i*d-joints[0]-joints[1])
                if (joints < 1.57).all() and (joints > -1.57).all():
                    q=tf.transformations.quaternion_about_axis(des_yaw+i*d, (0,0,1))
                    #self.manip_pub.publish(PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='hydrus_xi/link1'),pose=Pose(position=Point(x=des_x-(l1-l2), y=des_y, z=0), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))))
                    print "Found solution: tol +%d" % i
                    self.last_pose = [des_x, des_y, des_yaw+i*d]
                    break
                #else:
                #    print "failed: %lf %lf %lf" % (joints[0], joints[1], joints[2])
                joints[0]=self.joint_sanitize(np.arctan2(l23[1], l23[0])-theta)
                joints[1]=self.joint_sanitize(2*theta)
                joints[2]=self.joint_sanitize(des_yaw+i*d-joints[0]-joints[1])
                if (joints < 1.57).all() and (joints > -1.57).all():
                    q=tf.transformations.quaternion_about_axis(des_yaw+i*d, (0,0,1))
                    #self.manip_pub.publish(PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='hydrus_xi/link1'),pose=Pose(position=Point(x=des_x-(l1-l2), y=des_y, z=0), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))))
                    print "Found solution: tol +%d" % i
                    self.last_pose = [des_x, des_y, des_yaw+i*d]
                    break
                #else:
                #    print "failed: %lf %lf %lf" % (joints[0], joints[1], joints[2])
            l23=des+RotMat(des_yaw-i*d)*link4-link1
            if (l2+l3)>=np.linalg.norm(l23):
                theta=np.arccos(np.linalg.norm(l23)/2/l2)
                joints[0]=self.joint_sanitize(np.arctan2(l23[1], l23[0])+theta)
                joints[1]=self.joint_sanitize(-2*theta)
                joints[2]=self.joint_sanitize(des_yaw-i*d-joints[0]-joints[1])
                if (joints < 1.57).all() and (joints > -1.57).all():
                    q=tf.transformations.quaternion_about_axis(des_yaw-i*d, (0,0,1))
                    #self.manip_pub.publish(PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='hydrus_xi/link1'),pose=Pose(position=Point(x=des_x-(l1-l2), y=des_y, z=0), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))))
                    print "Found solution: tol -%d" % i
                    self.last_pose = [des_x, des_y, des_yaw-i*d]
                    break
                #else:
                #    print "failed: %lf %lf %lf" % (joints[0], joints[1], joints[2])
                joints[0]=self.joint_sanitize(np.arctan2(l23[1], l23[0])-theta)
                joints[1]=self.joint_sanitize(2*theta)
                joints[2]=self.joint_sanitize(des_yaw-i*d-joints[0]-joints[1])
                if (joints < 1.57).all() and (joints > -1.57).all():
                    q=tf.transformations.quaternion_about_axis(des_yaw-i*d, (0,0,1))
                    #self.manip_pub.publish(PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='hydrus_xi/link1'),pose=Pose(position=Point(x=des_x-(l1-l2), y=des_y, z=0), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))))
                    print "Found solution: tol -%d" % i
                    self.last_pose = [des_x, des_y, des_yaw-i*d]
                    break
                #else:
                #    print "failed: %lf %lf %lf" % (joints[0], joints[1], joints[2])
            if i==25:
                print "Could not find solution: (x, y, yaw) = (%f, %f, %f)" % (des_x, des_y, des_yaw)
                solfound=False
        if solfound:
            self.joint_publish([joints[0], joints[1], joints[2]], short_waittime=True)
        
    def ik_array(self, start, end, n, dt=0, timeout=10):
        stime = rospy.Time.now()
        dx   = (end[0]-start[0])/n
        dy   = (end[1]-start[1])/n
        dyaw = (end[2]-start[2])/n
        for i in range(n+1):
            if (rospy.Time.now()-stime).to_sec() > timeout:
                print "timeout"
                return
            self.ik(start[0]+i*dx, start[1]+i*dy, start[2]+i*dyaw)
            rospy.sleep(rospy.Duration(dt))
    
    def ik_target(self, target, n, dt=0, timeout=10):
        target[2]=self.joint_sanitize(target[2])
        print "ik target: %lf %lf %lf" % (target[0], target[1], target[2])
        self.ik_array(self.fk(self.joint_angles_now), target, n, dt, timeout)

    def manip_cb(self, msg):
        manip_from_root = tf2_geometry_msgs.do_transform_pose(msg, self.buf.lookup_transform('hydrus_xi/root', 'hydrus_xi/camera', rospy.Time()))
        manip_from_root.pose.position.x+=0.08
        pos_now = self.fk(self.joint_angles_now)
        print "pos now: %lf %lf %lf" % (pos_now[0], pos_now[1], pos_now[2])
        q=manip_from_root.pose.orientation
        diff = float((manip_from_root.pose.position.x - pos_now[0])**2 + (manip_from_root.pose.position.y - pos_now[1])**2)**0.5
        self.ik_target([manip_from_root.pose.position.x, manip_from_root.pose.position.y, self.joint_sanitize(tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2])], 1+int(diff*50), timeout=0.3)

def sendFFWrench(pub, fx, fy, tz):
    pub.publish(Vector3(x=fx, y=fy, z=tz))

if __name__ == '__main__':
    rospy.init_node("interactive_test")
    pub = rospy.Publisher("/hydrus_xi/ff_wrench", Vector3, queue_size=10)
    rospy.sleep(rospy.Duration(5.0))
    hyd = HydrusCommander()
    
    if hyd.test_mode != 'F':
        rospy.sleep(rospy.Duration(1.0))
        hyd.arm_and_takeoff()
        rospy.sleep(rospy.Duration(15.0))
        hyd.change_yaw(1.57)
        rospy.sleep(rospy.Duration(2.0))
    
    if hyd.test_mode == 'J':
        hyd.ik_target([-0.2, 0.5, np.pi+0.5],100,0)
        rospy.sleep(rospy.Duration(2.0))
        hyd.change_yaw(1)
        rospy.sleep(rospy.Duration(2.0))
        hyd.move_to(0.2,0.2)
    elif hyd.test_mode == 'L':
        hyd.joint_publish([1.4, 1.57, 0])
        rospy.sleep(rospy.Duration(1.0))
        hyd.joint_publish([1.57, 0, 0])
        rospy.sleep(rospy.Duration(3.0))
        hyd.move_to(0,0.4)
    elif hyd.test_mode == 'U':
        hyd.joint_publish([1.4, 1.57, 0])
        rospy.sleep(rospy.Duration(1.0))
        hyd.joint_publish([1.57, 0, 1.57])
        rospy.sleep(rospy.Duration(3.0))
        hyd.move_to(0.2,0.4)
    elif hyd.test_mode == 'DOOR':
        hyd.joint_publish([1.4, 1.57, 0])
        rospy.sleep(rospy.Duration(1.0))
        #hyd.ik(-0.1,1.1,-0.2)
        hyd.joint_publish([0.773, 1.378, 0.793])
        rospy.sleep(rospy.Duration(3.0))
        hyd.move_to(0.5,0.5)
        rospy.sleep(rospy.Duration(1.0))
        hyd.change_yaw(2.36)
        rospy.sleep(rospy.Duration(3.0))
        hyd.move_to(0.2,-0.1)
    
    embed()
