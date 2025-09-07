#!/usr/bin/env python3
"""
predictive_haptics_node.py

ROS node that generates a feedback wrench for a haptic device by:
 - estimating operator impedance online (RLS)
 - predicting short-term operator response to nominal feedback wrench
 - estimating predicted energy increase
 - applying TDPA (PO/PC) + Energy Tank interventions if needed
 - publishing safe, clipped WrenchStamped for operator device

Topics:
 - Sub: /cfs/data            geometry_msgs/WrenchStamped   (robot end-effector measured wrench)
 - Sub: /me6_robot/joint_states sensor_msgs/JointState      (robot joint states)
 - Sub: /twin_hammer/mocap/pose geometry_msgs/PoseStamped   (operator device pose)
 - Pub: /twin_hammer/haptics_wrench geometry_msgs/WrenchStamped

Author: ChatGPT (adapted for your system)
"""

import rospy
import numpy as np
import threading
import copy
import time
from geometry_msgs.msg import WrenchStamped, PoseStamped, Wrench, Vector3
from sensor_msgs.msg import JointState
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tft

# -------------------------
# Utility helper functions
# -------------------------
def quat_to_rpy(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

def pose_to_mat(pose):
    q = pose.orientation
    rpy = quat_to_rpy(q)
    t = np.array([pose.position.x, pose.position.y, pose.position.z])
    R = tft.euler_matrix(rpy[0], rpy[1], rpy[2])[:3, :3]
    return R, t

def wrench_msg_from_vec6(vec):
    w = Wrench()
    w.force.x, w.force.y, w.force.z = vec[0], vec[1], vec[2]
    w.torque.x, w.torque.y, w.torque.z = vec[3], vec[4], vec[5]
    return w

def vec6_from_wrench_msg(wmsg):
    return np.array([wmsg.force.x, wmsg.force.y, wmsg.force.z,
                     wmsg.torque.x, wmsg.torque.y, wmsg.torque.z], dtype=float)

def twist_from_pose_diff(p_cur, p_prev, dt):
    # p_cur/p_prev: geometry_msgs/PoseStamped
    # returns 6x1 vector: [vx, vy, vz, wx, wy, wz] in the current frame (approx)
    if dt <= 0:
        return np.zeros(6)
    # linear velocity (world frame)
    dx = np.array([p_cur.position.x - p_prev.position.x,
                   p_cur.position.y - p_prev.position.y,
                   p_cur.position.z - p_prev.position.z]) / dt
    # angular velocity approximate: use quaternion -> rotation vector
    q1 = [p_prev.orientation.x, p_prev.orientation.y, p_prev.orientation.z, p_prev.orientation.w]
    q2 = [p_cur.orientation.x, p_cur.orientation.y, p_cur.orientation.z, p_cur.orientation.w]
    # compute delta quaternion qd ~ q2 * q1^{-1}
    qd = tft.quaternion_multiply(q2, tft.quaternion_inverse(q1))
    # convert small-angle quaternion to axis-angle
    angle = 2.0 * np.arccos(np.clip(qd[3], -1.0, 1.0))
    if abs(angle) < 1e-6:
        omega = np.zeros(3)
    else:
        axis = np.array([qd[0], qd[1], qd[2]]) / np.sin(angle / 2.0)
        omega = axis * (angle / dt)
    return np.concatenate((dx, omega))

def transform_wrench_slave_to_master(Ws_vec, R_s_to_m, p_m_to_s_in_m):
    """
    Ws_vec: 6-vector [Fx,Fy,Fz,Tx,Ty,Tz] in slave frame S
    R_s_to_m: 3x3 rotation matrix that rotates vectors from slave frame to master frame
    p_m_to_s_in_m: vector from master origin to slave origin expressed in master frame
    Returns Wm_vec in master frame.
    Formula:
      F_m = R * F_s
      T_m = R * T_s + p x F_m
    """
    F_s = Ws_vec[:3]
    T_s = Ws_vec[3:]
    F_m = R_s_to_m.dot(F_s)
    T_m = R_s_to_m.dot(T_s) + np.cross(p_m_to_s_in_m, F_m)
    return np.concatenate((F_m, T_m))

# -------------------------
# RLS estimator (per axis)
# -------------------------
class RLSLinear:
    """
    Recursive least squares for model:
      y = theta^T * phi   (phi is column vector)
    We keep theta and covariance P.
    """
    def __init__(self, n=3, lam=0.995, delta=1e3):
        self.n = n
        self.lam = lam  # forgetting factor
        self.theta = np.zeros(n)
        self.P = np.eye(n) * delta

    def update(self, phi, y):
        """
        phi: ndarray shape (n,)
        y: scalar
        """
        phi = phi.reshape((self.n,))
        Pphi = self.P.dot(phi)
        denom = self.lam + phi.dot(Pphi)
        k = Pphi / denom
        err = y - self.theta.dot(phi)
        self.theta = self.theta + k * err
        self.P = (self.P - np.outer(k, Pphi)) / self.lam

    def predict(self, phi):
        return self.theta.dot(phi)

# -------------------------
# Main Node
# -------------------------
class PredictiveHapticsNode:
    def __init__(self):
        rospy.init_node('predictive_haptics_node')

        # Parameters (minimized list)
        self.rate_hz = rospy.get_param('~rate', 200)                # main loop rate
        self.ee_frame = rospy.get_param('~ee_frame', 'ee_link')    # expected TF for robot ee (preferred)
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.master_frame = rospy.get_param('~master_frame', 'twin_hammer')  # operator device frame name
        self.k_force = rospy.get_param('~k_force', 0.9)             # scaling for force components
        self.k_torque = rospy.get_param('~k_torque', 0.9)          # scaling for torque components
        self.dt_pred = rospy.get_param('~dt_pred', 0.05)           # prediction horizon (s)
        self.rls_lambda = rospy.get_param('~rls_lambda', 0.995)    # forgetting factor for RLS
        self.rls_delta = rospy.get_param('~rls_delta', 1e3)        # initial cov
        self.min_B = rospy.get_param('~min_B', 0.1)                # avoid division by zero in prediction
        self.Etank_init = rospy.get_param('~Etank_init', 10.0)     # initial energy tank (J)
        self.Etank_max = rospy.get_param('~Etank_max', 100.0)      # max tank capacity
        self.PO_tolerance = rospy.get_param('~PO_tolerance', 0.0)  # allowed negative PO energy before PC kicks
        self.alpha_smoothing = rospy.get_param('~alpha_smoothing', 0.2)  # smoothing for scaling factor
        self.max_feedback_force = rospy.get_param('~max_feedback_force', 30.0) # N
        self.max_feedback_torque = rospy.get_param('~max_feedback_torque', 5.0) # Nm
        self.device_limits = np.array([self.max_feedback_force]*3 + [self.max_feedback_torque]*3)

        # Internal states / buffers
        self.lock = threading.Lock()
        self.latest_wrench_slave = None       # WrenchStamped
        self.latest_joint_states = None       # JointState
        self.latest_pose_master = None        # PoseStamped
        self.last_pose_master = None
        self.last_time_master = None
        self.last_pose_robot = None
        self.last_time_robot = None

        # TF for pose lookup
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # RLS estimators for 6 axes: each axis has [K, B, bias]
        self.rls = [RLSLinear(n=3, lam=self.rls_lambda, delta=self.rls_delta) for _ in range(6)]

        # Energy bookkeeping
        self.E_PO = 0.0
        self.E_tank = self.Etank_init

        # smoothing state
        self.prev_alpha = 1.0

        # Publishers / Subscribers
        self.pub_feedback = rospy.Publisher('/twin_hammer/haptics_wrench', WrenchStamped, queue_size=1)

        rospy.Subscriber('/cfs/data', WrenchStamped, self.cb_wrench_slave, queue_size=1)
        rospy.Subscriber('/me6_robot/joint_states', JointState, self.cb_joint_states, queue_size=1)
        rospy.Subscriber('/twin_hammer/mocap/pose', PoseStamped, self.cb_pose_master, queue_size=1)

        rospy.loginfo("predictive_haptics_node initialized. rate=%d Hz, ee_frame=%s", self.rate_hz, self.ee_frame)

    # -------------------------
    # Callbacks
    # -------------------------
    def cb_wrench_slave(self, msg):
        with self.lock:
            self.latest_wrench_slave = msg
            # for robot pose velocity (if using TF below)
            try:
                # attempt to fetch ee pose via TF (best)
                trans = self.tf_buffer.lookup_transform(self.world_frame, self.ee_frame, rospy.Time(0), rospy.Duration(0.01))
                pose = PoseStamped()
                pose.header = trans.header
                pose.pose.position.x = trans.transform.translation.x
                pose.pose.position.y = trans.transform.translation.y
                pose.pose.position.z = trans.transform.translation.z
                pose.pose.orientation = trans.transform.rotation
                now = trans.header.stamp.to_sec()
                if self.last_pose_robot is not None:
                    self.last_time_robot = getattr(self, 'last_time_robot', now)
                    self.last_pose_robot = getattr(self, 'last_pose_robot', pose)
                else:
                    self.last_pose_robot = pose
                    self.last_time_robot = now
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                # no TF; user is expected to provide FK via service if needed
                pass

    def cb_joint_states(self, msg):
        with self.lock:
            self.latest_joint_states = msg

    def cb_pose_master(self, msg):
        with self.lock:
            # track last and current for velocity estimation
            if self.latest_pose_master is None:
                self.latest_pose_master = msg
                self.last_pose_master = msg
                self.last_time_master = msg.header.stamp.to_sec()
            else:
                self.last_pose_master = copy.deepcopy(self.latest_pose_master)
                self.latest_pose_master = msg
                self.last_time_master = getattr(self, 'last_time_master', msg.header.stamp.to_sec())
                self.last_time_master = msg.header.stamp.to_sec()

    # -------------------------
    # Helper: get robot end-effector pose (world frame)
    # -------------------------
    def get_robot_pose(self):
        # Primary: TF
        try:
            trans = self.tf_buffer.lookup_transform(self.world_frame, self.ee_frame, rospy.Time(0), rospy.Duration(0.01))
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            return pose
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            # fallback: if joint_states & FK service available, user can implement; here we warn
            rospy.logwarn_throttle(10.0, "TF for ee_frame not available. Ensure tf or provide FK service.")
            return None

    # -------------------------
    # Main loop
    # -------------------------
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        prev_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            with self.lock:
                now = rospy.Time.now().to_sec()
                dt = now - prev_time if now - prev_time > 0 else 1.0 / self.rate_hz
                prev_time = now

                # copy locals
                wrench_slave_msg = copy.deepcopy(self.latest_wrench_slave)
                pose_master_msg = copy.deepcopy(self.latest_pose_master)
            # if essential data missing, skip
            if wrench_slave_msg is None or pose_master_msg is None:
                rate.sleep()
                continue

            # get robot pose
            pose_robot_msg = self.get_robot_pose()

            # 1) Transform W_measured (slave) into master/operator frame
            # compute rotation R_s_to_m and p_m_to_s_in_m (master frame)
            try:
                # world -> slave and world -> master rotation matrices
                R_robot, t_robot = pose_to_mat(pose_robot_msg.pose) if pose_robot_msg is not None else (np.eye(3), np.zeros(3))
                R_master, t_master = pose_to_mat(pose_master_msg.pose)
            except Exception as e:
                rospy.logwarn("Failed to compute poses: %s", str(e))
                rate.sleep()
                continue

            # R_s_to_m = R_master^T * R_robot (rotates slave vectors to master frame)
            R_s_to_m = R_master.T.dot(R_robot)
            # vector from master origin to slave origin in master frame:
            p_world = t_robot - t_master
            p_m_to_s_in_m = R_master.T.dot(p_world)

            W_meas_vec = vec6_from_wrench_msg(wrench_slave_msg.wrench)
            # transform to master frame
            W_meas_in_master = transform_wrench_slave_to_master(W_meas_vec, R_s_to_m, p_m_to_s_in_m)

            # 2) Nominal scaling -> proposed feedback wrench
            k_vec = np.array([self.k_force, self.k_force, self.k_force,
                              self.k_torque, self.k_torque, self.k_torque])
            W_nom = k_vec * W_meas_in_master  # candidate feedback if no intervention

            # 3) Estimate operator impedance parameters (RLS).
            # We use phi = [x_i, v_i, 1] per axis, y = commanded_force_prev (we will use previous published command as proxy)
            # For first step, if no prev command exists, skip update.
            # maintain last_published_command in self (we'll store)
            if not hasattr(self, 'last_published_cmd'):
                # initialize with zero wrench
                self.last_published_cmd = np.zeros(6)

            # compute operator pose/velocity
            with self.lock:
                pm_cur = copy.deepcopy(self.latest_pose_master)
                pm_prev = copy.deepcopy(self.last_pose_master)
                t_prev = getattr(self, 'last_time_master', None)
            if pm_prev is None or pm_cur is None or t_prev is None:
                v_op = np.zeros(6)
                x_op = np.zeros(6)
            else:
                dt_pose = max(1e-6, (pose_master_msg.header.stamp.to_sec() - t_prev))
                v_op = twist_from_pose_diff(pm_cur.pose, pm_prev.pose, dt_pose)
                x_op = np.array([pm_cur.pose.position.x, pm_cur.pose.position.y, pm_cur.pose.position.z] +
                                list(quat_to_rpy(pm_cur.pose.orientation)))
            # Update RLS per axis with last_published_cmd as the applied wrench (proxy for human-applied force)
            for i in range(6):
                phi = np.array([x_op[i], v_op[i], 1.0])
                y = self.last_published_cmd[i]  # proxy of force applied to operator in previous step
                try:
                    self.rls[i].update(phi, y)
                except Exception as e:
                    rospy.logwarn_throttle(30, "RLS update exception: %s", str(e))

            # Extract estimated parameters
            theta = np.array([self.rls[i].theta for i in range(6)])  # 6 x 3
            # theta[:,0] = K_est, theta[:,1] = B_est, theta[:,2] = bias (-K*x_des)
            K_est = theta[:, 0]
            B_est = theta[:, 1]
            bias_est = theta[:, 2]

            # prevent tiny B estimates
            B_est = np.where(np.abs(B_est) < self.min_B, np.sign(B_est) * self.min_B + 1e-6, B_est)

            # 4) Predict short-term operator response to W_nom (per-axis)
            # For prediction choose applied wrench = W_nom
            # Predict translational velocity: v_pred = (F_applied - K*x - bias) / B
            # For rotation similarly with torques and angular velocities (small-angle)
            F_applied = W_nom  # 6-vector
            v_pred = np.zeros(6)
            for i in range(6):
                v_pred[i] = (F_applied[i] - K_est[i] * x_op[i] - bias_est[i]) / B_est[i]

            # 5) Predict predicted output power and input power
            # P_out_pred = W_nom^T * v_pred
            P_out_pred = float(np.dot(W_nom, v_pred))

            # P_in_pred approximate: use measured W_meas and robot end-effector velocity (numerical diff)
            # compute robot ee twist
            if getattr(self, 'last_pose_robot', None) is not None and pose_robot_msg is not None:
                # estimate robot velocity from last_pose_robot/time
                # if last_time_robot exists
                now_t = pose_robot_msg.header.stamp.to_sec()
                prev_t = getattr(self, 'last_time_robot', now_t)
                dt_robot = max(1e-6, now_t - prev_t)
                v_robot = twist_from_pose_diff(pose_robot_msg.pose, getattr(self, 'last_pose_robot').pose, dt_robot)
                # update last_pose_robot/time
                self.last_pose_robot = pose_robot_msg
                self.last_time_robot = now_t
                P_in_pred = float(np.dot(W_meas_in_master, v_robot))
            else:
                # if no robot twist, approximate using zero (conservative)
                P_in_pred = 0.0

            # Predicted delta energy over dt_pred
            DeltaE_pred = (P_out_pred - P_in_pred) * self.dt_pred

            # 6) Energy tank & predictive intervention
            # If predicted positive energy and tank insufficient -> reduce W_nom scaling alpha
            alpha = 1.0
            if DeltaE_pred > 1e-6:
                if self.E_tank <= 0.0:
                    alpha = 0.0
                else:
                    alpha = min(1.0, max(0.0, self.E_tank / (DeltaE_pred + 1e-9)))
                # smooth alpha to avoid sudden jumps
                alpha = self.prev_alpha * (1.0 - self.alpha_smoothing) + alpha * self.alpha_smoothing

            # apply alpha scaling
            W_candidate = alpha * W_nom

            # 7) PO update using candidate output (master port)
            # measure instantaneous power P_inst = W_candidate^T * v_op
            P_inst = float(np.dot(W_candidate, v_op))
            self.E_PO += P_inst * dt

            # 8) If PO shows negative energy beyond tolerance, apply PC (damping injection)
            W_out = W_candidate.copy()
            if self.E_PO < -self.PO_tolerance:
                # compute required absorb power to bring E_PO to zero next step
                required_absorb_power = (-self.E_PO) / max(dt, 1e-6)  # W
                vnorm2 = float(np.dot(v_op, v_op)) + 1e-9
                d_needed = required_absorb_power / vnorm2
                # clip d to reasonable range
                d = float(np.clip(d_needed, 0.0, 1000.0))
                W_out = W_out - d * v_op
                # update E_PO using new power
                P_after = float(np.dot(W_out, v_op))
                self.E_PO = self.E_PO + (P_after - P_inst) * dt  # or recompute absolute
                # charge tank with absorbed energy (positive)
                absorbed = (P_inst - P_after) * dt
                if absorbed > 0:
                    self.E_tank = min(self.E_tank + absorbed, self.Etank_max)

            # 9) Apply additional tank-based consumption if DeltaE_pred positive
            if DeltaE_pred > 1e-6:
                # consume tank proportional to predicted energy
                consume = min(self.E_tank, max(0.0, (1.0 - alpha) * DeltaE_pred))
                self.E_tank = max(0.0, self.E_tank - consume)

            # 10) Clip output wrench to device limits and publish
            # ensure per-axis clipping
            W_out_clipped = np.clip(W_out, -self.device_limits, self.device_limits)
            out_msg = WrenchStamped()
            out_msg.header.stamp = rospy.Time.now()
            out_msg.header.frame_id = self.master_frame
            wmsg = wrench_msg_from_vec6(W_out_clipped)
            out_msg.wrench = wmsg
            self.pub_feedback.publish(out_msg)

            # store last published for RLS next iteration
            self.last_published_cmd = W_out_clipped.copy()
            self.prev_alpha = alpha

            # debug logging occasionally
            rospy.logdebug_throttle(2.0, "P_out_pred=%.4f P_in_pred=%.4f DeltaE_pred=%.4f E_tank=%.3f E_PO=%.3f alpha=%.3f",
                                    P_out_pred, P_in_pred, DeltaE_pred, self.E_tank, self.E_PO, alpha)

            rate.sleep()


if __name__ == '__main__':
    node = PredictiveHapticsNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
