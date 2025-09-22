#!/usr/bin/env python3
"""
haptics_feedback_node.py

- Subscriber:
  - /cfs/data            : geometry_msgs/WrenchStamped  (robot measured wrench)
  - /twin_hammer/mocap/pose : geometry_msgs/PoseStamped  (mocap pose)
  - /twin_hammer/Imu     : spinal/Imu                    (custom IMU message described by user)

- Publisher:
  - /twin_hammer/haptics_wrench : geometry_msgs/WrenchStamped

Implements:
 - per-axis RLS (theta = [M, B, K]) for 6 axes (3 trans + 3 rot)
 - mocap + IMU fusion for velocity/acceleration (simple complementary / EMA filters)
 - prediction of v_hand at next step
 - estimation of w_hand
 - energy tank accounting and scaling/damping per provided algorithm
 - runs at 100 Hz
"""

import rospy
import numpy as np
import yaml
import os
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_msgs.msg import Header

# custom IMU message as given by user
# rosmsg show spinal/Imu:
# time stamp
# float32[3] acc_data
# float32[3] gyro_data
# float32[3] mag_data
# float32[3] angles
from spinal.msg import Imu  # assume package exists in workspace

# Helpers: quaternion -> euler if needed (but we use IMU angles for orientation)
from tf.transformations import euler_from_quaternion

# ---------- Utility functions ----------
def clip(x, a, b):
    return max(a, min(b, x))

def safe_div(a, b, eps=1e-9):
    return a / b if abs(b) > eps else 0.0

# ---------- Node implementation ----------
class HapticsFeedbackNode:
    def __init__(self, param_path=None):
        rospy.init_node('haptics_feedback_node', anonymous=False)

        # Load params from YAML file in same directory if provided
        if param_path is None:
            script_dir = os.path.dirname(os.path.realpath(__file__))
            param_path = os.path.join(script_dir, 'operator_predictive_params.yaml')
        rospy.loginfo(f"Loading params from {param_path}")
        if os.path.exists(param_path):
            with open(param_path, 'r') as f:
                self.params = yaml.safe_load(f)
        else:
            rospy.logwarn("Param file not found; using embedded defaults")
            self.params = {}

        # set parameters with defaults
        p = self.params
        self.dt = p.get('dt', 0.01)  # 100 Hz default
        self.E_tank = p.get('Etank_0', 0.0)
        self.E_tank_max = p.get('Etank_max', 5.0)
        self.kscale_trans = p.get('kscale_trans', 0.4)
        self.kscale_rot = p.get('kscale_rot', 0.1)
        self.lambda_r = p.get('lambda_r', 0.995)
        Q0_diag = p.get('Q0_diag', [100.0, 100.0, 100.0])
        self.Q0 = np.diag(Q0_diag)
        M0_trans = p.get('M0_trans', 1.0)
        M0_rot = p.get('M0_rot', 1e-3)
        B0_trans = p.get('B0_trans', 1.0)
        B0_rot = p.get('B0_rot', 1e-2)
        K0_trans = p.get('K0_trans', 100.0)
        K0_rot = p.get('K0_rot', 1.0)
        # other
        self.P_thre = p.get('P_thre', 1e-3)
        self.alpha_const = p.get('alpha_const', 0.1)
        self.eps_v = p.get('eps_v', 1e-6)
        self.dconst = p.get('dconst', 1.0)
        self.dmax = p.get('dmax', 1000.0)
        self.ema_vel = p.get('ema_vel', 0.8)  # EMA for velocity
        self.ema_acc = p.get('ema_acc', 0.8)  # EMA for acceleration

        # RLS state: per-axis (6 axes) theta (3,) and Q (3x3)
        # axis order: 0..2 linear x,y,z ; 3..5 angular x,y,z
        self.theta = np.zeros((6, 3))
        for a in range(3):
            self.theta[a, :] = np.array([M0_trans, B0_trans, K0_trans])
        for a in range(3,6):
            self.theta[a, :] = np.array([M0_rot, B0_rot, K0_rot])
        self.Q = np.stack([self.Q0.copy() for _ in range(6)], axis=0)

        # data buffers
        self.w_robot_meas = np.zeros(6)   # incoming robot wrench (force[0:3], torque[0:3])
        self.mocap_pos = np.zeros(3)
        self.mocap_orient = np.zeros(3)   # use euler if needed
        self.imu_acc = np.zeros(3)
        self.imu_gyro = np.zeros(3)
        self.imu_angles = np.zeros(3)

        # filtered kinematics
        self.v_hand = np.zeros(6)
        self.v_hand_filtered = np.zeros(6)
        self.x_hand = np.zeros(6)
        self.x_hand_prev = np.zeros(6)
        self.xdot_prev = np.zeros(6)
        self.xddot_filtered = np.zeros(6)

        # reference origin x_ref (set at start)
        self.x_ref = np.zeros(6)
        self.is_xref_set = False

        # last time stamps
        self.last_mocap_time = None
        self.last_imu_time = None
        self.last_loop_time = rospy.Time.now()

        # Publisher
        self.pub = rospy.Publisher('/twin_hammer/haptics_wrench', WrenchStamped, queue_size=1)

        # Subscribers
        rospy.Subscriber('/cfs/data', WrenchStamped, self.robot_wrench_cb, queue_size=1)
        rospy.Subscriber('/twin_hammer/mocap/pose', PoseStamped, self.mocap_cb, queue_size=1)
        rospy.Subscriber('/twin_hammer/Imu', Imu, self.imu_cb, queue_size=1)

        rospy.loginfo("HapticsFeedbackNode initialized.")

    # ---------- callbacks ----------
    def robot_wrench_cb(self, msg: WrenchStamped):
        # Map geometry_msgs/Wrench -> 6-vector
        f = msg.wrench.force
        t = msg.wrench.torque
        self.w_robot_meas = np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def mocap_cb(self, msg: PoseStamped):
        # Use position from PoseStamped
        pos = msg.pose.position
        quat = msg.pose.orientation
        self.mocap_pos = np.array([pos.x, pos.y, pos.z])
        # convert quaternion to euler (radians)
        euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.mocap_orient = np.array(euler)  # roll,pitch,yaw

        stamp = msg.header.stamp
        if not self.is_xref_set:
            # set reference at first mocap pose
            self.x_ref[0:3] = self.mocap_pos.copy()
            self.x_ref[3:6] = self.mocap_orient.copy()
            self.is_xref_set = True

        # update x_hand (6-d)
        self.x_hand[0:3] = self.mocap_pos
        self.x_hand[3:6] = self.mocap_orient

        self.last_mocap_time = stamp

    def imu_cb(self, msg: Imu):
        # IMU message fields described by user
        self.imu_acc = np.array(msg.acc_data, dtype=float)    # linear acc (3,)
        self.imu_gyro = np.array(msg.gyro_data, dtype=float)  # angular velocity (rad/s)
        self.imu_angles = np.array(msg.angles, dtype=float)   # angles (rad)
        stamp = getattr(msg, 'stamp', rospy.Time.now())
        self.last_imu_time = stamp

        # also update orientation from imu angles if mocap missing
        if not self.is_xref_set:
            self.x_ref[3:6] = self.imu_angles.copy()
            # do not set linear ref here (mocap is preferred)

    # ---------- RLS update for one axis ----------
    def rls_update_axis(self, axis, phi, y):
        """
        axis: 0..5
        phi: 3-vector [xdd, xdot, x - xref]  (column)
        y: scalar (previous feedback force applied to operator for that axis)
        """
        # ensure shapes
        phi = phi.reshape((3,1))  # column
        Q = self.Q[axis]
        lam = self.lambda_r

        # Qi update: Qi = 1/lambda (Qi- Qi phi phi^T Qi / (lambda + phi^T Qi phi))
        denom = lam + float(phi.T @ Q @ phi)
        numer = Q @ phi @ phi.T @ Q
        Q_new = (1.0 / lam) * (Q - numer / denom)
        self.Q[axis] = Q_new

        # theta update: theta = theta + Qi phi (y - phi^T theta)
        theta_prev = self.theta[axis].reshape((3,1))
        inov = y - float(phi.T @ theta_prev)
        theta_new = theta_prev + Q_new @ phi * inov
        self.theta[axis] = theta_new.flatten()

    def _compute_dadd_block(self, pj_block, v_block, Preq_block):
        """
        pj_block: array of pj for the block (pj = max(0, wcand_j * vj))
        v_block: array of vj (vhand_next for the block)
        Preq_block: scalar (required dissipated power for this block)
        returns: array of dj for the block
        """
        pj_sum = np.sum(pj_block)
        if pj_sum < 1e-12 or Preq_block <= 0:
            # no informative power distribution -> fallback to constant damping
            return np.array([self.dconst] * len(v_block))

        # normalized weights qhat_j
        qhat = pj_block / (pj_sum + 1e-12)

        # denom = sum_j qhat_j * vj^2/(vj^2 + eps)
        denom = np.sum(qhat * (v_block**2) / (v_block**2 + self.eps_v))
        if denom < 1e-12:
            return np.array([self.dconst] * len(v_block))

        kappa = Preq_block / denom

        # dj = kappa * qhat_j / (vj^2 + eps)
        djs = kappa * qhat / (v_block**2 + self.eps_v)
        # cap to dmax and ensure non-negative
        djs = np.minimum(np.maximum(djs, 0.0), self.dmax)
        return djs


    # ---------- main loop step ----------
    def step(self):
        now = rospy.Time.now()
        # compute dt from requested loop rate, but be robust
        current_dt = (now - self.last_loop_time).to_sec()
        if current_dt <= 0 or current_dt > 1.0:
            current_dt = self.dt
        self.last_loop_time = now

        # --- estimate velocities and accelerations (fusion) ---
        # For linear: use finite difference on mocap for velocity and IMU for accel; apply EMA filters
        # For angular: use IMU.gyro for angular velocity; angular accel via diff of gyro

        # compute xdot (raw) from mocap if available
        xdot_raw = np.zeros(6)
        if self.last_mocap_time is not None:
            xdot_raw[0:3] = (self.x_hand[0:3] - self.x_hand_prev[0:3]) / current_dt     # linear
            # angular: use imu angles if available, else mocap euler
            if np.any(self.imu_gyro):
                xdot_raw[3:6] = self.imu_gyro  # angular velocity from IMU (preferred)
            else:
                xdot_raw[3:6] = (self.x_hand[3:6] - self.x_hand_prev[3:6]) / current_dt
        else:
            xdot_raw = self.v_hand.copy()
        self.v_hand_filtered = self.ema_vel * self.v_hand_filtered + (1.0 - self.ema_vel) * xdot_raw
        self.v_hand = self.v_hand_filtered.copy()

        # acceleration: use IMU linear acc for 0:3; angular accel via diff of gyro
        xdd_raw = np.zeros(6)
        xdd_raw[0:3] = self.imu_acc     # linear acceleration: imu_acc (assumed already in m/s^2)
        xdd_raw[3:6] = (self.imu_gyro - self.xdot_prev[3:6]) / max(current_dt, 1e-6)        # angular acceleration: derivative of gyro
        self.xddot_filtered = self.ema_acc * self.xddot_filtered + (1.0 - self.ema_acc) * xdd_raw
        xdd = self.xddot_filtered.copy()

        # Save prevs
        self.x_hand_prev = self.x_hand.copy()
        self.xdot_prev = self.v_hand.copy()

        # --- RLS: update per-axis using phi = [xdd, xdot, x - xref], y = previous applied feedback wrench command (we don't have measured hand wrench) ---
        # For yi we use the previous command value that was sent to operator. 
        # To approximate this, we will use last published feedback (we keep last_feedback_cmd)
        if not hasattr(self, 'last_feedback_cmd'):
            self.last_feedback_cmd = np.zeros(6)
        for axis in range(6):
            phi = np.array([xdd[axis], self.v_hand[axis], self.x_hand[axis] - self.x_ref[axis]])
            y = float(self.last_feedback_cmd[axis])
            try:
                self.rls_update_axis(axis, phi, y)
            except Exception as e:
                rospy.logwarn_throttle(5.0, f"RLS update error axis {axis}: {e}")

        # --- Predict v_hand at next step using wcand (scaled measured robot wrench) ---
        # wcand = kscale ⊙ w_robot_meas
        wcand = np.zeros(6)
        wcand[0:3] = self.kscale_trans * self.w_robot_meas[0:3]
        wcand[3:6] = self.kscale_rot * self.w_robot_meas[3:6]

        # compute predicted acceleration per axis using current theta (M,B,K)
        vhand_next = np.zeros(6)
        for axis in range(6):
            M, B, K = self.theta[axis]
            # avoid zero M
            if abs(M) < 1e-6:
                M = 1e-6
            # xdd_pred = (wcand_j - B xdot - K x)/M
            x = self.x_hand[axis]
            xdot = self.v_hand[axis]
            wj = wcand[axis]
            xdd_pred = (wj - B * xdot - K * x) / M
            vhand_next[axis] = xdot + xdd_pred * current_dt

        # --- Estimate hand wrench at current step using measured kinematics and theta (Eq. 10) ---
        whand_est = np.zeros(6)
        for axis in range(6):
            M, B, K = self.theta[axis]
            whand_est[axis] = M * xdd[axis] + B * self.v_hand[axis] + K * (self.x_hand[axis] - self.x_ref[axis])

        # --- compute Pin (power into device from operator at current step) ---
        # Pin = w_hand(i)^T v_hand(i)
        Pin = float(np.dot(whand_est, self.v_hand))

        # --- compute predicted energy flow for next step ---
        # dEpred = (wcand^T vhand_next - Pin) * dt
        P_cand_vnext = float(np.dot(wcand, vhand_next))
        dEpred = (P_cand_vnext - Pin) * current_dt

        E_tank_next = self.E_tank - dEpred

        # default: no scaling (alpha = 1), no added damping
        alpha = 1.0
        use_damping = False
        Dadd = np.zeros(6)

        # --- If E_tank_next < 0 => need scaling alpha in [0,1) so E_tank_next becomes 0 ---
        if E_tank_next < 0:
            denom = (P_cand_vnext)
            if abs(denom) < self.P_thre:
                alpha = self.alpha_const
            else:
                alpha = safe_div(self.E_tank / current_dt + Pin, P_cand_vnext)
            alpha = clip(alpha, 0.0, 1.0)
            E_tank_next = 0.0  # after scaling we set to 0
            rospy.loginfo_throttle(5.0, f"Energy underflow: applying scaling alpha={alpha:.3f}")

        # --- If E_tank_next > E_tank_max => add damping to dissipate energy ---
        if E_tank_next > self.E_tank_max:
            # compute required dissipated power Preq per eq (23)
            Preq = (self.E_tank_max - self.E_tank) / current_dt + P_cand_vnext - Pin
            # split into trans and rot blocks
            # compute per-axis positive power contributions pj = max(0, wcand_j * vj)

            if Preq <= 0:
                use_damping = False
                E_tank_next = self.E_tank_max
                rospy.loginfo_throttle(5.0, "Energy overflow detected but Preq <= 0 -> no damping applied.")
            else:
                p = np.maximum(0.0, wcand * vhand_next)
                sum_p = np.sum(p) + 1e-12
                sum_p_trans = np.sum(p[0:3]) + 1e-12
                sum_p_rot = np.sum(p[3:6]) + 1e-12

                if sum_p > 1e-12:
                    Preq_trans = Preq * (sum_p_trans / sum_p)
                else:
                    Preq_trans = Preq * 0.5
                Preq_rot = Preq - Preq_trans
            
            qhat = np.zeros(6)
            denom_trans = 0
            denom_rot = 0
            for j in range(len(qhat)):
                if j < 3:
                    qhat[j] = p[j] / sum_p_trans
                else:
                    qhat[j] = p[j] / sum_p_rot
                denom_j = qhat * vhand_next[j] * vhand_next[j] / (vhand_next[j] * vhand_next[j] + self.eps_v)
                if j < 3:
                    denom_trans += denom_j
                else:
                    denom_rot += denom_j
            kappa_trans = Preq_trans / denom_trans
            kappa_rot = Preq_rot / denom_rot
            for j in range(len(qhat)):
                if j < 3:
                    dj = kappa_trans * qhat[j]
                else:
                    dj = kappa_rot * qhat[j]
                dj = clip(dj, 0.0, self.dmax)
                Dadd[j,j] = dj

            use_damping = True
            # after damping selection, set E_tank_next to E_tank_max (we dissipate to that)
            E_tank_next = self.E_tank_max
            rospy.loginfo_throttle(5.0, f"Energy overflow: adding damping, Dadd diag approx = {Dadd}")

        # --- compute final feedback wrench ---
        if use_damping:
            wfeedback = wcand - (Dadd * vhand_next)
        else:
            wfeedback = alpha * wcand

        # store last feedback command (for RLS y)
        self.last_feedback_cmd = wfeedback.copy()

        # update E_tank
        self.E_tank = E_tank_next

        # publish as WrenchStamped
        msg = WrenchStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.wrench.force.x = float(wfeedback[0])
        msg.wrench.force.y = float(wfeedback[1])
        msg.wrench.force.z = float(wfeedback[2])
        msg.wrench.torque.x = float(wfeedback[3])
        msg.wrench.torque.y = float(wfeedback[4])
        msg.wrench.torque.z = float(wfeedback[5])
        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(1.0 / self.dt)  # intended loop frequency (e.g., dt=0.01 -> 100Hz)
        rospy.loginfo("HapticsFeedbackNode running main loop.")
        while not rospy.is_shutdown():
            try:
                self.step()
            except Exception as e:
                rospy.logerr_throttle(5.0, f"Exception in step(): {e}")
            rate.sleep()

if __name__ == "__main__":
    node = HapticsFeedbackNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
