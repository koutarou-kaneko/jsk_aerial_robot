#!/usr/bin/env python3
"""
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
from std_msgs.msg import Header, Float32MultiArray, Int32
from spinal.msg import Imu
from tf.transformations import euler_from_quaternion
from scipy.signal import medfilt
from collections import deque

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
        self.est_medfilt_window = p.get('est_medfilt_window', 3)

        # RLS state: per-axis (6 axes) theta (3,) and Q (3x3)
        # axis order: 0..2 linear x,y,z ; 3..5 angular x,y,z
        self.theta = np.zeros((6, 3))
        for a in range(3):
            self.theta[a, :] = np.array([M0_trans, B0_trans, K0_trans])
        for a in range(3,6):
            self.theta[a, :] = np.array([M0_rot, B0_rot, K0_rot])
        self.Q = np.stack([self.Q0.copy() for _ in range(6)], axis=0)

        self.cfs_connection_state = False
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
        self.vhand_next_buffer = deque(maxlen=self.est_medfilt_window)
        self.whand_est_buffer = deque(maxlen=self.est_medfilt_window)
        self.P_cand_vnext_buffer = deque(maxlen=self.est_medfilt_window)

        # reference origin x_ref (set at start)
        self.x_ref = np.zeros(6)
        self.is_xref_set = False

        # last time stamps
        self.last_mocap_time = None
        self.last_imu_time = None
        self.last_loop_time = rospy.Time.now()

        # Publisher
        self.wrench_pub = rospy.Publisher('/twin_hammer/haptics_wrench', WrenchStamped, queue_size=1)
        self.debug_handnext_pub = rospy.Publisher('/debug/handnext', Float32MultiArray, queue_size=1)
        self.debug_whand_est_pub = rospy.Publisher('/debug/whand_est', Float32MultiArray, queue_size=1)
        self.debug_energy_pub = rospy.Publisher('/debug/energy', Float32MultiArray, queue_size=1)
        self.debug_scale_pub = rospy.Publisher('/debug/scale', Float32MultiArray, queue_size=1)
        self.debug_dadd_pub = rospy.Publisher('/debug/dadd', Float32MultiArray, queue_size=1)
        self.debug_impedance_x_pub = rospy.Publisher('/debug/impedance/x', Float32MultiArray, queue_size=1)
        self.debug_impedance_y_pub = rospy.Publisher('/debug/impedance/y', Float32MultiArray, queue_size=1)
        self.debug_impedance_z_pub = rospy.Publisher('/debug/impedance/z', Float32MultiArray, queue_size=1)
        self.debug_impedance_roll_pub = rospy.Publisher('/debug/impedance/roll', Float32MultiArray, queue_size=1)
        self.debug_impedance_pitch_pub = rospy.Publisher('/debug/impedance/pitch', Float32MultiArray, queue_size=1)
        self.debug_impedance_yaw_pub = rospy.Publisher('/debug/impedance/yaw', Float32MultiArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('/cfs/data', WrenchStamped, self.robot_wrench_cb, queue_size=1)
        rospy.Subscriber('/twin_hammer/mocap/pose', PoseStamped, self.mocap_cb, queue_size=1)
        rospy.Subscriber('/twin_hammer/Imu', Imu, self.imu_cb, queue_size=1)
        rospy.Subscriber('/cfs/connection_state', Int32, self.cfs_connection_cb, queue_size=1)

        rospy.loginfo("HapticsFeedbackNode initialized.")

    # ---------- callbacks ----------
    def robot_wrench_cb(self, msg: WrenchStamped):
        f = msg.wrench.force
        t = msg.wrench.torque
        self.w_robot_meas = np.array([f.z, f.x, f.y, t.z, t.x, t.y])

    def mocap_cb(self, msg: PoseStamped):
        pos = msg.pose.position
        quat = msg.pose.orientation
        self.mocap_pos = np.array([pos.x, pos.y, pos.z])
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
        self.imu_acc = np.array(msg.acc_data, dtype=float)    # linear acc (3,)
        self.imu_gyro = np.array(msg.gyro_data, dtype=float)  # angular velocity (rad/s)
        self.imu_angles = np.array(msg.angles, dtype=float)   # angles (rad)
        stamp = getattr(msg, 'stamp', rospy.Time.now())
        self.last_imu_time = stamp

        # also update orientation from imu angles if mocap missing
        if not self.is_xref_set:
            self.x_ref[3:6] = self.imu_angles.copy()
            # do not set linear ref here (mocap is preferred)

    def cfs_connection_cb(self, msg: Int32):
        self.cfs_connection_state = msg.data

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
        denom = lam + (phi.T @ Q @ phi).item()
        numer = Q @ phi @ phi.T @ Q
        Q_new = (1.0 / lam) * (Q - numer / denom)
        self.Q[axis] = Q_new

        # theta update: theta = theta + Qi phi (y - phi^T theta)
        theta_prev = self.theta[axis].reshape((3,1))
        inov = y - (phi.T @ theta_prev).item()
        theta_new = theta_prev + Q_new @ phi * inov
        self.theta[axis] = theta_new.flatten()


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
        self.vhand_next_buffer.append(vhand_next.copy())
        if len(self.vhand_next_buffer) == self.est_medfilt_window:
            vhand_stacked = np.stack(self.vhand_next_buffer, axis=0)
            vhand_next = np.median(vhand_stacked, axis=0)
        vhand_next = medfilt(vhand_next, kernel_size=3)

        # --- Estimate hand wrench at current step using measured kinematics and theta (Eq. 10) ---
        whand_est = np.zeros(6)
        for axis in range(6):
            M, B, K = self.theta[axis]
            whand_est[axis] = M * xdd[axis] + B * self.v_hand[axis] + K * (self.x_hand[axis] - self.x_ref[axis])
        self.whand_est_buffer.append(whand_est.copy())
        # if len(self.whand_est_buffer) == self.est_medfilt_window:
        #     whand_stacked = np.stack(self.whand_est_buffer, axis=0)
        #     whand_est = np.median(whand_stacked, axis=0)
        # whand_est = medfilt(whand_est, kernel_size=3)

        # --- compute Pin (power into device from operator at current step) ---
        Pin = float(np.dot(whand_est, self.v_hand))

        # --- compute predicted energy flow for next step ---
        P_cand_vnext = float(np.dot(wcand, vhand_next))
        self.P_cand_vnext_buffer.append(P_cand_vnext)
        if len(self.P_cand_vnext_buffer) == self.est_medfilt_window:
            P_cand_vnext = np.median(self.P_cand_vnext_buffer)
        dEpred = (P_cand_vnext - Pin) * current_dt
        E_tank_next = self.E_tank - dEpred

        use_QP = False
        use_damping = False
        Dadd = np.zeros((6,6))

        if E_tank_next < 0:
            # scaling
            '''
            denom = (P_cand_vnext)
            if abs(denom) < self.P_thre:
                alpha = self.alpha_const
            else:
                alpha = safe_div(self.E_tank / current_dt + Pin, P_cand_vnext)
            alpha = clip(alpha, 0.0, 1.0)
            E_tank_next = 0.0  # after scaling we set to 0
            rospy.loginfo_throttle(5.0, f"Energy underflow: applying scaling alpha={alpha:.3f}")
            '''

            # QP
            rhs = self.E_tank / current_dt + Pin
            vTv = float(np.dot(vhand_next, vhand_next)) + 1e-9
            vTw_cand = float(np.dot(vhand_next, wcand))
            lam = max(0.0, (vTw_cand - rhs) / vTv)
            w_opt = wcand - lam * vhand_next
            E_tank_next = max(0.0, self.E_tank - current_dt * (np.dot(w_opt, vhand_next) - Pin))
            use_QP = True
            rospy.loginfo_throttle(1.0, f"Energy underflow -> minimal QP correction, λ={lam:.3e}, Etank_next={E_tank_next:.3f}")
            
        if E_tank_next > self.E_tank_max:
            # compute required dissipated power Preq per eq (23)
            Preq = (self.E_tank_max - self.E_tank) / current_dt + P_cand_vnext - Pin
            # split into trans and rot blocks
            # compute per-axis positive power contributions pj = max(0, wcand_j * vj)

            if Preq <= 0:
                use_damping = False
                E_tank_next = self.E_tank_max
                rospy.loginfo_throttle(1.0, "Energy overflow detected but Preq <= 0 -> no damping applied.")
            else:
                p_in = np.maximum(0.0, wcand * vhand_next)
                sum_p_in = np.sum(p_in) + 1e-12
                sum_p_in_trans = np.sum(p_in[0:3]) + 1e-12
                sum_p_in_rot = np.sum(p_in[3:6]) + 1e-12

                if sum_p_in > 1e-12:
                    Preq_trans = Preq * (sum_p_in_trans / sum_p_in)
                else:
                    Preq_trans = Preq * 0.5
                Preq_rot = Preq - Preq_trans
            
                qhat = np.zeros(6)
                denom_trans = 0
                denom_rot = 0
                for j in range(len(qhat)):
                    if j < 3:
                        qhat[j] = p_in[j] / sum_p_in_trans
                    else:
                        qhat[j] = p_in[j] / sum_p_in_rot
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
                rospy.loginfo_throttle(1.0, f"Energy overflow: adding damping, Dadd diag approx = {Dadd}")

        # --- compute final feedback wrench ---
        if use_QP:
            wfeedback = w_opt
        if use_damping:
            wfeedback = wcand - (Dadd * vhand_next)
        else:
            wfeedback = wcand

        # store last feedback command (for RLS y)
        self.last_feedback_cmd = wfeedback.copy()

        # update E_tank
        self.E_tank = E_tank_next

        # publish as WrenchStamped
        msg = WrenchStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        if self.cfs_connection_state:
            msg.wrench.force.x = float(wfeedback[0])
            msg.wrench.force.y = float(wfeedback[1])
            msg.wrench.force.z = float(wfeedback[2])
            msg.wrench.torque.x = float(wfeedback[3])
            msg.wrench.torque.y = float(wfeedback[4])
            msg.wrench.torque.z = float(wfeedback[5])
        else:
            msg.wrench.force.x = float(0.0)
            msg.wrench.force.y = float(0.0)
            msg.wrench.force.z = float(0.0)
            msg.wrench.torque.x = float(0.0)
            msg.wrench.torque.y = float(0.0)
            msg.wrench.torque.z = float(0.0)

        self.wrench_pub.publish(msg)

        # publish for debug

        handnext_msg = Float32MultiArray()
        handnext_msg.data = [float(vhand_next[0]), float(vhand_next[1]), float(vhand_next[2]), float(vhand_next[3]), float(vhand_next[4]), float(vhand_next[5])]
        self.debug_handnext_pub.publish(handnext_msg)

        whand_est_msg = Float32MultiArray()
        whand_est_msg.data = [float(whand_est[0]), float(whand_est[1]), float(whand_est[2]), float(whand_est[3]), float(whand_est[4]), float(whand_est[5])]
        self.debug_whand_est_pub.publish(whand_est_msg)

        energy_msg = Float32MultiArray()
        energy_msg.data = [float(self.E_tank), float(Pin), float(P_cand_vnext), float(dEpred)]
        self.debug_energy_pub.publish(energy_msg)

        # scale_msg = Float32MultiArray()
        # scale_msg.data = [float(alpha)]
        # self.debug_scale_pub.publish(scale_msg)

        dadd_msg = Float32MultiArray()
        dadd_msg.data = [float(Dadd[0,0]), float(Dadd[1,1]), float(Dadd[2,2]), float(Dadd[3,3]), float(Dadd[4,4]), float(Dadd[5,5])]
        self.debug_dadd_pub.publish(dadd_msg)

        impedance_x_msg = Float32MultiArray()
        impedance_x_msg.data = [float(self.theta[0,0]), float(self.theta[0,1]), float(self.theta[0,2])]
        self.debug_impedance_x_pub.publish(impedance_x_msg)

        impedance_y_msg = Float32MultiArray()
        impedance_y_msg.data = [float(self.theta[1,0]), float(self.theta[1,1]), float(self.theta[1,2])]
        self.debug_impedance_y_pub.publish(impedance_y_msg)

        impedance_z_msg = Float32MultiArray()
        impedance_z_msg.data = [float(self.theta[2,0]), float(self.theta[2,1]), float(self.theta[2,2])]
        self.debug_impedance_z_pub.publish(impedance_z_msg)

        impedance_roll_msg = Float32MultiArray()
        impedance_roll_msg.data = [float(self.theta[3,0]), float(self.theta[3,1]), float(self.theta[3,2])]
        self.debug_impedance_roll_pub.publish(impedance_roll_msg)

        impedance_pitch_msg = Float32MultiArray()
        impedance_pitch_msg.data = [float(self.theta[4,0]), float(self.theta[4,1]), float(self.theta[4,2])]
        self.debug_impedance_pitch_pub.publish(impedance_pitch_msg)

        impedance_yaw_msg = Float32MultiArray()
        impedance_yaw_msg.data = [float(self.theta[5,0]), float(self.theta[5,1]), float(self.theta[5,2])]
        self.debug_impedance_yaw_pub.publish(impedance_yaw_msg)


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
