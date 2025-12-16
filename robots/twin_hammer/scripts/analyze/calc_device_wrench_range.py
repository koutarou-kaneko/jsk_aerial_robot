#!/usr/bin/env python3
import yaml
import numpy as np
import itertools
import matplotlib.pyplot as plt
from pathlib import Path
import time
import sys

# ============================================================
# USER SETTINGS (resolution & progress)
# ============================================================
ANGLE_STEP_DEG = 30.0   # deflection angle resolution [deg]
THRUST_STEP_N = 5.0     # thrust resolution [N]

OUTPUT_DIR = Path("/home/kaneko/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/twin_hammer/scripts/analyze")
# OUTPUT_DIR.mkdir(exist_ok=True)

# Progress print every N percent (integer 1-100). Set to 1 for frequent prints.
PROGRESS_PERCENT_STEP = 1

# ============================================================
# Utility functions
# ============================================================
def skew(p):
    return np.array([
        [0.0,     -p[2],  p[1]],
        [p[2],   0.0,    -p[0]],
        [-p[1],  p[0],   0.0]
    ])

def rot_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1, 0,  0],
        [0, c, -s],
        [0, s,  c]
    ])

def rot_y(phi):
    c, s = np.cos(phi), np.sin(phi)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c]
    ])

# ============================================================
# Load parameters
# ============================================================
with open("device_params.yaml", "r") as f:
    params = yaml.safe_load(f)

# rotor positions (placeholders allowed)
p_list = [
    np.array(params["geometry"]["p1"], dtype=float),
    np.array(params["geometry"]["p2"], dtype=float),
    np.array(params["geometry"]["p3"], dtype=float),
    np.array(params["geometry"]["p4"], dtype=float),
]

# thrust limits
thrust_min = params["thrust"]["min"]
thrust_max = params["thrust"]["max"]

# deflection limits
theta_min = np.deg2rad(params["deflection"]["theta_min_deg"])
theta_max = np.deg2rad(params["deflection"]["theta_max_deg"])
phi_min   = np.deg2rad(params["deflection"]["phi_min_deg"])
phi_max   = np.deg2rad(params["deflection"]["phi_max_deg"])

# gravity
mass = params["gravity"]["mass"]
gval = params["gravity"]["g"]
g_dir = np.array(params["gravity"]["direction_C"], dtype=float)
gravity_force = mass * gval * g_dir

# ============================================================
# Wrench computation
# ============================================================
def compute_wrench(thetas, phis, thrusts):
    """
    thetas  : [theta_module1, theta_module2]  (rad)
    phis    : [phi_module1,   phi_module2]    (rad)
    thrusts : [lambda1, lambda2, lambda3, lambda4] (N)
    returns: (f_total[3], tau_total[3])
    """
    f_total = np.zeros(3)
    tau_total = np.zeros(3)

    for i in range(4):
        module_id = 0 if i < 2 else 1
        R = rot_y(phis[module_id]) @ rot_x(thetas[module_id])

        thrust_dir = R @ np.array([0.0, 0.0, 1.0])
        fi = thrusts[i] * thrust_dir

        f_total += fi
        tau_total += np.cross(p_list[i], fi)

    # add gravity compensation (gravity_force is negative z if g_dir=[0,0,-1])
    f_total += gravity_force

    return f_total, tau_total

# ============================================================
# Parameter grids
# ============================================================
theta_vals = np.arange(theta_min, theta_max + 1e-12, np.deg2rad(ANGLE_STEP_DEG))
phi_vals   = np.arange(phi_min, phi_max + 1e-12, np.deg2rad(ANGLE_STEP_DEG))
thrust_vals = np.arange(thrust_min, thrust_max + 1e-12, THRUST_STEP_N)

n_theta = len(theta_vals)
n_phi = len(phi_vals)
n_thrust = len(thrust_vals)

# Compute total iterations (note: may be very large)
total_iters = (n_theta ** 2) * (n_phi ** 2) * (n_thrust ** 4)

print(f"Grid sizes: n_theta={n_theta}, n_phi={n_phi}, n_thrust={n_thrust}")
print(f"Total parameter combinations (iterations): {total_iters:,}")
sys.stdout.flush()

if total_iters == 0:
    raise RuntimeError("Grid sizes produce zero combinations. Check resolution and ranges.")

# Choose progress interval in absolute iterations
progress_interval = max(total_iters * PROGRESS_PERCENT_STEP // 100, 1)

# ============================================================
# Exhaustive search with progress prints
# ============================================================
# Pre-allocate lists if reasonable size, otherwise accumulate in Python lists.
# WARNING: For very large total_iters, memory usage will be high.
forces = []
torques = []

iter_count = 0
next_progress_at = progress_interval
start_time = time.time()
last_print_time = start_time

# Nested loops converted to iterators to maintain single counter
# We'll iterate over indices to avoid creating huge intermediate product lists.
theta_indices = range(n_theta)
phi_indices = range(n_phi)
thrust_indices = range(n_thrust)

# We'll convert thrust index to a tuple of 4 thrusts via base-n_thrust enumeration
def thrust_index_to_tuple(idx):
    # idx in [0, n_thrust**4 - 1]
    vals = []
    for _ in range(4):
        vals.append(thrust_vals[idx % n_thrust])
        idx //= n_thrust
    return tuple(vals)

# Outer loops over theta1, theta2, phi1, phi2; inner over thrust combination index
# Precompute counts for nested iteration multipliers to convert a single large loop index if needed.
# Simpler approach: nested loops but update iter_count and check progress periodically.
for i_theta1 in theta_indices:
    theta1 = theta_vals[i_theta1]
    for i_theta2 in theta_indices:
        theta2 = theta_vals[i_theta2]
        for i_phi1 in phi_indices:
            phi1 = phi_vals[i_phi1]
            for i_phi2 in phi_indices:
                phi2 = phi_vals[i_phi2]
                # iterate over all combinations of 4 thrusts
                # Use nested loops when n_thrust is small; else use integer enumeration
                if n_thrust <= 8:
                    for t0 in thrust_vals:
                        for t1 in thrust_vals:
                            for t2 in thrust_vals:
                                for t3 in thrust_vals:
                                    f, tau = compute_wrench([theta1, theta2], [phi1, phi2], [t0, t1, t2, t3])
                                    forces.append(f)
                                    torques.append(tau)
                                    iter_count += 1
                                    if iter_count >= next_progress_at:
                                        now = time.time()
                                        elapsed = now - start_time
                                        ips = iter_count / elapsed if elapsed > 0 else float('inf')
                                        pct = (iter_count / total_iters) * 100
                                        print(f"[{pct:6.2f}%] iters={iter_count:,}/{total_iters:,} "
                                              f"elapsed={elapsed:.1f}s ips={ips:,.1f}")
                                        sys.stdout.flush()
                                        next_progress_at += progress_interval
                else:
                    # enumerate thrust combinations by single index to avoid deep nesting
                    max_thrust_comb = n_thrust ** 4
                    for t_idx in range(max_thrust_comb):
                        t0 = thrust_vals[(t_idx >> 0) % n_thrust]
                        t1 = thrust_vals[(t_idx >> 1) % n_thrust]
                        t2 = thrust_vals[(t_idx >> 2) % n_thrust]
                        t3 = thrust_vals[(t_idx >> 3) % n_thrust]
                        f, tau = compute_wrench([theta1, theta2], [phi1, phi2], [t0, t1, t2, t3])
                        forces.append(f)
                        torques.append(tau)
                        iter_count += 1
                        if iter_count >= next_progress_at:
                            now = time.time()
                            elapsed = now - start_time
                            ips = iter_count / elapsed if elapsed > 0 else float('inf')
                            pct = (iter_count / total_iters) * 100
                            print(f"[{pct:6.2f}%] iters={iter_count:,}/{total_iters:,} "
                                  f"elapsed={elapsed:.1f}s ips={ips:,.1f}")
                            sys.stdout.flush()
                            next_progress_at += progress_interval

# ============================================================
# Convert to arrays
# ============================================================
forces = np.array(forces)
torques = np.array(torques)

# Final progress print
total_elapsed = time.time() - start_time
print(f"Completed iterations: {iter_count:,} in {total_elapsed:.1f}s (avg ips = {iter_count/total_elapsed:,.1f})")
sys.stdout.flush()

# ============================================================
# Output min / max
# ============================================================
print("=== Force range [N] ===")
for i, ax in enumerate(["Fx", "Fy", "Fz"]):
    print(f"{ax}: min = {forces[:, i].min(): .6f}, max = {forces[:, i].max(): .6f}")

print("\n=== Torque range [Nm] ===")
for i, ax in enumerate(["Tx", "Ty", "Tz"]):
    print(f"{ax}: min = {torques[:, i].min(): .6f}, max = {torques[:, i].max(): .6f}")

# ============================================================
# Visualization
# ============================================================
def plot_3d(points, title, labels, filename):
    fig = plt.figure(figsize=(7,6))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1)
    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_zlabel(labels[2])
    ax.set_title(title)
    plt.tight_layout()
    plt.savefig(str(filename))
    plt.show()

plot_3d(
    forces,
    "Force Workspace",
    ["Fx [N]", "Fy [N]", "Fz [N]"],
    OUTPUT_DIR / "force_workspace.png"
)

plot_3d(
    torques,
    "Torque Workspace",
    ["Tx [Nm]", "Ty [Nm]", "Tz [Nm]"],
    OUTPUT_DIR / "torque_workspace.png"
)
