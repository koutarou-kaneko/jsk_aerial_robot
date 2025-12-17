#!/usr/bin/env python3
import yaml
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import time
import sys
from scipy.optimize import minimize

# ============================================================
# USER SETTINGS
# ============================================================
N_DIRECTION = 5000
ANGLE_INIT_DEG = 0.0
MAX_ITER = 200

OUTPUT_DIR = Path(
    "/home/kaneko/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/twin_hammer/scripts/analyze"
)

# ============================================================
# Utility functions
# ============================================================
def rot_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])

def rot_y(phi):
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])

# ============================================================
# Load parameters
# ============================================================
with open("device_params.yaml", "r") as f:
    params = yaml.safe_load(f)

p_list = [
    np.array(params["geometry"]["p1"], dtype=float),
    np.array(params["geometry"]["p2"], dtype=float),
    np.array(params["geometry"]["p3"], dtype=float),
    np.array(params["geometry"]["p4"], dtype=float),
]

thrust_min = params["thrust"]["min"]
thrust_max = params["thrust"]["max"]

theta_min = np.deg2rad(params["deflection"]["theta_min_deg"])
theta_max = np.deg2rad(params["deflection"]["theta_max_deg"])
phi_min   = np.deg2rad(params["deflection"]["phi_min_deg"])
phi_max   = np.deg2rad(params["deflection"]["phi_max_deg"])

mass = params["gravity"]["mass"]
gval = params["gravity"]["g"]
g_dir = np.array(params["gravity"]["direction_C"], dtype=float)
gravity_force = mass * gval * g_dir

# ============================================================
# Scaling (IMPORTANT)
# ============================================================
force_scale = 4.0 * thrust_max
torque_scale = max(np.linalg.norm(p) for p in p_list) * thrust_max

# ============================================================
# Wrench computation
# ============================================================
def compute_wrench(thetas, phis, thrusts):
    f_total = np.zeros(3)
    tau_total = np.zeros(3)

    for i in range(4):
        module_id = 0 if i < 2 else 1
        R = rot_y(phis[module_id]) @ rot_x(thetas[module_id])
        thrust_dir = R @ np.array([0.0, 0.0, 1.0])
        fi = thrusts[i] * thrust_dir
        f_total += fi
        tau_total += np.cross(p_list[i], fi)

    f_total += gravity_force
    return np.hstack([f_total, tau_total])

# ============================================================
# Support function optimization (scaled)
# ============================================================
def support_value(direction):
    d = direction / np.linalg.norm(direction)

    d_f = d[0:3] / force_scale
    d_t = d[3:6] / torque_scale

    def objective(x):
        thetas = [x[0], x[1]]
        phis   = [x[2], x[3]]

        thrusts = []
        for i in range(4):
            module_id = 0 if i < 2 else 1
            R = rot_y(phis[module_id]) @ rot_x(thetas[module_id])
            thrust_dir = R @ np.array([0.0, 0.0, 1.0])

            w_i = np.hstack([
                thrust_dir,
                np.cross(p_list[i], thrust_dir)
            ])

            score = d_f @ w_i[0:3] + d_t @ w_i[3:6]
            thrusts.append(thrust_max if score > 0 else thrust_min)

        w = compute_wrench(thetas, phis, thrusts)
        return -(d_f @ w[0:3] + d_t @ w[3:6])

    x0 = np.deg2rad([ANGLE_INIT_DEG]*4)
    bounds = [
        (theta_min, theta_max),
        (theta_min, theta_max),
        (phi_min, phi_max),
        (phi_min, phi_max)
    ]

    res = minimize(
        objective,
        x0,
        method="L-BFGS-B",
        bounds=bounds,
        options={"maxiter": MAX_ITER}
    )

    x = res.x
    thetas = [x[0], x[1]]
    phis   = [x[2], x[3]]

    thrusts = []
    for i in range(4):
        module_id = 0 if i < 2 else 1
        R = rot_y(phis[module_id]) @ rot_x(thetas[module_id])
        thrust_dir = R @ np.array([0.0, 0.0, 1.0])
        w_i = np.hstack([thrust_dir, np.cross(p_list[i], thrust_dir)])
        score = d_f @ w_i[0:3] + d_t @ w_i[3:6]
        thrusts.append(thrust_max if score > 0 else thrust_min)

    return compute_wrench(thetas, phis, thrusts)

# ============================================================
# Direction sampling (force / torque mixed)
# ============================================================
np.random.seed(0)

dirs_force = np.hstack([
    np.random.randn(N_DIRECTION//2, 3),
    0.2 * np.random.randn(N_DIRECTION//2, 3)
])

dirs_torque = np.hstack([
    0.2 * np.random.randn(N_DIRECTION//2, 3),
    np.random.randn(N_DIRECTION//2, 3)
])

directions = np.vstack([dirs_force, dirs_torque])
directions /= np.linalg.norm(directions, axis=1, keepdims=True)

# ============================================================
# Main loop
# ============================================================
wrenches = []
start_time = time.time()

for i, d in enumerate(directions):
    w = support_value(d)
    wrenches.append(w)

    pct = (i + 1) / len(directions) * 100
    elapsed = time.time() - start_time
    print(f"[{pct:6.2f}%] directions={i+1}/{len(directions)} elapsed={elapsed:.1f}s")
    sys.stdout.flush()

wrenches = np.array(wrenches)

print(f"Completed support function sampling in {time.time() - start_time:.1f}s")

# ============================================================
# Visualization
# ============================================================
def plot_3d(points, title, labels, filename):
    fig = plt.figure(figsize=(7,6))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=8)
    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_zlabel(labels[2])
    ax.set_title(title)
    plt.tight_layout()
    plt.savefig(str(filename))
    plt.show()

plot_3d(
    wrenches[:, 0:3],
    "Force Workspace Boundary (Support Function)",
    ["Fx [N]", "Fy [N]", "Fz [N]"],
    OUTPUT_DIR / "force_workspace_support.png"
)

plot_3d(
    wrenches[:, 3:6],
    "Torque Workspace Boundary (Support Function)",
    ["Tx [Nm]", "Ty [Nm]", "Tz [Nm]"],
    OUTPUT_DIR / "torque_workspace_support.png"
)
