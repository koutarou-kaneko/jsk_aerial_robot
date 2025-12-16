#!/usr/bin/env python3
import yaml
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import time
import sys
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# ============================================================
# USER SETTINGS
# ============================================================
ANGLE_STEP_DEG = 30.0        # coarse angular resolution
N_DIRECTIONS = 100           # number of wrench directions to probe
PROGRESS_PRINT_STEP = 10     # print every N directions

OUTPUT_DIR = Path("/home/kaneko/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/twin_hammer/scripts/analyze")

# ============================================================
# Utility functions
# ============================================================
def rot_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def rot_y(phi):
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

# ============================================================
# Load parameters
# ============================================================
with open("device_params.yaml", "r") as f:
    params = yaml.safe_load(f)

p_list = [
    np.array(params["geometry"]["p1"], float),
    np.array(params["geometry"]["p2"], float),
    np.array(params["geometry"]["p3"], float),
    np.array(params["geometry"]["p4"], float),
]

thrust_min = params["thrust"]["min"]
thrust_max = params["thrust"]["max"]

theta_min = np.deg2rad(params["deflection"]["theta_min_deg"])
theta_max = np.deg2rad(params["deflection"]["theta_max_deg"])
phi_min   = np.deg2rad(params["deflection"]["phi_min_deg"])
phi_max   = np.deg2rad(params["deflection"]["phi_max_deg"])

mass = params["gravity"]["mass"]
gval = params["gravity"]["g"]
g_dir = np.array(params["gravity"]["direction_C"], float)
gravity_force = mass * gval * g_dir

# ============================================================
# Wrench computation
# ============================================================
def compute_wrench(thetas, phis, thrusts):
    f = np.zeros(3)
    tau = np.zeros(3)

    for i in range(4):
        m = 0 if i < 2 else 1
        R = rot_y(phis[m]) @ rot_x(thetas[m])
        dir_i = R @ np.array([0,0,1])
        fi = thrusts[i] * dir_i
        f += fi
        tau += np.cross(p_list[i], fi)

    f += gravity_force
    return np.hstack([f, tau])

# ============================================================
# Direction sampling (6D unit vectors)
# ============================================================
def sample_directions(n):
    D = np.random.randn(n, 6)
    D /= np.linalg.norm(D, axis=1, keepdims=True)
    return D

# ============================================================
# Angle grids (coarse)
# ============================================================
theta_vals = np.arange(theta_min, theta_max + 1e-12, np.deg2rad(ANGLE_STEP_DEG))
phi_vals   = np.arange(phi_min, phi_max + 1e-12, np.deg2rad(ANGLE_STEP_DEG))

# ============================================================
# Directional Optimization
# ============================================================
dirs = sample_directions(N_DIRECTIONS)
boundary_wrenches = []

start_time = time.time()

for k, d in enumerate(dirs):
    best_val = -np.inf
    best_w = None

    for theta1 in theta_vals:
        for theta2 in theta_vals:
            for phi1 in phi_vals:
                for phi2 in phi_vals:

                    # thrust extreme selection
                    # compute contribution of each rotor
                    w_unit = []
                    for i in range(4):
                        m = 0 if i < 2 else 1
                        R = rot_y([phi1, phi2][m]) @ rot_x([theta1, theta2][m])
                        fi = R @ np.array([0,0,1])
                        tau_i = np.cross(p_list[i], fi)
                        w_unit.append(np.hstack([fi, tau_i]))

                    thrusts = []
                    for wi in w_unit:
                        thrusts.append(thrust_max if d @ wi >= 0 else thrust_min)

                    w = compute_wrench(
                        [theta1, theta2],
                        [phi1, phi2],
                        thrusts
                    )

                    val = d @ w
                    if val > best_val:
                        best_val = val
                        best_w = w

    boundary_wrenches.append(best_w)

    if (k + 1) % PROGRESS_PRINT_STEP == 0 or k == 0:
        elapsed = time.time() - start_time
        print(f"[Dir {k+1}/{N_DIRECTIONS}] elapsed={elapsed:.1f}s")
        sys.stdout.flush()

boundary_wrenches = np.array(boundary_wrenches)

# ============================================================
# Output ranges
# ============================================================
print("\n=== Wrench range ===")
labels = ["Fx","Fy","Fz","Tx","Ty","Tz"]
for i, l in enumerate(labels):
    print(f"{l}: min={boundary_wrenches[:,i].min():.4f}, "
          f"max={boundary_wrenches[:,i].max():.4f}")

# ============================================================
# Visualization
# ============================================================
def plot_3d(data, idx, title, labels, filename):
    fig = plt.figure(figsize=(7,6))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(data[:,idx[0]], data[:,idx[1]], data[:,idx[2]], s=5)
    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_zlabel(labels[2])
    ax.set_title(title)
    plt.tight_layout()
    plt.savefig(filename)
    plt.show()

# plot_3d(
#     boundary_wrenches, [0,1,2],
#     "Force Workspace (Boundary)",
#     ["Fx","Fy","Fz"],
#     OUTPUT_DIR / "force_workspace_boundary.png"
# )

# plot_3d(
#     boundary_wrenches, [3,4,5],
#     "Torque Workspace (Boundary)",
#     ["Tx","Ty","Tz"],
#     OUTPUT_DIR / "torque_workspace_boundary.png"
# )


# ============================================================
# Visualization (Convex Hull)
# ============================================================
def plot_convex_hull(points, title, labels, filename):
    """
    points: (N,3)
    """
    hull = ConvexHull(points)

    fig = plt.figure(figsize=(7,6))
    ax = fig.add_subplot(111, projection="3d")

    # draw hull facets
    faces = []
    for simplex in hull.simplices:
        faces.append(points[simplex])

    poly = Poly3DCollection(
        faces,
        facecolor="cyan",
        edgecolor="k",
        alpha=0.5
    )
    ax.add_collection3d(poly)

    # also plot vertices lightly
    ax.scatter(points[:,0], points[:,1], points[:,2], s=5)

    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_zlabel(labels[2])
    ax.set_title(title)

    # axis scaling
    max_range = (points.max(axis=0) - points.min(axis=0)).max() / 2
    mid = points.mean(axis=0)
    ax.set_xlim(mid[0]-max_range, mid[0]+max_range)
    ax.set_ylim(mid[1]-max_range, mid[1]+max_range)
    ax.set_zlim(mid[2]-max_range, mid[2]+max_range)

    plt.tight_layout()
    plt.savefig(filename)
    plt.show()

# Force hull
plot_convex_hull(
    boundary_wrenches[:, 0:3],
    "Force Workspace Boundary (Convex Hull)",
    ["Fx [N]", "Fy [N]", "Fz [N]"],
    OUTPUT_DIR / "force_workspace_boundary_hull.png"
)

# Torque hull
plot_convex_hull(
    boundary_wrenches[:, 3:6],
    "Torque Workspace Boundary (Convex Hull)",
    ["Tx [Nm]", "Ty [Nm]", "Tz [Nm]"],
    OUTPUT_DIR / "torque_workspace_boundary_hull.png"
)

