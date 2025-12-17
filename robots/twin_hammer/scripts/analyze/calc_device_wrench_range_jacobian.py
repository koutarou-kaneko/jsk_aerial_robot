#!/usr/bin/env python3
import yaml
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ============================================================
# USER SETTINGS
# ============================================================
N_SAMPLE = 10000
OUTPUT_DIR = Path("./jacobian_analysis")
OUTPUT_DIR.mkdir(exist_ok=True)

# ============================================================
# Rotation utilities
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
with open("device_params_jacobian.yaml", "r") as f:
    params = yaml.safe_load(f)

p_list = [np.array(params["geometry"][f"p{i+1}"], float) for i in range(4)]

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

eps_thrust = params["jacobian"]["diff_eps"]["thrust"]
eps_angle  = params["jacobian"]["diff_eps"]["angle_rad"]

# ============================================================
# Wrench computation
# ============================================================
def compute_wrench(u):
    thrusts = u[0:4]
    thetas = u[4:6]
    phis   = u[6:8]

    f = np.zeros(3)
    tau = np.zeros(3)

    for i in range(4):
        module = 0 if i < 2 else 1
        R = rot_y(phis[module]) @ rot_x(thetas[module])
        d = R @ np.array([0, 0, 1.0])
        fi = thrusts[i] * d
        f += fi
        tau += np.cross(p_list[i], fi)

    f += gravity_force
    return np.hstack([f, tau])

# ============================================================
# Numerical Jacobian
# ============================================================
def numerical_jacobian(u):
    J = np.zeros((6, 8))
    for i in range(8):
        du = np.zeros(8)
        du[i] = eps_thrust if i < 4 else eps_angle
        wp = compute_wrench(u + du)
        wm = compute_wrench(u - du)
        J[:, i] = (wp - wm) / (2 * du[i])
    return J

# ============================================================
# Gravity compensation state
# ============================================================
u0 = np.zeros(8)
u0[0:4] = np.linalg.norm(gravity_force) / 4.0
u0[4:8] = 0.0

# ============================================================
# Sampling
# ============================================================
results = []

np.random.seed(0)
for _ in range(N_SAMPLE):
    du = np.zeros(8)
    du[0:4] = np.random.uniform(-1.0, 1.0, 4)
    du[4:6] = np.random.uniform(theta_min, theta_max, 2)
    du[6:8] = np.random.uniform(phi_min, phi_max, 2)

    u = u0 + du
    w = compute_wrench(u)
    J = numerical_jacobian(u)
    rank = np.linalg.matrix_rank(J)

    results.append((w, rank))

results = np.array(results, dtype=object)

# ============================================================
# Extract data
# ============================================================
w_all = np.vstack(results[:,0])
rank_all = results[:,1].astype(int)

mask_indep = rank_all == 6

F = w_all[:,0:3]
T = w_all[:,3:6]

F_ind = F[mask_indep]
T_ind = T[mask_indep]

# ============================================================
# Boundary output (STDOUT)
# ============================================================
print("\n========== Strictly Independent Wrench Boundary ==========")

labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
for i, lab in enumerate(labels):
    print(f"{lab}: min = {w_all[mask_indep,i].min(): .4f}, "
          f"max = {w_all[mask_indep,i].max(): .4f}")

print(f"|F|: min = {np.linalg.norm(F_ind,axis=1).min(): .4f}, "
      f"max = {np.linalg.norm(F_ind,axis=1).max(): .4f}")
print(f"|T|: min = {np.linalg.norm(T_ind,axis=1).min(): .4f}, "
      f"max = {np.linalg.norm(T_ind,axis=1).max(): .4f}")

# ============================================================
# Visualization
# ============================================================
def plot_3d(data, mask, title, labels, filename):
    fig = plt.figure(figsize=(7,6))
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(data[mask,0], data[mask,1], data[mask,2],
               c="b", s=10, label="rank=6")
    ax.scatter(data[~mask,0], data[~mask,1], data[~mask,2],
               c="r", s=10, label="rank<6")

    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_zlabel(labels[2])
    ax.set_title(title)
    ax.legend()
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / filename)
    plt.show()

plot_3d(F, mask_indep,
        "Force space (Fx, Fy, Fz)",
        ["Fx [N]", "Fy [N]", "Fz [N]"],
        "force_space.png")

plot_3d(T, mask_indep,
        "Torque space (Tx, Ty, Tz)",
        ["Tx [Nm]", "Ty [Nm]", "Tz [Nm]"],
        "torque_space.png")

# ============================================================
# Force–Torque norm plot
# ============================================================
plt.figure(figsize=(6,5))
plt.scatter(np.linalg.norm(F_ind,axis=1),
            np.linalg.norm(T_ind,axis=1),
            c="b", s=15, label="rank=6")
plt.scatter(np.linalg.norm(F[~mask_indep],axis=1),
            np.linalg.norm(T[~mask_indep],axis=1),
            c="r", s=15, label="rank<6")
plt.xlabel("|F| [N]")
plt.ylabel("|T| [Nm]")
plt.title("Force–Torque simultaneous capability")
plt.legend()
plt.tight_layout()
plt.savefig(OUTPUT_DIR / "force_torque_norm.png")
plt.show()
