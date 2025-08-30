#!/usr/bin/env python3
import os
import pybullet as p
import pybullet_data
import numpy as np
import xacro
import tempfile
import subprocess

# === 設定 ===
NUM_SAMPLES = 500000   # サンプリング数（必要に応じて増やす）
OUTPUT_FILE = "unsafe_configs.npy"

# ------------------------------
# URDFパス
# ------------------------------
script_dir = os.path.dirname(os.path.realpath(__file__))
xacro_path = os.path.join(script_dir, "../../../TCP-IP-ROS-6AXis/dobot_gazebo/urdf/me6_robot.xacro")
urdf_path = os.path.join(script_dir, "me6_robot_converted.urdf")

# ------------------------------
# xacro → urdf 変換
# ------------------------------
print("[INFO] Converting xacro to urdf...")
subprocess.run(["xacro", xacro_path, "-o", urdf_path], check=True)

# STLのパス修正
print("[INFO] Rewriting mesh paths in URDF...")
with open(urdf_path, "r") as f:
    urdf_text = f.read()
urdf_text = urdf_text.replace("package://dobot_description/meshes/me6/",
                              os.path.join(script_dir, "../../../TCP-IP-ROS-6AXis/dobot_description/meshes/me6/"))
with open(urdf_path, "w") as f:
    f.write(urdf_text)

# === PyBullet初期化 ===
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

print("[INFO] Loading URDF into PyBullet...")
robot_id = p.loadURDF(urdf_path, useFixedBase=True)
num_joints = p.getNumJoints(robot_id)
print(f"[INFO] URDF loaded! robot_id = {robot_id}, num_joints = {num_joints}")

# 可動範囲取得
joint_limits = []
joint_indices = []
for j in range(num_joints):
    info = p.getJointInfo(robot_id, j)
    joint_type = info[2]
    if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
        lower, upper = info[8], info[9]
        if lower > upper:  # 無限回転用のチェック
            lower, upper = -np.pi, np.pi
        joint_limits.append((lower, upper))
        joint_indices.append(j)

joint_limits = np.array(joint_limits)
print(f"[INFO] Controllable joints: {len(joint_indices)}")

# === サンプリングして衝突チェック ===
unsafe_configs = []

for i in range(NUM_SAMPLES):
    # ランダムサンプリング
    q = []
    for (lo, hi) in joint_limits:
        q.append(np.random.uniform(lo, hi))
    q = np.array(q)

    # セット
    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=q
    )
    p.stepSimulation()

    # 衝突判定（地面除外）
    contacts = p.getContactPoints(bodyA=robot_id)
    if len(contacts) > 0:
        unsafe_configs.append(q)

    # if i % 200 == 0:
    #     print(f"[INFO] {i}/{NUM_SAMPLES} samples checked...")

# === 保存 ===
unsafe_configs = np.array(unsafe_configs)
np.save(OUTPUT_FILE, unsafe_configs)
print(f"[INFO] Saved {OUTPUT_FILE} with shape {unsafe_configs.shape}")

p.disconnect()
