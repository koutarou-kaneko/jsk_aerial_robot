import rosbag
import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

args = sys.argv
print(len(args))
assert len(args) >= 2, "you must specify the argument."

# Get path
filename = os.path.normpath(os.path.join(os.getcwd(), args[1]))
print(filename)

# Initialize ROS node
rospy.init_node('data_collector', anonymous=True)

# Global variable to store data
data = {
    'sensor1': [],
    'sensor2': []
}

def callback(message, sensor_type):
    if sensor_type == 'sensor1':
        data['sensor1'].append([
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
            message.wrench.torque.x,
            message.wrench.torque.y,
            message.wrench.torque.z
        ])
    elif sensor_type == 'sensor2':
        data['sensor2'].append([
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
            message.wrench.torque.x,
            message.wrench.torque.y,
            message.wrench.torque.z
        ])

# Read from bag file and process data
bag = rosbag.Bag(filename)
for topic, msg, t in bag.read_messages():
    if topic == "/sensor1":
        callback(msg, 'sensor1')
    elif topic == "/sensor2":
        callback(msg, 'sensor2')

bag.close()

# Convert data to numpy arrays for easier processing
np_s1 = np.array(data['sensor1'])  # t行6列の配列
np_s2 = np.array(data['sensor2'])  # 真値

t = np_s1.shape[0]
h = 0  # h の値を適宜設定してください
R = np.array([0, 0, h])
r_m_1 = np.zeros((np_s1.shape[0], 3))
r_fxy_1 = np.zeros(np_s1.shape[0])
r_f_1 = np.zeros(np_s1.shape[0])
error = np.zeros((np_s1.shape[0], 6))

for i in range(t):
    r_m_1[i] = np_s1[i, -3:] + np.cross(R, np_s1[i, 0:3])
    r_fxy_1[i] = (np_s1[i][0]**2 + np_s1[i][1]**2)**(0.5)  # 引き倒し力
    r_f_1[i] = np.dot(np_s1[i, 0:3], np_s1[i, 0:3].T)
    error[i] = np.subtract((np_s1[i, 0:3] + r_m_1[i]), np_s2[i])

# plot
plt.subplot(1, 5, 1)
plt.title("error")
plt.plot(range(t), error[:, 0], 'r', label="x")
plt.xlabel("time[s]")
plt.ylabel("error")
plt.legend()
plt.show()