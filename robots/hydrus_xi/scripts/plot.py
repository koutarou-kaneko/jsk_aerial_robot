# -*- coding: utf-8
import numpy as np
import matplotlib.pyplot as plt

# CSVのロード
data = np.genfromtxt("target_error1.csv",delimiter=",", skip_header=0, dtype='float')

x=data[:,0]
y=data[:,1]
xt=data[:,2]
yt=data[:,3]
plt.title('End effector trajectory and target position')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.plot(x, y, marker=".", markerfacecolor="r", label="end effector")
plt.plot(xt, yt, marker="o", markerfacecolor="y", label="target position")
plt.show()