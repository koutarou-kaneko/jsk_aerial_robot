# -*- coding: utf-8
import numpy as np
import matplotlib.pyplot as plt

# CSVのロード
data = np.genfromtxt("target1.csv",delimiter=",", skip_header=0, dtype='float')

x=data[:,0]
y=data[:,1]
xt=data[:,2]
yt=data[:,3]
plt.title('End effector trajectory and target position')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.plot(xt, yt, color="y", marker="o", markerfacecolor="y", label="target position")
plt.plot(x, y, color="b", marker=".", markerfacecolor="r", label="end effector")
plt.xlim(0.7, 1.4)
plt.axes().set_aspect('equal')
plt.legend(bbox_to_anchor=(0, 1), loc='upper left', borderaxespad=1, fontsize=8)
plt.show()
