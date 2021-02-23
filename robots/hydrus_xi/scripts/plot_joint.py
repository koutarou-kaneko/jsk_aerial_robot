# -*- coding: utf-8
import numpy as np
import matplotlib.pyplot as plt

# CSVのロード
data = np.genfromtxt("joint.csv",delimiter=",", skip_header=0, dtype='float')

t=data[:,0]-data[0,0]
x=data[:,1]
y=data[:,2]
z=data[:,3]
#z=data[:,2]
plt.title('Joint angles')
plt.xlabel('Time [s]')
plt.ylabel('Joint Angle [rad]')
plt.plot(t, x, color="r", label="j1")
plt.plot(t, y, color="g", label="j2")
plt.plot(t, z, color="b", label="j3")
plt.xlim(0, 40)
#plt.plot(z, color="b", label="z")
#plt.xticks(np.arange(0, 49.0, step=0.1))
xticks, strs = plt.xticks()
plt.xticks(xticks, ["%1.1f" % x for x in xticks])
plt.legend(bbox_to_anchor=(0, 0), loc='lower left', borderaxespad=1, fontsize=10)
plt.grid(which = "major", axis = "x", color = "gray", alpha = 0.3, linestyle = "-", linewidth = 1)
plt.grid(which = "major", axis = "y", color = "gray", alpha = 0.3, linestyle = "-", linewidth = 1)
plt.show()
