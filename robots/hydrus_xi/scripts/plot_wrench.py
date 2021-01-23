# -*- coding: utf-8
import numpy as np
import matplotlib.pyplot as plt

# CSVのロード
data = np.genfromtxt("wrench2.csv",delimiter=",", skip_header=0, dtype='float')
tar = np.genfromtxt("target.csv",delimiter=",", skip_header=0, dtype='float')

x=data[:,0]
y=data[:,1]
tarx=tar[:,0]
tary=tar[:,1]
#z=data[:,2]
plt.title('Translational thrust')
plt.xlabel('Time [s]')
plt.ylabel('Thrust [N]')
plt.plot(x, color="r", label="x")
plt.plot(y, color="g", label="y")
plt.plot(tarx, color="orange", label="x_target")
plt.plot(tary, color="y", label="y_target")
#plt.plot(z, color="b", label="z")
#plt.xticks(np.arange(0, 49.0, step=0.1))
xticks, strs = plt.xticks()
plt.xticks(xticks, ["%1.1f" % x for x in 0.1 * xticks])
plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=1, fontsize=15)
plt.show()
