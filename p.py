import numpy as np

nPoints = 1500
# x, y, z = np.mgrid[-10:10, -20:20, 0.5:1]

# xx = x.flatten()
# yy = y.flatten()
# zz = z.flatten()

temp = np.random.random_sample((nPoints, 3)) * 20 - 10
xx = temp[:, 0]
yy = temp[:, 1]
zz = np.ones(nPoints)

w_points = np.vstack([xx, yy, zz])

print(xx, yy)

width  = 1857
height = 1404
# width  = 752
# height = 480
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

figure = plt.figure()
# ax = figure.add_subplot(111, projection='3d')
figure.set_size_inches(width/100, height/100)
# ax.view_init(elev=0, azim=90)
plt.axis('off')
# ax.scatter(w_points[0], w_points[1], w_points[2], s=0.1)
plt.scatter(xx, yy, c=np.arange(nPoints), s=7, cmap='jet')
plt.xlim(-10, 10)
plt.ylim(-10, 10)
# plt.gca().set_position([0, 0, 1, 1])  
plt.savefig("image.png", dpi=100, bbox_inches='tight', pad_inches=0)

plt.show()
