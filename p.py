import numpy as np

nPoints = 150

# width  = 1857
# height = 1404
width  = 752
height = 480

temp_x = np.random.random_sample(nPoints) * width
temp_y = np.random.random_sample(nPoints) * height
temp = np.vstack([temp_x, temp_y]).T
xx = temp[:, 0]
yy = temp[:, 1]
zz = np.ones(nPoints)

w_points = np.vstack([xx, yy, zz])

# print(xx, yy)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

figure = plt.figure()
# ax = figure.add_subplot(111, projection='3d')
figure.set_size_inches(width/100, height/100)
# ax.view_init(elev=0, azim=90)
plt.axis('off')
# ax.scatter(w_points[0], w_points[1], w_points[2], s=0.1)
plt.plot(xx, yy, ".", color="black")#, c=np.arange(nPoints), s=7, cmap='jet')
plt.plot(width/4, height/4, "o", color="red", zorder=10)
plt.plot(width/4 + width/2, height/4, "o", color="red", zorder=10)
plt.plot(width/4, height/4 + height/2, "o", color="red", zorder=10)
plt.plot(width/4 + width/2, height/4 + height/2, "o", color="red", zorder=10)

p1 = np.array([width/4, height/4, 1])
p2 = np.array([width/4 + width/2, height/4, 1])
p3 = np.array([width/4, height/4 + height/2, 1])
p4 = np.array([width/4 + width/2, height/4 + height/2, 1])
P = np.array([p1, p2, p3, p4])

for p in P:
  CHO = np.array(sorted(w_points.T, key=lambda x: np.linalg.norm(x - p)))[0]
  plt.plot(CHO[0], CHO[1], ".", color="blue", zorder=10)

plt.xlim(0, width)
plt.ylim(0, height)

# plt.gca().set_position([0, 0, 1, 1])  
# plt.savefig("image.png", dpi=100, bbox_inches='tight', pad_inches=0)

plt.show()
