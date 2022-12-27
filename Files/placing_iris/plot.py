import matplotlib.pyplot as plt
import numpy as np 
import sys

error = np.loadtxt('error.txt')[1:]
x = np.loadtxt('x.txt')[1:]
y = np.loadtxt('y.txt')[1:]
z = np.loadtxt('z.txt')[1:]
yaw = np.loadtxt('yaw.txt')[1:]
params = np.loadtxt('params.txt')


labels = ["", "x_d", "y_d", "z_d", "yaw_d"]
colors = ["", "r", "g", "b", "y"]


fig, ax = plt.subplots(1, 2, figsize=(10, 5))
ax[0].plot(x, label="x", color=colors[1], alpha=0.7)
ax[0].plot(y, label="y", color=colors[2], alpha=0.7)
ax[0].plot(z, label="z", color=colors[3], alpha=0.7)
ax[0].plot(yaw, label="yaw", color=colors[4], alpha=0.7)

for i in range(len(params)):
  l = params[i]
  # print(labels[i+1], " -> ", l)
  ax[0].plot([0, len(x)], [l, l], '--', label=labels[i+1], color=colors[i+1])

ax[0].legend()
ax[0].set_xlabel('Iteration')
ax[0].set_ylabel('Position')
ax[0].set_title('Position')


ax[1].plot(error, label="error")
ax[1].legend()
ax[1].set_xlabel('Iteration')
ax[1].set_ylabel('Error')
ax[1].set_title('Error')

print(f"""
Errores:
x[-1]: {x[-1]}\t x_d: {params[0]}\t error: {x[-1] - params[0]}
y[-1]: {y[-1]}\t y_d: {params[1]}\t error: {y[-1] - params[1]}
z[-1]: {z[-1]}\t z_d: {params[2]}\t error: {z[-1] - params[2]}
yaw[-1]: {yaw[-1]}\t yaw_d: {params[3]}\t error: {yaw[-1] - params[3]}
""")

plt.show()