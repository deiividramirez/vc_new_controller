import matplotlib.pyplot as plt
import numpy as np

err = np.loadtxt("errors.txt")
time = np.loadtxt("time.txt")
vx = np.loadtxt("Vx.txt")
vy = np.loadtxt("Vy.txt")
vz = np.loadtxt("Vz.txt")
vyaw = np.loadtxt("Vyaw.txt")
lamb = np.loadtxt("lambda.txt")

fig, ax = plt.subplots()
ax.plot(time[1:],err[1:])

plt.ylabel('Error Promedio (pixeles)')
plt.xlabel('Tiempo (s)')
plt.title('Error promedio en funcion del tiempo')
plt.show()

fig, ax = plt.subplots()
ax.plot(time[1:], vx[1:], label='$V_x$')
ax.plot(time[1:],vy[1:],label='$V_y$')
ax.plot(time[1:],vz[1:],label='$V_z$')
ax.plot(time[1:],vyaw[1:],label='$W_z$')
legend = ax.legend(loc='best', shadow=True, fontsize='x-large')

plt.ylabel('Velocidades (u/s)')
plt.xlabel('Tiempo (s)')
plt.title('Velocidades en funcion del tiempo')
plt.show()

fig, ax = plt.subplots()
ax.plot(time[1:],lamb[1:])

plt.ylabel('Lambda')
plt.xlabel('Tiempo (s)')
plt.title('Lambda en funcion del tiempo')
plt.show()

