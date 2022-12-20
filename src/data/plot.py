import matplotlib.pyplot as plt
import numpy as np

err = np.loadtxt("errors.txt")
time = np.loadtxt("time.txt")
vx = np.loadtxt("Vx.txt")
vy = np.loadtxt("Vy.txt")
vz = np.loadtxt("Vz.txt")
vyaw = np.loadtxt("Vyaw.txt")
lamb = np.loadtxt("lambda.txt")

if len(vx) < 2: exit()
for i in range(2, len(vx)):
    if np.linalg.norm(vx[i]-vx[i-1]) > 1e-3:
      NUM = i
      break


print(f"""INFORMACION DEL EXPERIMENTO

Tiempo de vuelo: {time[-1]} s
Error promedio: {err[-1]} pixeles
Velocidad media en x: {np.mean(vx[NUM:])} u/s
Velocidad media en y: {np.mean(vy[NUM:])} u/s
Velocidad media en z: {np.mean(vz[NUM:])} u/s
Velocidad media en yaw: {np.mean(vyaw[NUM:])} u/s
Lambda promedio: {np.mean(lamb[NUM:])}

Error final: {err[-1]} pixeles
Velocidad final en x: {vx[-1]} u/s
Velocidad final en y: {vy[-1]} u/s
Velocidad final en z: {vz[-1]} u/s
Velocidad final en yaw: {vyaw[-1]} u/s""")

fig, ax = plt.subplots()
ax.plot(time[NUM:],err[NUM:])

plt.ylabel('Error Promedio (pixeles)')
plt.xlabel('Tiempo (s)')
plt.title('Error promedio en funcion del tiempo')
plt.show()

fig, ax = plt.subplots()
ax.plot(time[NUM:], vx[NUM:], label='$V_x$')
ax.plot(time[NUM:],vy[NUM:],label='$V_y$')
ax.plot(time[NUM:],vz[NUM:],label='$V_z$')
ax.plot(time[NUM:],vyaw[NUM:],label='$W_z$')
legend = ax.legend(loc='best', shadow=True, fontsize='x-large')

plt.ylabel('Velocidades (u/s)')
plt.xlabel('Tiempo (s)')
plt.title('Velocidades en funcion del tiempo')
plt.show()

fig, ax = plt.subplots()
ax.plot(time[NUM:],lamb[NUM:])

plt.ylabel('Lambda')
plt.xlabel('Tiempo (s)')
plt.title('Lambda en funcion del tiempo')
plt.show()

