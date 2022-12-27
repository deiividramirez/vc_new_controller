import matplotlib.pyplot as plt
import numpy as np

err = np.loadtxt("errors.txt")
time = np.loadtxt("time.txt")
vx = np.loadtxt("Vx.txt")
vy = np.loadtxt("Vy.txt")
vz = np.loadtxt("Vz.txt")
vyaw = np.loadtxt("Vyaw.txt")
lamb = np.loadtxt("lambda.txt")

NUM = 0
if len(vx) < 2:
    exit()
for i in range(2, len(vx)):
    if np.linalg.norm(vx[i]-vx[i-1]) > 1e-3:
        NUM = i
        break


print(f"""


INFORMACION DEL EXPERIMENTO

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

fig, ax = plt.subplots(3, 1)
fig.canvas.set_window_title('Resultados del experimento')

ax[0].plot(time[NUM:], err[NUM:], "purple")

ax[0].set_ylabel('Error Promedio')

ax[1].plot(time[NUM:], vx[NUM:], label='$V_x$')
ax[1].plot(time[NUM:], vy[NUM:], label='$V_y$')
ax[1].plot(time[NUM:], vz[NUM:], label='$V_z$')
ax[1].plot(time[NUM:], vyaw[NUM:], label='$W_z$')

ax[1].legend(loc='right', shadow=True)
ax[1].set_ylabel('Velocidades')

ax[2].plot(time[NUM:], lamb[NUM:], "r")
ax[2].set_ylabel('Lambda')
ax[2].set_xlabel('Tiempo (s)')

plt.tight_layout()
plt.show()
