import matplotlib.pyplot as plt
import numpy as np
import os

err = np.loadtxt("errors.txt")
err_pix = np.loadtxt("errors_pix.txt")
time = np.loadtxt("time.txt")
vx = np.loadtxt("Vx.txt")
vy = np.loadtxt("Vy.txt")
vz = np.loadtxt("Vz.txt")
vyaw = np.loadtxt("Vyaw.txt")
lamb = np.loadtxt("lambda.txt")

NUM = 0
try:
    if len(vx) < 3:
        exit()
except:
    exit()
for i in range(2, len(vx)):
    if np.linalg.norm(vx[i]-vx[i-1]) > 1e-3:
        NUM = i
        break


print(f"""


INFORMACION DEL EXPERIMENTO

Tiempo de vuelo: {time[-1]} s
Error promedio: {np.mean(err[NUM:])} pixeles
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

fig, ax = plt.subplots(3, 1, figsize=(5, 10))
# fig.canvas.set_window_title('Resultados del experimento')

ax[0].plot(time[NUM:], err[NUM:], "purple", label='Error')
err_pix = err_pix / max(err_pix)
ax[0].plot(time[NUM:], err_pix[NUM:], "r", label='Error en pixeles')
# ax[0].legend(loc='best', shadow=True)
ax[0].set_ylabel('Error Promedio')


ax[1].plot(time[NUM:], vx[NUM:], label='$V_x$')
ax[1].plot(time[NUM:], vy[NUM:], label='$V_y$')
ax[1].plot(time[NUM:], vz[NUM:], label='$V_z$')
ax[1].plot(time[NUM:], vyaw[NUM:], label='$W_z$')
# ax[1].legend(loc='best', shadow=True)
ax[1].set_ylabel('Velocidades')


ax[2].plot(time[NUM:], lamb[NUM:], "r")
ax[2].set_ylabel('Lambda')
ax[2].set_xlabel('Tiempo (s)')


plt.tight_layout()



fig, ax = plt.subplots(1, 2, figsize=(10, 5))
imgs = sorted(os.listdir("./img"), key=lambda x: int(x.split(".")[0]))
des = "../desired2.jpg"

ax[0].imshow(plt.imread(des))
ax[0].set_title("Imagen deseada")
ax[0].axis("off")
ax[1].imshow(plt.imread("./img/"+imgs[-1]))
ax[1].set_title("Imagen final")
ax[1].axis("off")

plt.show()
