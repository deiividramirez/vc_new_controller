# -*- coding: utf-8 -*-
"""
    2018
    @author: robotics lab (Patricia Tavares)
    @email: patricia.tavares@cimat.mx
    version: 1.0
    This code cointains a position based control
    of a camera using the Essential Matrix.

    Edited by: 
        David Leonardo Ramírez Parada
        2022
"""
from Functions.PlanarCamera import PlanarCamera
import matplotlib.pyplot as plt
from numpy.linalg import inv
import numpy as np
import sys

global CONTROL
CONTROL = 1

#================================================================Functions
class vecDist:
    def __init__(self, i: int, j:int, dist:float, dist2:float) -> None:
        self.i = i
        self.j = j
        self.dist = dist
        self.dist2 = dist2

def toSpehere(p1, p2, camera):
    p1s  = np.zeros((3, p1.shape[1]))
    p2s  = np.zeros((3, p2.shape[1]))
    p23D = np.zeros((3, p2.shape[1]))
    for i in range(p1.shape[1]):
        temp = np.concatenate( [p1[:, i], [1]] ).reshape(3,1)
        temp = inv(camera.K) @ temp
        p1s[:, i] = temp.T / np.linalg.norm(temp)
        
        temp = np.concatenate( [p2[:, i], [1]] ).reshape(3,1)
        temp = inv(camera.K) @ temp
        p2s[:, i] = temp.T / np.linalg.norm(temp)
        p23D[:, i] = temp.T

    return p1s, p2s, p23D

def distances(p1, p2):
    distancias, error = [], []
    for i in range(p1.shape[1]):
        for j in range(i):
            if i != j:
                dist  = np.sqrt( 2 - 2 * np.dot(p2[:,i], p2[:,j]) )
                dist2 = np.sqrt( 2 - 2 * np.dot(p1[:,i], p1[:,j]) ) 
                if dist <= 1e-9 or dist2 <= 1e-9:
                    continue

                distancias.append(
                    vecDist(i, j, 1/dist, 1/dist2) if CONTROL == 1 else vecDist(i, j, dist, dist2)
                )
            
    distancias = sorted(distancias, key=lambda x: x.dist, reverse=True)
    error = [i.dist2 - i.dist for i in distancias]
    return np.array(error).reshape(len(error), 1), distancias

def ortoProj(p1):
    return np.eye(3) - p1 @ p1.T

def Lvl(p23D, p2s, distancias):
    n = len(distancias)
    L = np.zeros((n, 3))

    for i in range(n):
        s = -distancias[i].dist**3 if CONTROL == 1 else 1/distancias[i].dist
        
        temp = s * ( (p2s[:, distancias[i].i].reshape(1,3)) @ ortoProj(p2s[:, distancias[i].j].reshape(3,1)) + 
                     (p2s[:, distancias[i].j].reshape(1,3)) @ ortoProj(p2s[:, distancias[i].i].reshape(3,1)) )
        
        L[i, :] = temp
    return L

def main():
    args = sys.argv
    if len(args) == 1:
        print("Se usará el control por defecto := 1")
    else:
        print("Se usará el control := 2")
        global CONTROL
        CONTROL = int(args[1])
        # CONTROL = 2


    UP = "\x1B[3A"
    CLR = "\x1B[0K"
    print("\n\n")

    #================================================================point cloud
    xx       = np.loadtxt('cloud/x.data')
    yy       = np.loadtxt('cloud/y.data')
    zz       = np.loadtxt('cloud/z.data')
    # xx       = np.loadtxt('cloud/x copy.data')
    # yy       = np.loadtxt('cloud/y copy.data')
    # zz       = np.loadtxt('cloud/z copy.data')
    n_points = len(xx)
    w_points = np.vstack([xx, yy, zz])
    print("Number of points: ", n_points)

    #==============================================================target camera
    target_x        =  0.0
    target_y        =  0.0
    target_z        =  6.0
    target_roll     = np.deg2rad(0.0) # Degrees to radians 'x'
    target_pitch    = np.deg2rad(0.0) # Degrees to radians 'y'
    target_yaw      = np.deg2rad(0.0) # Degrees to radians 'z'
    target_camera = PlanarCamera() # Set the target camera
    target_camera.set_position(target_x, target_y, target_z,target_roll, target_pitch, target_yaw)
    p_target = target_camera.projection(w_points, n_points) # Project the points for camera 1

    #=============================================================current camera
    init_x        =  0.0
    init_y        =  0.0
    init_z        =  8.0
    init_pitch    = np.deg2rad(0.0)
    init_roll     = np.deg2rad(0.0)
    init_yaw      = np.deg2rad(0.0)
    moving_camera = PlanarCamera() # Set the init camera
    moving_camera.set_position(init_x, init_y, init_z,init_roll, init_pitch, init_yaw)
    p_moving = moving_camera.projection(w_points,n_points)

    #============================================================defining params
    dt    = 0.01   # Time Delta, seconds.
    t0    = 0      # Start time of the simulation
    steps = 1_000   # max iterations

    #==================================================variables to use and plot
    U = np.zeros((6,1))
    UArray              = np.zeros((6,steps))           # Matrix to save controls history
    tArray              = np.zeros(steps)               # Matrix to save the time steps
    pixelCoordsArray    = np.zeros((2*n_points,steps))  # List to save points positions on the image
    ErrorArray          = np.zeros(steps)           # Matrix to save error points positions
    positionArray       = np.zeros((3,steps))           # Matrix to save  camera positions
    distancesArray      = np.zeros((6,steps))    # Matrix to save distances between points
    I                   = np.eye(3, 3)

    lamb       = 10
    t          = t0

    #=========================================================auxiliar variables
    x_pos   = init_x
    y_pos   = init_y
    z_pos   = init_z
    roll    = init_roll
    pitch   = init_pitch
    yaw     = init_yaw
    p20     = []
    j       = 0
    err_pix = 10 #error in pixels

    #=============================================================init algorithm
    while( j < steps and err_pix > 1e-3):
    # while( j < steps):

    # ===================== Calculate new translation and rotation values using Euler's method====================
        x_pos  +=  dt * U[0, 0]
        y_pos  +=  dt * U[1, 0] # Note the velocities change due the camera framework
        z_pos  +=  dt * U[2, 0]
        roll   +=  dt * U[3, 0]
        pitch  +=  dt * U[4, 0]
        yaw    +=  dt * U[5, 0]
        

        moving_camera.set_position(x_pos, y_pos, z_pos,roll, pitch, yaw)
        p_moving = moving_camera.projection(w_points,n_points)
        
    # =============================================================================
        # Se envían los puntos a la esfera por medio del modelo genérico de la cámara
        p1s, p2s, p23D = toSpehere(p_target, p_moving, target_camera)

        # Se calculan las distancias entre los puntos de la esfera
        # y se calcula el error, se devuelven solos las 500 distancias con mayor error
        error, distancias = distances(p1s, p2s)
        p1 = p_target[0:2, :].reshape((2*n_points, 1), order='F')
        p2 = p_moving[0:2, :].reshape((2*n_points, 1), order='F')
        errori = p2 - p1

        # Obtención de la matriz Jacobiana L para traslacíon pero no rotación
        L = Lvl(p23D, p2s, distancias)
        # print(L)
        # print(L.shape)
        # exit()

        # Pseudo inversa de L
        Lo = np.linalg.pinv(L)
        # print(Lo)
        # print(Lo.shape)
        # exit()

    # ==================================== CONTROL COMPUTATION =======================================
        """
            CONTROL DE ERROR
            Se debe tener en cuenta que este error es solo para 
            la parte translacional del control, por tanto
            no se usa ningún control para la rotación
        """
        U_temp = -Lo @ error
        U_temp2 = np.array([[target_roll - roll], [target_pitch - pitch], [target_yaw - yaw]])
        U = lamb * np.concatenate((U_temp, U_temp2), axis=0)
        # U = lamb * np.concatenate((U_temp, np.zeros((3,1))), axis=0) #* np.linalg.norm(error)
        
        #Avoiding numerical error
        U[np.abs(U) < 1.0e-9] = 0.0
        
        # Copy data for plot
        UArray[:, j]          = U.reshape((6,))
        tArray[j]             = t
        pixelCoordsArray[:,j] = p_moving.reshape(( 2 * n_points,1), order='F')[:,0]
        positionArray[0, j]   = x_pos
        positionArray[1, j]   = y_pos
        positionArray[2, j]   = z_pos
        distancesArray[:, j]  = [i.dist for i in distancias]
        # print([i.dist for i in distancias])
        # print(len(distancias))
        # exit()

    # =================================== Average feature error ======================================
        
        ErrorArray[j]         = np.linalg.norm(error)
        err_pix               = np.linalg.norm(error)
        
    # ==================================== Printing Dat =======================================
        t += dt
        j += 1
        if (j) % 5 == 0:
            print(f"""{UP}{j}/{steps} -> \tx: {x_pos:.5f}, y: {y_pos:.5f}, z: {z_pos:.5f}, roll: {roll:.5f}, pitch: {pitch:.5f}, yaw: {yaw:.5f}{CLR}
    \tU: {np.round(U.T, 5)}, |e|: {np.linalg.norm(error):.5f}...{CLR}\n""")

        if err_pix > 600:
            break

    print(f"""{UP}{j}/{steps} -> \tx: {x_pos:.5f}, y: {y_pos:.5f}, z: {z_pos:.5f}, roll: {roll:.5f}, pitch: {pitch:.5f}, yaw: {yaw:.5f}{CLR}
    \tU: {np.round(U.T, 5)}, |e|: {np.linalg.norm(error):.5f}{CLR}\n""")

    print(f"Finished at: {j} steps -- Error: {np.linalg.norm(error)}")

    # ======================================  Draw cameras ========================================
    colores = np.random.rand(n_points,3)

    fig = plt.figure(figsize=(10, 10))
    fig.suptitle(f'Control #{1 if CONTROL==1 else 2}: {n_points} points', fontsize=16)
    ax = fig.add_subplot(2, 2, 1, projection='3d')
    ax = fig.gca()
    ax.plot(xx, yy, zz, 'o')
    ax.plot(positionArray[0,0:j],positionArray[1,0:j],positionArray[2,0:j]) # Plot camera trajectory
    axis_scale   = 0.5
    camera_scale = 0.02
    target_camera.draw_camera(ax, scale=camera_scale, color='red')
    target_camera.draw_frame(ax, scale=axis_scale, c='black')
    moving_camera.set_position(x_pos, y_pos, z_pos,roll,pitch,yaw)
    moving_camera.draw_camera(ax, scale=camera_scale, color='black')
    moving_camera.draw_frame(ax, scale=axis_scale, c='black')
    moving_camera.set_position(init_x, init_y, init_z,init_roll, init_pitch, init_yaw)
    moving_camera.draw_camera(ax, scale=camera_scale, color='blue')
    moving_camera.draw_frame(ax, scale=axis_scale, c='black')

    ax.set_xlabel("$w_x$")
    ax.set_ylabel("$w_y$")
    ax.set_zlabel("$w_z$")

    # ======================================  Plot the pixels ==========================================
    ax = fig.add_subplot(2, 2, 2)
    p20 = pixelCoordsArray[:,0].reshape((2, n_points), order='F')

    ax.set_ylim(0, target_camera.height)
    ax.set_xlim(0, target_camera.width)

    for i in range(0, 2*n_points, 2):
        ax.plot(pixelCoordsArray[i, 0:j],
                pixelCoordsArray[i+1, 0:j], color=colores[i//2], alpha=0.5)
        if i != 0:
            ax.plot(p_target[0, i//2],  p_target[1, i//2], 'o', color=colores[i//2])
            ax.plot(p20[0, i//2], p20[1, i//2], '.', color=colores[i//2])
        else:
            ax.plot(p_target[0, i//2],  p_target[1, i//2], 'o',
                    label="Deseada", color=colores[i//2])
            ax.plot(p20[0, i//2], p20[1, i//2], '.',
                    label="Inicio", color=colores[i//2])

    ax.legend(loc="best")

    # ======================================  Plot the controls ========================================
    ax = fig.add_subplot(2, 2, 3)
    ax.plot(tArray[0:j], UArray[0, 0:j], label='$v_x$')
    ax.plot(tArray[0:j], UArray[1, 0:j], label='$v_y$')
    ax.plot(tArray[0:j], UArray[2, 0:j], label='$v_z$')
    ax.plot(tArray[0:j], UArray[3, 0:j], label='$\omega_x$')
    ax.plot(tArray[0:j], UArray[4, 0:j], label='$\omega_y$')
    ax.plot(tArray[0:j], UArray[5, 0:j], label='$\omega_z$')
    ax.grid(True)
    ax.legend(loc="center right")

    # ======================================  Plot the pixels position ===================================

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(tArray[0:j], ErrorArray[0:j], label='$Error$')

    ax.grid(True)
    ax.legend(loc="upper right")

    
    fig, ax = plt.subplots()
    ax.set_title("Distancias")
    for i in range(6):
        ax.plot(distancesArray[i, 0:j], label=f'p{i}', color=colores[i//2])
        ax.plot([0, j], [distancias[i].dist2, distancias[i].dist2], '--', color=colores[i//2], label=f'p{i}*')
    
    ax.legend(loc="upper right")
    plt.show()


if __name__ == "__main__":
    main()