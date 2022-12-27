# Visual-Servo---Paper
Este repositorio fue hecho para implementar una modificación al algoritmo descrito en el paper "Image-based estimation, planning, and control for high-speed flying through multiple openings" de Guo, Dejun and Leang

## Requerimientos

* Python 3
  * Matplotlib
  * Numpy
* ROS Noetic
* Gazebo - ROS Noetic

## Instalación de ROS Noetic (Ubuntu 20.04)

### Instalación con dos líneas

~~~ bash
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
~~~

### Comprobación de los paquetes instalados

~~~ bash
sudo apt-get install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink protobuf-compiler libgoogle-glog-dev ros-noetic-control-toolbox python3-wstool python3-catkin-tools
~~~

### Preparación de Python 3

Si no se tiene instalado pip en *Python 3.7+*:

~~~ bash
sudo curl https://bootstrap.pypa.io/get-pip.py | sudo python3
sudo curl https://bootstrap.pypa.io/get-pip.py | python3
~~~

### Añadir a variables de entorno
* Bash
 ~~~bash
 echo 'export PATH=$PATH:"$HOME/.local/bin"' >> ~/.bashrc
 source ~/.bashrc
 ~~~
* Zsh
 ~~~zsh
 echo 'export PATH=$PATH:"$HOME/.local/bin"' >> ~/.zshrc
 source ~/.zshrc
 ~~~

### Instalación de paquetes de ROS adicionales en Python 3
  
~~~ bash
sudo python3 -m pip install -U rosdep catkin_pkg future
python3 -m pip install -U rosdep catkin_pkg future empy defusedxml numpy matplotlib imageio opencv-python
~~~

## Preparación del espacio CATKIN

Si ya tiene el espacio de trabajo catkin creado, puede omitir este paso.

### Creación del espacio de trabajo catkin

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace  # Inicialización del workspace
```

### Instalación de RotorS y MAVROS

```bash
cd ~/catkin_ws/src
wstool init
wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
git clone https://github.com/ethz-asl/rotors_simulator
git clone https://github.com/ethz-asl/mav_comm
wstool merge rotors_hil.rosinstall
wstool update
```

## Instalación de vc_new_controller

```bash
cd ~/catkin_ws/src
git clone https://github.com/deiividramirez/vc_new_controller
git clone https://github.com/deiividramirez/placing_iris.git
```

### Construcción del espacio

```bash
# Inicialización de rosdep y recolección de paquetes
sudo rosdep init
rosdep update
cd ~/catkin_ws/
# Actualización de paquetes rotos o dependencias
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
# Construcción de catkin_workspace
catkin build
``` 

### Añadir las variables de entorno

* Bash
 ```bash
 echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
 source ~/.bashrc
 ```
* Zsh
 
 ```zsh
 echo "source ~/catkin_ws/devel/setup.zsh" >> ~/.zshrc
 source ~/.zshrc
 ```

## Ejecución

Antes de la ejecución, es necesario los archivos en la carpeta de CATKIN_WS, si el espacio de trabajo no tiene este nombre, cambiar la variable `WORKSPACE` por el nombre del espacio de trabajo. Además, el siguiente código se debe ejecutar dentro de la carpeta `vc_new_controller`.

```bash
WORKSPACE=~/catkin_ws
cd $WORKSPACE/src/vc_new_controller
cp -r Files/mygroundplane/ ~/.gazebo/models/
cp Files/ground.world ${WORKSPACE}/src/rotors_simulator/rotors_gazebo/worlds/
cp Files/hummingbird.xacro ${WORKSPACE}/src/rotors_simulator/rotors_description/urdf/hummingbird.xacro
```

### Ejecución de Gazebo

Cada uno de los siguientes comandos deben ser ejecutados en una terminal diferente.

```bash
roslaunch vc_new_controller mav_hovering_example.launch mav_name:=hummingbird  world_name:=ground
```

### Ejecución de la parte visual

```bash
rosrun rqt_image_view rqt_image_view           
```

### Ejecución del control

```bash
rosrun vc_new_controller vc_new_controller
```


<!-- 1. Se ejecuta vc_new_controller.launch
2. Este mismo ejecuta de primeras: chaumette.cpp (línea 159)
3. Este ejecuta internamente a compute_descriptors.cpp (línea 14)
4. Después camera_norm.cpp (línea 27)
5. interaction_Mat.cpp (Línea 37)
6. Moore_Penrose_PInv.cpp (Línea 42)
7. Se actualiza los valores de velocidad, se integran y se publican en el dron
control uav ros robots formation formation-control dron cube-formation -->
