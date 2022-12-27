# Image Based Visual Servoing - Based on a paper

Este repositorio fue hecho para implementar una modificación al algoritmo descrito en el paper "Image-based estimation, planning, and control for high-speed flying through multiple openings" de Guo, Dejun and Leang.

## Requerimientos

- Python 3
- OpenCV 4+
- ROS Noetic
- Gazebo - ROS Noetic

## Instalación de ROS Noetic (Ubuntu 20.04)

### Instalación con dos líneas

```bash
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
```

### Comprobación de los paquetes instalados

```bash
sudo apt-get install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink protobuf-compiler libgoogle-glog-dev ros-noetic-control-toolbox python3-wstool python3-catkin-tools
```

### Preparación de Python 3

#### Preparación de PIP - Python 3.7+

Si no se tiene instalado pip en _Python 3.7+_:

```bash
sudo curl https://bootstrap.pypa.io/get-pip.py | sudo python3
sudo curl https://bootstrap.pypa.io/get-pip.py | python3
```

#### Añadir a variables de entorno

- Bash

  ```bash
  echo 'export PATH=$PATH:"$HOME/.local/bin"' >> ~/.bashrc
  source ~/.bashrc
  ```

- Zsh

  ```zsh
  echo 'export PATH=$PATH:"$HOME/.local/bin"' >> ~/.zshrc
  source ~/.zshrc
  ```

### Instalación de paquetes de ROS adicionales en Python 3

```bash
sudo python3 -m pip install -U rosdep catkin_pkg future
python3 -m pip install -U rosdep catkin_pkg future empy defusedxml numpy matplotlib imageio opencv-python
```

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

## Construcción del espacio

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

- Bash

  ```bash
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

- Zsh

  ```bash
  echo "source ~/catkin_ws/devel/setup.zsh" >> ~/.zshrc
  source ~/.zshrc
  ```

## Ejecución

Antes de la ejecución, es necesario los archivos adjuntos en la carpeta _Files_. Estos archivos modificarán configuraciones predeterminadas de RotorS por lo cual _se pide que se haga una copia de seguridad de los archivos originales_.

- [iris.xacro]
  
  Este archivo modifica el dron de tal forma que se añadan dos cámaras, una frontal y otra inferior. Estas cámaras son necesarias para el algoritmo de control visual propuesto en este trabajo.

  Se debe copiar en la ruta _~/catkin_ws/src/rotors_simulator/rotors_description/urdf/_

  ```bash
  cd ~/catkin_ws/src/vc_new_controller/Files
  cp iris.xacro ~/catkin_ws/src/rotors_simulator/rotors_description/urdf/iris.xacro
  ```

- [hummingbird.xacro]

  Este archivo modifica el dron de tal forma que se añade una cámara inferior. Esta cámara es necesaria para la ejecución del algoritmo de control visual propuesto en este trabajo.

  Se debe copiar en la ruta _~/catkin_ws/src/rotors_simulator/rotors_description/urdf/_

  ```bash
  cd ~/catkin_ws/src/vc_new_controller/Files
  cp hummingbird.xacro ~/catkin_ws/src/rotors_simulator/rotors_description/urdf/hummingbird.xacro
  ```

- [lee_controller_iris.yaml]

  Este archivo modifica el controlador de bajo nivel del dron. Se realiza esto, debido a que el dron inicialmente posee problemas para estabilizarse de forma correcta. Las ganancias exhibidas pueden ser modificadas o sintonizadas para obtener un mejor comportamiento.

  Se debe copiar en la ruta _~/catkin_ws/src/rotors_simulator/rotors_gazebo/resource/_

  ```bash
  cd ~/catkin_ws/src/vc_new_controller/Files
  cp lee_controller_iris.yaml ~/catkin_ws/src/rotors_simulator/rotors_gazebo/resource/lee_controller_iris.yaml
  ```

## Ejecución de Gazebo

El control propuesto en este trabajo se ejecuta en el simulador Gazebo. Una vez instalados todos los requerimientos, se pueden ejecutar ambos controles. Dentro de la carpeta _config_ dentro del paquete se encuentras los archivos _yaml_ que contienen las configuraciones de ambos drones. Estos archivos se pueden modificar para cambiar parámetros como parámetros intrínsecos de las cámaras, ganancias del controlador, tolerancias de error, etc.

Cabe mencionar que cada uno de estas líneas de código deben ser ejecutadas en una terminal diferente.

- Control para el dron Hummingbird

  - Ejecución de Gazebo

    ```bash
    roslaunch vc_new_controller hummingbird.launch
    ```
  
  - Posicionamiento correcto del dron
  
    ```bash
    rosrun placing_iris placing_iris hummingbird %X %Y %Z %YAW 
    ```

    en donde %X, %Y, %Z y %YAW son los valores de posición y orientación del dron en el espacio. Estos valores deben ser modificados para que el dron se posicione correctamente.

    Y, finalmente, se ejecuta el control visual.

    ```bash
    roslaunch vc_new_controller ibvs_hummingbird.launch
    ```

- Control para el dron Iris

  - Ejecución de Gazebo

    ```bash
    roslaunch vc_new_controller iris.launch
    ```

  - Posicionamiento correcto del dron

    ```bash
    rosrun placing_iris placing_iris iris %X %Y %Z %YAW 
    ```

    en donde %X, %Y, %Z y %YAW son los valores de posición y orientación del dron en el espacio. Estos valores deben ser modificados para que el dron se posicione correctamente.

    Y, finalmente, se ejecuta el control visual.

    ```bash
    roslaunch vc_new_controller ibvs_iris.launch
    ```

  Cabe mencionar que para este último dron, el posible como ya se mencionó ambas cámaras, esto se puede cambiar en el archivo de configuración _params_hummingbird.yaml_.

### Ejecución de la parte visual

Si se quiere ver en tiempo real la cámaraa de cualquier dron, se puede ejecutar el siguiente comando:

```bash
rosrun rqt_image_view rqt_image_view
```