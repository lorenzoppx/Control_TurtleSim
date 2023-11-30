# Control_TurtleSim
This code is used to control the turtle simulator for go to goal. ROS 2.
Para utilização copie a pasta 'py_pubsub' para 'ros2_ws/src'. Em seguida Direcione-se para o diretório 'ros2_wc/', e utilize os seguintes comandos:
``` shell
source install/setup.bash
colcon build --packages-select py_pubsub
```
Inicialização do turtlesim: 
``` shell
ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/LPPX
```
Inicialização do controle:
``` shell
ros2 run py_pubsub talker --ros-args -r __ns:=/LPPX
```
Após o comando acima tartaruga irá se mover para a posição default goal estabelecida, para definir uma nova posição goal, utilize o seguinte comando:
``` shell
ros2 topic pub /LPPX/goal turtlesim/msg/Pose "{x: 7.0, y: 7.0}"
```
Demonstração:
<img src="https://github.com/lorenzoppx/Control_TurtleSim/control.gif" width="300">
