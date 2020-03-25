# Psoc con Ros

Demo Video: https://youtu.be/beOhI5lUNJg

Se requiere la previa instalacion de Ros en un sistema operativo Ubuntu.
Enseguida instalar el paquete "ros-spine-control" por medio de catkin.


Obteniendo entorno de trabajo (para que ros encuentre los builds)
Esto se debe ejecutar cada nueva terminar que correra ROSRUN
source /home/ros/catkin_ws/devel/setup.bash


Enseguida abriremos una terminal, esta inicia el servicio de ROS:
Terminal 1
 roscore

Abrimos otra terminal y corremos el recibidor controlador de comunicacion (envios):
Terminal 2 
 rosrun spine_controller serial_rx_echo.py /dev/ttyACM0

Abrimos una ultima terminal, esta imprime el mensaje enviado desde la psoc:
Terminal 3
 rosrun spine_controller serial_tx_cmdline.py /dev/ttyACM0

