cd franka-interface
source catkin_ws/devel/setup.bash

echo Launching franka_interface
./build/franka_interface --logdir=/external_logs/logs &> /external_logs/franka_interface.log &

sleep 2

echo Launching franka_ros_interface
roslaunch franka_ros_interface franka_ros_interface.launch &> /external_logs/franka_ros_interface.log &

echo Spinning...
sleep infinity
