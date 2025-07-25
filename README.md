these ROS2 packages are a iniital prototype for autonomous movement of the unitree GO2 quadruped robot.
REQUIREMENTS:
* on board dog computer (NVIDIA JETSON NANO)
* LIVOX MID 360 lidar
* ROS2 foxy (installed by default on the on board computer)

the model i used for testing is the EDU version (shipped with the requirements)

HOW TO USE
* clone this repo in your ROS2 src workspace
* `cd ..`
* ` colcon build ` <br> ` source install/setup.bash ` <br> ` ros2 go2_navigation mapping.launch.py rviz:=True `
