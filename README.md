# arcl_youbot

Test base_controller in gazebo

Command: 

1, roslaunch luh_youbot_gazebo youbot.launch  

2, cd catkin_youbot_ws/src/luh_youbot_controller/src/luh_youbot_controller/node  
   python controller_node.py  
   
3, rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'


# Tweaks for Luh-youbot-os
## YouBot - Robot - Lubuntu 18.04
### catkin_make
1. put CATKIN_IGNORE `under luh_youbot_vrep_api/`

2. `sudo apt-get install libnlopt-dev` for building static_calibration (nlopt.hpp)

3. change gksudo to sudo under `luh_youbot_controller/CMakeLists.txt`

### roslaunch
`sudo ldconfig /opt/ros/melodic/lib`

`ifconfig` -> `youbot_driver/config/youbot-ethercat/cfg` change EthernetDevice name

`luh_youbot_controller/launch` change launch driver node param

`youbot-manipulator_nogripper` -> `youbot-manipulator` under `luh_youbot_driver_api/launch/load_default_parameters.launch`

## YouBot - Desktop - Ubuntu 18.04
### catkin_make
1. put CATKIN_IGNORE `under luh_youbot_vrep_api/`

2. `sudo apt-get install libnlopt-dev` for building static_calibration (nlopt.hpp)

3. change gksudo to sudo under `luh_youbot_controller/CMakeLists.txt`
