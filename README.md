# arcl_youbot

## WARNING

To run youbot in real world, please see ManipulationServer.cpp line 130.

To run youbot in real world, please see luh_youbot_controller/cfg/cmodule_base_controller/params.yaml

The distance between pick and youbot, please see ManipulationServer.cpp line 144.

## INSTALL

1. `git clone https://github.com/wt160/arcl_youbot.git`

2. `cd arcl_youbot`

3. `bash install.sh`

4. For youbot only: `bash youbot_ros_variable.sh; cd; source .bashrc`


Test youbot_pick_demo in gazebo

Command: 

1. `roslaunch luh_youbot_gazebo youbot.launch`

2. `roslaunch luh_youbot_controller gazebo_controller.launch`

3. `rosrun youbot_grasp youbot_grasp_planning_service`

4. `rosrun arcl_youbot_application manipulation_server`

5. `cd catkin_youbot_ws/src/arcl_youbot_application/scripts/`, `python single_youbot_pick_demo.py`
 
Test base_controller in gazebo
`rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'`

## Dependency:
### Base Planner:
`pip install shapley`
`apt install ros-melodic-pybin11-catkin`
install https://github.com/sybrenstuvel/Python-RVO2


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
