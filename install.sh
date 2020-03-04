#!/usr/bin/env sh

sudo apt install ros-melodic-pybind11-catkin libfcl-dev libfcl0.5 liburdfdom-tools ros-melodic-ompl ros-melodic-geometric-shapes

catkin_make youbot_grasp_gencpp  arcl_youbot_msgs_gencpp driver_base_gencpp driver_base_genpy

catkin_make

