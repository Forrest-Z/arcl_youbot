# arcl_youbot

Test base_controller in gazebo

Command: 

1, roslaunch luh_youbot_gazebo youbot.launch  

2, cd catkin_youbot_ws/src/luh_youbot_controller/src/luh_youbot_controller/node  
   python controller_node.py  
   
3, rostopic pub -r 50 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

