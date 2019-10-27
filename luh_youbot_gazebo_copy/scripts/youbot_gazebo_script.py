#! /usr/bin/env python
import rospy
import math
import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import luh_youbot_gazebo.base_gazebo_interface as base_gazebo_interface

if __name__ == '__main__':
    rospy.init_node("base_gazebo_interface", anonymous=True)
    try:
        r = rospy.Rate(0.2) # 10hz
        while not rospy.is_shutdown():
            interface_ = base_gazebo_interface.baseGazeboInterface(0)
            vel_cmd = geometry_msgs.msg.Twist()
            vel_cmd.linear.x = 1
            vel_cmd.linear.y = 0
            vel_cmd.angular.z = 0
            interface_.init()
            interface_.setVelocity(vel_cmd)
            interface_.writeCommand()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
