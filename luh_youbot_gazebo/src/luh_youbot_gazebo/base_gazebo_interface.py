#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import math
import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
from luh_youbot_driver_api.base_interface import baseInterface

from tf.transformations import euler_from_quaternion
class baseGazeboInterface(baseInterface):
    def __init__(self, robot_id):
        self.robot_id = robot_id

    def odom_callback(data):
        self.odom_msg_ = data

    def init(self): 
        self.cmd_vel_pub_ = rospy.Publisher("gazebo/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)
        # rospy.Subscriber("odom_" + str(self.robot_id), nav_msgs.msg.Odometry, odom_callback)

    def readState(self):
        self.pose2d = geometry_msgs.msg.Pose2D()
        self.pose2d.x = self.odom_msg_.pose.pose.position.x
        self.pose2d.y = self.odom_msg_.pose.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion(self.odom_msg_.pose.pose.orientation)
        if yaw > math.pi:
            yaw -= 2 * math.pi
        elif yaw < - math.pi:
            yaw += 2 * math.pi
        self.pose2d.theta = yaw
        
    def writeCommand(self):
        self.cmd_vel_pub_.publish(self.velocity_command_)

    def setVelocity(self, vel):
        self.velocity_command_ = vel



if __name__ == '__main__':
    rospy.init_node("base_gazebo_interface", anonymous=True)
    try:
        r = rospy.Rate(0.2) # 10hz
        while not rospy.is_shutdown():
            interface_ = baseGazeboInterface(0)
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
