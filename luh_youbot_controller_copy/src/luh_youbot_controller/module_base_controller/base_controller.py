#!/usr/bin/env python
import rospy 
import math
from geometry_msgs.msg import Twist 

POSITION = 0
VELOCITY = 1



class BaseController:
    def __init__(self, input_youbot_interface):
        self.youbot_interface_ = input_youbot_interface
        self.mode_ = POSITION
        self.max_velocity_x_ = rospy.get_param("module_base_controller/max_velocity_x", 0.2)
        self.max_velocity_y_ = rospy.get_param("module_base_controller/max_velocity_y", 0.2)
        self.max_velocity_theta_ = rospy.get_param("module_base_controller/max_velocity_y", 0.4)
        self.base_frequency = rospy.get_param('luh_youbot_controller/base_controller_frequency', 50.0)
        self.velocity_command_ = Twist()
        self.last_time_get_vel_cmd = 0.0
        self.velocity_command_.linear.x = 0 
        self.velocity_command_.linear.y = 0       
        self.velocity_command_.linear.z = 0       
        self.velocity_command_.angular.x = 0
        self.velocity_command_.angular.y = 0
        self.velocity_command_.angular.z = 0
        rospy.Subscriber("/cmd_vel", Twist, self.velocity_callback)

    def update(self):
        current_time = rospy.get_time()
        if current_time - self.last_time_get_vel_cmd > 1.0 / self.base_frequency:
            self.velocity_command_.linear.x = 0.0
            self.velocity_command_.linear.y = 0.0
            self.velocity_command_.linear.z = 0.0
            self.velocity_command_.angular.x = 0.0
            self.velocity_command_.angular.y = 0.0
            self.velocity_command_.angular.z = 0.0

        self.youbot_interface_.base_interface.set_velocity(self.velocity_command_)
        
    def velocity_callback(self, data):

        self.velocity_command_ = data
        self.last_time_get_vel_cmd = rospy.get_time()

        if self.mode_ == POSITION:
            print "Got velocity command. Preempting current action."
            #preempt()

        vx = self.velocity_command_.linear.x
        vy = self.velocity_command_.linear.y
        factor = math.sqrt(vx*vx / (self.max_velocity_x_ * self.max_velocity_x_) + vy*vy / (self.max_velocity_y_ * self.max_velocity_y_))

        if factor > 1:
            self.velocity_command_.linear.x = vx / factor
            self.velocity_command_.linear.y = vy / factor
        

        self.velocity_command_.angular.z = min(self.velocity_command_.angular.z, self.max_velocity_theta_)
        self.velocity_command_.angular.z = max(self.velocity_command_.angular.z, -self.max_velocity_theta_)

        self.mode_ = VELOCITY