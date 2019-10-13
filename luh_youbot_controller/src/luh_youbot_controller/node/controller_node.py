#!/usr/bin/env python
import rospy
import luh_youbot_gazebo
import luh_youbot_gazebo.arm_gazebo_interface as arm_gazebo_interface
import luh_youbot_gazebo.youbot_gazebo_interface as youbot_gazebo_interface
import luh_youbot_controller.module_base_controller.base_controller as base_controller

class controller_node():
    
    def __init__(self, robot_id):
        self.base_frequency = rospy.get_param('luh_youbot_controller/base_controller_frequency', 50.0)
        self.arm_frequency = rospy.get_param('luh_youbot_controller/arm_controller_frequency', 200.0)
        self.is_using_robot = rospy.get_param('arcl_youbot_controller/use_youbot', False)
        self.is_using_gazebo = rospy.get_param('arcl_youbot_controller/use_gazebo', True)
        self.robot_id = robot_id


        rospy.Timer(rospy.Duration(1.0 / self.base_frequency), self.base_timer_callback)
        rospy.Timer(rospy.Duration(1.0 / self.arm_frequency), self.arm_timer_callback)
        
        if self.is_using_robot == True:
            self.youbot_ = youbot_interface.YoubotInterface(self.robot_id)
        elif self.is_using_gazebo == True:
            self.youbot_ = youbot_gazebo_interface.YoubotGazeboInterface(self.robot_id) 

        self.base_controller_module = base_controller.BaseController(self.youbot_)



    def base_timer_callback(self, event=None):
       # print "enter t_callback in" + rospy.get_name() 
        self.youbot_.base_interface.read_state()

        self.base_controller_module.update()

        self.youbot_.base_interface.write_command()

        self.youbot_.base_interface.publish_message()

    def arm_timer_callback(self, event=None):
        pass
        # self.youbot_.arm_interface.read_state()

        # self.arm_controller_module.update()

        # self.youbot_.arm_interface.write_command()

        # self.youbot_.arm_interface.publish_message()


if __name__ == "__main__":
    rospy.init_node('controller_node')
    node = controller_node(0)
    #node_1 = controller_node(1)
    rospy.spin()