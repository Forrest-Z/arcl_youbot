import rospy
import math
import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import arcl_youbot_msgs.msg._JointVector
import sensor_msgs.msg._JointState
# from luh_youbot_driver_api.base_interface import baseInterface

from tf.transformations import euler_from_quaternion

POSITION_ = 0
VELOCITY_ = 1
TORQUE_ = 2


class armInterface():
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.has_new_arm_command_ = False
        self.mode_ = POSITION_
        self.joint_state_ = sensor_msgs.msg.JointState()
        self.joint_names_ = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        del self.joint_state_.name[:]
        del self.joint_state_.position[:]
        del self.joint_state_.velocity[:]
        del self.joint_state_.effort[:]        
        for name in self.joint_names_:
            self.joint_state_.name.append(name)
            self.joint_state_.position.append(0)
            self.joint_state_.velocity.append(0)
            self.joint_state_.effort.append(0)



        
    def joint_state_callback(self, data):
        for i in range(len(data.name)):
            current_name = data.name[i]
            index = 0
            if current_name == "arm_joint_1":
                index = 0
            elif current_name == "arm_joint_2":
                index = 1
            elif current_name == "arm_joint_3":
                index = 2
            elif current_name == "arm_joint_4":
                index = 3
            elif current_name == "arm_joint_5":
                index = 4
            else:
                continue            
            self.joint_state_.position[index] = data.position[i]
            self.joint_state_.velocity[index] = data.velocity[i]
            self.joint_state_.effort[index] = data.effort[i]

    def init(self): 
        self.position_cmd_pub_ = rospy.Publisher("/gazebo/joint_position_command", luh_youbot_msgs.msg.JointVector, queue_size=10)
        self.velocity_cmd_pub_ = rospy.Publisher("/gazebo/joint_velocity_command", luh_youbot_msgs.msg.JointVector, queue_size=10)
        self.torque_cmd_pub_ = rospy.Publisher("/gazebo/joint_torque_command", luh_youbot_msgs.msg.JointVector, queue_size=10)
        rospy.Subscriber("gazebo/joint_states", sensor_msgs.msg.JointState, self.joint_state_callback)
        # rospy.Subscriber("odom_" + str(self.robot_id), nav_msgs.msg.Odometry, odom_callback)

    def read_state(self):
        pass
        # joint_position_.setValues(joint_state_.position)
        # joint_position_.addOffset()
        # joint_velocity_.setValues(joint_state_.velocity)
        # joint_torque_.setValues(joint_state_.effort)

    def set_joint_velocity(self, vel):
        self.mode_ = VELOCITY_
        self.velocity_command_ = vel
        self.has_new_arm_command_ = True

    def set_joint_position(self, pos):            
        self.mode_ = POSITION_
        self.position_command_ = pos
        self.has_new_arm_command_ = True

    def set_joint_torque(self, tor):
        self.mode_ = TORQUE_
        self.torque_command_ = tor
        self.has_new_arm_command_ = True 


# //########## SET POSITIONS #############################################################################################
# void YoubotArmInterface::setJointPositions(ykin::JointPosition positions)
# {
#     positions.subtractOffset();
#     mode_ = POSITION;
#     position_command_ = positions;
#     has_new_arm_command_ = true;
# }

# //########## SET VELOCITIES ############################################################################################
# void YoubotArmInterface::setJointVelocities(ykin::JointVelocity velocities)
# {
#     mode_ = VELOCITY;
#     velocity_command_ = velocities;
#     has_new_arm_command_ = true;
# }

# //########## SET TORQUES ###############################################################################################
# void YoubotArmInterface::setJointTorques(ykin::JointVector torques)
# {
#     mode_ = TORQUE;
#     torque_command_ = torques;
#     has_new_arm_command_ = true;
# }        




    def writeCommand(self):
        msg = arcl_youbot_msgs.msg.JointVector() 
        if self.has_new_arm_command_ == True:
            if self.mode_ == POSITION_:
                msg.q1 = self.position_command_[0]
                msg.q2 = self.position_command_[1]
                msg.q3 = self.position_command_[2]
                msg.q4 = self.position_command_[3]
                msg.q5 = self.position_command_[4]
                self.position_cmd_pub_.publish(msg)
            elif self.mode_ == VELOCITY_:
                msg.q1 = self.velocity_command_[0]
                msg.q2 = self.velocity_command_[1]
                msg.q3 = self.velocity_command_[2]
                msg.q4 = self.velocity_command_[3]
                msg.q5 = self.velocity_command_[4]
                self.velocity_cmd_pub_.publish(msg)
            else:
                msg.q1 = self.torque_command_[0]
                msg.q2 = self.torque_command_[1]
                msg.q3 = self.torque_command_[2]
                msg.q4 = self.torque_command_[3]
                msg.q5 = self.torque_command_[4]
                self.torque_cmd_pub_.publish(msg)

            self.has_new_arm_command_ = False
    
        if has_new_gripper_command_ == True:
            self.has_new_gripper_command_ = False
