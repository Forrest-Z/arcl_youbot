/* *****************************************************************
 *
 * luh_youbot_controller
 *
 * Copyright (c) 2015,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 * Author: Simon Aden (info@luhbots.de)
 ******************************************************************/


#ifndef LUH_YOUBOT_CONTROLLER_MODULE_INTERPOLATION_H
#define LUH_YOUBOT_CONTROLLER_MODULE_INTERPOLATION_H

#include "ros/ros.h"
//#include "controller/pid.hpp"
#include "luh_youbot_controller/torque_controller/torque_controller.hpp"
#include "luh_youbot_controller/torque_controller/youbot_dynamics.hpp"
#include "../module_interpolation/ramp_generator.h"
#include "../module_base_class/controller_module.h"
#include "arcl_youbot_msgs/JointVector.h"
#include "arcl_youbot_msgs/CartesianVector.h"
#include "arcl_youbot_msgs/CylindricVector.h"
#include "actionlib/server/simple_action_server.h"
#include "arcl_youbot_msgs/MoveToCartesianPoseAction.h"
#include "arcl_youbot_msgs/MoveToCylindricPoseAction.h"
#include "arcl_youbot_msgs/MoveToJointPoseAction.h"
#include "arcl_youbot_msgs/MoveToNamedPoseAction.h"
#include "control_toolbox/pid.h"
#include <arcl_youbot_msgs/SetCartesianVelocity.h>
#include <arcl_youbot_msgs/SetCylindricVelocity.h>
#include <arcl_youbot_msgs/SetJointVelocity.h>


typedef actionlib::SimpleActionServer<arcl_youbot_msgs::MoveToCartesianPoseAction> CartesianServer;
typedef actionlib::SimpleActionServer<arcl_youbot_msgs::MoveToCylindricPoseAction> CylindricServer;
typedef actionlib::SimpleActionServer<arcl_youbot_msgs::MoveToJointPoseAction> JointServer;
typedef actionlib::SimpleActionServer<arcl_youbot_msgs::MoveToNamedPoseAction> NamedServer;

class ModuleInterpolation : public ControllerModule
{
public:
    enum Coordinates
    {
        CARTESIAN,
        CYLINDRIC,
        JOINTSPACE
    };

    enum CommandMode
    {
        POSITION,
        VELOCITY,
        TORQUE
    };

    ModuleInterpolation();
    ~ModuleInterpolation();

    void init();
    void activate();
    void deactivate();
    void update();
    void emergencyStop();

protected:

    enum InternalState
    {
        BLOCKED,
        INACTIVE,
        ACTIVE_VELOCITY,
        ACTIVE_POSITION
    };

    ros::Subscriber cyl_velocity_subscriber_;
    ros::Subscriber cart_velocity_subscriber_;
    ros::Subscriber jnt_velocity_subscriber_;

    ros::Publisher cyl_position_command_pub_;
    ros::Publisher cyl_position_state_pub_;

    CartesianServer* cartesian_pose_server_;
    CylindricServer* cylindric_pose_server_;
    JointServer* joint_pose_server_;
    NamedServer* named_pose_server_;

    ros::ServiceServer set_cart_vel_server_;
    ros::ServiceServer set_cyl_vel_server_;
    ros::ServiceServer set_jnt_vel_server_;

    RampGenerator cart_ramp_generator_;
    RampGenerator cyl_ramp_generator_;
    RampGenerator jnt_ramp_generator_;

    arcl_youbot_kinematics::CylindricVelocity max_cyl_velocity_;
    arcl_youbot_kinematics::CylindricAcceleration max_cyl_acceleration_;

    arcl_youbot_kinematics::CartesianVelocity max_cart_velocity_;
    arcl_youbot_kinematics::CartesianAcceleration max_cart_acceleration_;

    arcl_youbot_kinematics::JointVelocity max_joint_velocity_;
    arcl_youbot_kinematics::JointAcceleration max_joint_acceleration_;

    arcl_youbot_kinematics::JointPosition joint_state_position_;
    arcl_youbot_kinematics::JointVelocity joint_state_velocity_;

    double timer_frequency_;

    bool got_jointstate_;
    bool got_command_;
    bool moving_;
    Coordinates coordinate_mode_;
    CommandMode command_mode_;

    double t_now_;

    std::vector<control_toolbox::Pid> controller_pid_;
    nAxesControllerTorque* torque_controller_;
    YoubotDynamics* youbot_dynamics_;

    arcl_youbot_kinematics::JointVelocity last_velocity_;
    arcl_youbot_kinematics::GenericVector goal_position_;

    double timeout_threshold_;
    double timeout_time_;
//    int stop_count_;
//    int max_stop_count_;

//    bool deactivated_;
//    bool preempted_;

    InternalState internal_state_;

    bool mode_changed_;

    bool controller_test_mode_;

    bool goal_pose_is_named_;

    void cylVelocityCallback(const arcl_youbot_msgs::CylindricVector::ConstPtr &msg);
    void cartVelocityCallback(const arcl_youbot_msgs::CartesianVector::ConstPtr &msg);
    void jntVelocityCallback(const arcl_youbot_msgs::JointVector::ConstPtr &msg);

    void cartesianPoseCallback();
    void cylindricPoseCallback();
    void jointPoseCallback();
    void namedPoseCallback();
    void preemptCallback();

    double getSafetyFactor(arcl_youbot_kinematics::JointVelocity &jnt_velocity);
    void stop();
    void setVelocityCommand(const arcl_youbot_kinematics::GenericVector &velocity);
    void setPositionCommand(const arcl_youbot_kinematics::GenericVector &position, bool is_absolute);

    bool checkStateVelocity();
    bool checkStatePosition();

    bool setCartesianVelocityCallback(arcl_youbot_msgs::SetCartesianVelocity::Request &req,
                                      arcl_youbot_msgs::SetCartesianVelocity::Response &res);
    bool setCylindricVelocityCallback(arcl_youbot_msgs::SetCylindricVelocity::Request &req,
                                      arcl_youbot_msgs::SetCylindricVelocity::Response &res);
    bool setJointVelocityCallback(arcl_youbot_msgs::SetJointVelocity::Request &req,
                                  arcl_youbot_msgs::SetJointVelocity::Response &res);
};

#endif // MODULE_INTERPOLATION_H
