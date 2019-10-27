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


#ifndef LUH_YOUBOT_CONTROLLER_MODULE_GRAVITY_COMPENSATION_H
#define LUH_YOUBOT_CONTROLLER_MODULE_GRAVITY_COMPENSATION_H

#include "../module_base_class/controller_module.h"
#include <arcl_youbot_msgs/GetLoad.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <arcl_youbot_kinematics/arm_dynamics.h>
#include <deque>
#include <arcl_youbot_msgs/ForceFitAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<arcl_youbot_msgs::ForceFitAction> ForceFitServer;

class ModuleGravityCompensation : public ControllerModule
{
public:
    ModuleGravityCompensation();
    ~ModuleGravityCompensation();

    void init();
    void activate();
    void deactivate();
    void update();
    void emergencyStop();

protected:

    arcl_youbot_kinematics::StaticParameters params_;

    ForceFitServer* force_fit_server_;

    bool is_active_;
    bool do_compensation_;

    ros::ServiceServer compensate_server_;
    ros::ServiceServer deactivate_server_;
    ros::ServiceServer get_load_server_;

    arcl_youbot_kinematics::YoubotArmDynamics dynamics_;

    int buffer_size_;

    // force fitting
    double force_;
    double displacement_;
    double start_r_;
    double start_z_;

    std::deque<arcl_youbot_kinematics::JointPosition> position_buffer_;
    std::deque<arcl_youbot_kinematics::JointEffort> torque_buffer_;

    bool compensateCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool getLoadCallback(arcl_youbot_msgs::GetLoad::Request &req, arcl_youbot_msgs::GetLoad::Response &res);
    bool deactivateCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    void forceFitCallback();
    void preemptCallback();

    arcl_youbot_kinematics::JointEffort getEffortFromForce(const arcl_youbot_kinematics::JointPosition &pos);
    bool checkGoal(const arcl_youbot_kinematics::JointPosition &pos);
};

#endif // MODULE_GRAVITY_COMPENSATION_H
