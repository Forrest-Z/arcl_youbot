/* *****************************************************************
 *
 * luh_youbot_driver_api
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
 *
 *
 *
 * === Based on youbot_oodl with original license header: ===
 *
 * Copyright (c) 2011
 * Locomotec
 *
 * Author:
 * Sebastian Blumenthal
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#include "luh_youbot_gazebo/youbot_interface.h"

//########## CONSTRUCTOR ###############################################################################################
YoubotGazeboInterface::YoubotGazeboInterface(ros::NodeHandle &node):
    YoubotInterface(node)
{
}

//########## DESTRUCTOR ################################################################################################
YoubotGazeboInterface::~YoubotGazeboInterface()
{

}

//########## INITIALISE ################################################################################################
void YoubotGazeboInterface::initialise(bool use_standard_gripper, bool use_luh_gripper_v3_)
{
    // === GET PARAMETERS ===
    ros::param::param("youBotHasBase", config_.has_base, true);
    bool youbot_has_arms;
    ros::param::param("youBotHasArms", youbot_has_arms, true);
//    node.param("youBotDriverCycleFrequencyInHz", config_.frequency, 200.0);
    config_.config_path = ros::package::getPath("youbot_driver");
    config_.config_path.append("/config");
    ros::param::param("youBotConfigurationFilePath", config_.config_path, config_.config_path);

    // === CREATE ARMS ===
    if(youbot_has_arms)
    {
        int i = 1;
        std::stringstream ss;
        ss << "youBotArmName" << i; // youBotArmName1 is first checked param... then youBotArmName2, etc.
        while (ros::param::has(ss.str()))
        {
            std::string arm_name;
            ros::param::get(ss.str(), arm_name);

            arms_.push_back(new YoubotArmGazeboInterface(arm_name, config_));

            ss.str("");
            ss << "youBotArmName" <<  (++i);
        }
    }

    // === CREATE BASE ===
    if(config_.has_base)
    {
        std::string base_name;
        ros::param::param<std::string>("youBotBaseName", base_name, "youbot-base");
        base_ = new YoubotBaseGazeboInterface(base_name, config_);
    }

    if(config_.has_base)
    {
        base_->initialise();
        ROS_ASSERT(base_->isInitialised());
    }

    for(uint i=0; i<arms_.size(); i++)
    {
        arms_[i]->initialise(use_standard_gripper, false);
        ROS_ASSERT(arms_[i]->isInitialised());
    }
}

//########## READ STATE ################################################################################################
void YoubotGazeboInterface::readState()
{
    // read base sensors
    if(config_.has_base)
        base_->readState();

    // read arm sensors
    for(uint i=0; i<arms_.size(); i++)
    {
        arms_[i]->readState();
    }
}

//########## WRITE COMMANDS ############################################################################################
void YoubotGazeboInterface::writeCommands()
{
    // set base commands
    if(config_.has_base)
        base_->writeCommands();

    // set arm commands
    for(uint i=0; i<arms_.size(); i++)
    {
        arms_[i]->writeCommands();
    }
}

//########## PUBLISH MESSAGES ##########################################################################################
void YoubotGazeboInterface::publishMessages()
{
    if(config_.has_base)
        base_->publishMessages();

    // set arm commands
    for(uint i=0; i<arms_.size(); i++)
    {
        arms_[i]->publishMessages();
    }
}

//########## UPDATE ####################################################################################################
void YoubotGazeboInterface::update()
{
    base_->updateController();
}

//########## STOP ######################################################################################################
void YoubotGazeboInterface::stop()
{
    if(config_.has_base)
        base_->stop();

    delete base_;

    for(uint i=0; i<arms_.size(); i++)
    {
        arms_[i]->stop();
    }

    arms_.clear();
    config_.has_base = false;
    config_.num_arms = 0;
}
