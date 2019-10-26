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


#include "luh_youbot_controller/module_base_controller/base_controller.h"


//########## CONSTRUCTOR ###############################################################################################
ModuleBaseController::ModuleBaseController(): ControllerModule(),
    align_base_server_(NULL),
    move_base_server_(NULL),
    approach_server_(NULL)
{    
}

//########## DESTRUCTOR ################################################################################################
ModuleBaseController::~ModuleBaseController()
{
    delete align_base_server_;
    delete move_base_server_;
    delete approach_server_;
}

//########## INITIALIZATION ############################################################################################
void ModuleBaseController::init()
{
    ROS_INFO("Initialising Base Controller Module...");

    status_ = STATUS_IDLE;

    // === PARAMETERS ===
    activated_ = true;
    mode_ = IDLE;

	has_last_diff_ = false;

    node_->param("module_base_controller/max_velocity_x", max_velocity_x_, 0.1);
    node_->param("module_base_controller/max_velocity_y", max_velocity_y_, 0.1);
    node_->param("module_base_controller/max_velocity_theta", max_velocity_theta_, 0.1);

    node_->param("module_base_controller/velocity_p_factor_x", velocity_p_factor_x_, 2.0);
    node_->param("module_base_controller/velocity_p_factor_y", velocity_p_factor_y_, 2.0);
    node_->param("module_base_controller/velocity_p_factor_theta", velocity_p_factor_theta_, 0.7);

    node_->param("module_base_controller/velocity_i_factor_x", velocity_i_factor_x_, 0.1);
    node_->param("module_base_controller/velocity_i_factor_y", velocity_i_factor_y_, 0.1);
    node_->param("module_base_controller/velocity_i_factor_theta", velocity_i_factor_theta_, 0.1);

    node_->param("module_base_controller/velocity_d_factor_x", velocity_d_factor_x_, 0.1);
    node_->param("module_base_controller/velocity_d_factor_y", velocity_d_factor_y_, 0.1);
    node_->param("module_base_controller/velocity_d_factor_theta", velocity_d_factor_theta_, 0.1);

    node_->param("module_base_controller/position_tolerance_x", position_tolerance_x_, 0.01);
    node_->param("module_base_controller/position_tolerance_y", position_tolerance_y_, 0.01);
    node_->param("module_base_controller/position_tolerance_theta", position_tolerance_theta_, 0.01);

    node_->param("module_base_controller/move_lookahead", move_lookahead_, 0.5);


    fast_mode_ = false;
    node_->param("module_base_controller/goal_reached_threshold_x", goal_reached_threshold_x_, 0.01);
    node_->param("module_base_controller/goal_reached_threshold_y", goal_reached_threshold_y_, 0.01);
    node_->param("module_base_controller/goal_reached_threshold_theta", goal_reached_threshold_theta_, 0.01);

//    node_->param("module_base_controller/min_dist_x", min_dist_x_, 0.061);
//    node_->param("module_base_controller/min_dist_y", min_dist_y_, 0.185);
    node_->param("module_base_controller/velocity_command_timeout", velocity_command_timeout_, 1.0);
    node_->param("module_base_controller/memory_size", memory_size_, 5);
    node_->param("module_base_controller/align_stop_timeout", align_stop_timeout_, 1.0);
    node_->param("module_base_controller/align_fail_timeout", align_fail_timeout_, 1.0);
    node_->param("module_base_controller/align_topic", align_topic_, std::string(""));

	ROS_WARN("Velocity threshods: [%f, %f, %f]", max_velocity_x_, max_velocity_y_, max_velocity_theta_);
	ROS_WARN("Position tolerances: [%f, %f, %f]", position_tolerance_x_, position_tolerance_y_, position_tolerance_theta_);


    // === SUBSCRIBERS ===
//    laser_listener_ = node_->subscribe("/scan", 10, &ModuleBaseController::laserCallback, this);
    velocity_subscriber_ = node_->subscribe("robot/cmd_vel", 10, &ModuleBaseController::velocityCallback, this);
    if(!align_topic_.empty())
        pose_subscriber_ = node_->subscribe(align_topic_, 1, &ModuleBaseController::poseCallback, this);
    else
        ROS_INFO("No pose topic for alignment action specified.");

    //laser_subscriber_ = node_->subscribe("laser_watchdog/distances", 1, &ModuleBaseController::laserCallback, this);

    // === ACTION SERVERS ===
    move_base_server_ = new MoveBaseServer(*node_, "youbot_base/move", false);
    move_base_server_->registerGoalCallback(boost::bind(&ModuleBaseController::moveBaseCallback, this));
    move_base_server_->registerPreemptCallback(boost::bind(&ModuleBaseController::preemptCallback, this));
    move_base_server_->start();   
    align_base_server_ = new AlignBaseServer(*node_, "youbot_base/align", false);
    align_base_server_->registerGoalCallback(boost::bind(&ModuleBaseController::alignBaseCallback, this));
    align_base_server_->registerPreemptCallback(boost::bind(&ModuleBaseController::preemptCallback, this));
    align_base_server_->start();
    approach_server_ = new ApproachServer(*node_, "youbot_base/approach", false);
    approach_server_->registerGoalCallback(boost::bind(&ModuleBaseController::approachCallback, this));
    approach_server_->registerPreemptCallback(boost::bind(&ModuleBaseController::preemptCallback, this));
    approach_server_->start();

    // === SERVICE SERVER ===
    stop_server_ = node_->advertiseService("youbot_base/stop", &ModuleBaseController::stopCallback, this);
    get_pose_server_ = node_->advertiseService("youbot_base/get_pose", &ModuleBaseController::getPoseCallback, this);

    // === PUBLISHER ===
    error_publisher_ = node_->advertise<geometry_msgs::Pose2D>("youbot_base/position_error", 100);


    ROS_INFO("Base Controller Module initialised.");
}

//########## UPDATE ####################################################################################################
void ModuleBaseController::update()
{
    if(!activated_)
        return;

    updateMovedDistance();

    if(mode_ == POSITION)
        updatePositionMode();

    else if(mode_ == VELOCITY)
        updateVelocityMode();

    else if(mode_ == ALIGN)
        updateAlignMode();

    else if(mode_ == APPROACH)
        updateApproachMode();
    else
        return;

    // publish velocity
    youbot_->base()->setVelocity(velocity_command_);
    
}

//########## UPDATE MOVED DISTANCE #####################################################################################
void ModuleBaseController::updateMovedDistance()
{
    geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();
    current_pose_.x = base_pose.x;
    current_pose_.y = base_pose.y;
    current_pose_.theta = base_pose.theta;

    // get difference between current pose and goal pose
    double diff_x = current_pose_.x;// - start_pose_.x;
    double diff_y = current_pose_.y;// - start_pose_.y;
    double diff_theta = current_pose_.theta;// - start_pose_.theta;
    if(diff_theta > M_PI)
        diff_theta -= 2*M_PI;
    else if(diff_theta < -M_PI)
        diff_theta += 2*M_PI;

    //double diff_x_t = diff_x * cos(base_pose.theta) + diff_y * sin(base_pose.theta + start_pose_.theta);
    //double diff_y_t = diff_y * cos(base_pose.theta + start_pose_.theta) - diff_x * sin(base_pose.theta + start_pose_.theta);

    moved_distance_.x = diff_x;
    moved_distance_.y = diff_y;
    moved_distance_.theta = diff_theta;

    //ROS_INFO_STREAM("diff:   "<<diff_x<<"|||"<<diff_y<<"|||"<<diff_theta);
    
    //ROS_INFO_STREAM("current_pose_"<<current_pose_.x<<"|||"<<current_pose_.y<<"|||"<<current_pose_.theta);

}

double ModuleBaseController::poseDistance(BasePose &pose1, BasePose &pose2){
	return sqrt((pose1.x - pose2.x)*(pose1.x - pose2.x) + (pose1.y - pose2.y)*(pose1.y - pose2.y));
}

//########## UPDATE POSITION MODE ######################################################################################
void ModuleBaseController::updatePositionMode()
{

	/* Check distance to goal and update current temporary goal as needed  */
	double curX, curY, curTheta;

    geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();
    current_pose_.x = base_pose.x;
    current_pose_.y = base_pose.y;
    current_pose_.theta = base_pose.theta;

	// Check how far from goal and temp goal
	double distanceToGoal = poseDistance(current_pose_, goal_pose_);
	double distanceToCurrentGoal = poseDistance(current_pose_, current_goal_pose_);

	// ROS_INFO("Entering position update routine... ");

	// If we are far away from goal, may need to update current goal
 	if(distanceToGoal > move_lookahead_ + 0.025){
		// We update if current pose is not far enough from current goal
		// ROS_INFO("Distance to current goal: %f", distanceToGoal);
		if(distanceToCurrentGoal < move_lookahead_){
			// Move current goal pose by a bit 
			// ROS_INFO("Distance to current goal: %f", distanceToCurrentGoal);
			while(poseDistance(current_pose_, current_goal_pose_) <  move_lookahead_){
				current_goal_pose_.x +=  goal_cos_angle_ * 0.01;
				current_goal_pose_.y +=  goal_sin_angle_ * 0.01;
				double distanceFromStart = poseDistance(current_goal_pose_, start_pose_);
				double totalDistance = poseDistance(start_pose_, goal_pose_);

				current_goal_pose_.x = start_pose_.x + 
					(goal_pose_.x - start_pose_.x)*distanceFromStart/totalDistance;
				current_goal_pose_.y = start_pose_.y + 
					(goal_pose_.y - start_pose_.y)*distanceFromStart/totalDistance;
				current_goal_pose_.theta = start_pose_.theta + 
					(goal_pose_.theta - start_pose_.theta)*distanceFromStart/totalDistance;

//				ROS_INFO("Current pose: [%f, %f, %f]", current_pose_.x, current_pose_.y, current_pose_.theta);
//				ROS_INFO("Current goal: [%f, %f, %f]", current_goal_pose_.x, current_goal_pose_.y, current_goal_pose_.theta);
//				ROS_INFO("Distance to current goal: %f, threshold: %f", 
//						poseDistance(current_pose_, current_goal_pose_), move_lookahead_);

			}
		}
	}
	else{
		current_goal_pose_ = goal_pose_;
	}


	// Compute differences in x, y, and theta 

    double diff_x_t = (current_goal_pose_.x - current_pose_.x) * cos(current_pose_.theta) 
					  + (current_goal_pose_.y - current_pose_.y) * sin(current_pose_.theta);
    double diff_y_t = - (current_goal_pose_.x - current_pose_.x) * (sin(current_pose_.theta)) 
					  + (current_goal_pose_.y - current_pose_.y) * cos(current_pose_.theta);
    double diff_theta = current_goal_pose_.theta - current_pose_.theta;

//	ROS_INFO("Current pose: [%f, %f, %f]", current_pose_.x, current_pose_.y, current_pose_.theta);
//	ROS_INFO("Current goal: [%f, %f, %f]", current_goal_pose_.x, current_goal_pose_.y, current_goal_pose_.theta);
//	ROS_INFO("Relative move: [%f, %f, %f]", diff_x_t, diff_y_t, diff_theta);


    if(diff_theta > M_PI)
        diff_theta -= 2*M_PI;
    else if(diff_theta < -M_PI)
        diff_theta += 2*M_PI;

    bool goal_reached = true;

	double diff_x_dt = diff_x_t - last_diff_x_;
	double diff_y_dt = diff_y_t - last_diff_y_; 
	double diff_theta_dt = diff_theta - last_diff_theta_;
    ros::Time current_time = ros::Time::now();
    double delta_t = (current_time - last_update_time_).toSec();
    last_update_time_ = current_time;

	if(has_last_diff_ == false)
	{
		diff_x_dt = 0;
		diff_y_dt = 0; 
		diff_theta_dt = 0;
	}	

    // check x 
	if(std::fabs(diff_x_t) > position_tolerance_x_)
    {
        goal_reached = false;
        velocity_command_.linear.x = diff_x_t * velocity_p_factor_x_ + diff_x_dt * velocity_d_factor_x_ / delta_t;

        velocity_command_.linear.x = std::min(velocity_command_.linear.x, max_velocity_x_);
        velocity_command_.linear.x = std::max(velocity_command_.linear.x, -max_velocity_x_);
    }
    else
        velocity_command_.linear.x = 0;

    // check y
	if(std::fabs(diff_y_t) > position_tolerance_y_)
    {
        goal_reached = false;
        velocity_command_.linear.y = diff_y_t * velocity_p_factor_y_ + diff_y_dt * velocity_d_factor_y_ / delta_t;

        velocity_command_.linear.y = std::min(velocity_command_.linear.y, max_velocity_y_);
        velocity_command_.linear.y = std::max(velocity_command_.linear.y, -max_velocity_y_);

    }
    else
        velocity_command_.linear.y = 0;

    // check theta
	if(std::fabs(diff_theta)  > position_tolerance_theta_){
        goal_reached = false;
        velocity_command_.angular.z = diff_theta * velocity_p_factor_theta_ 
				+ diff_theta_dt * velocity_d_factor_theta_ / delta_t;

        velocity_command_.angular.z = std::min(velocity_command_.angular.z, max_velocity_theta_);
        velocity_command_.angular.z = std::max(velocity_command_.angular.z, -max_velocity_theta_);

    }
    else
        velocity_command_.angular.z = 0;

	last_diff_x_ = diff_x_t;
	last_diff_y_ = diff_y_t;
	last_diff_theta_ = diff_theta;
	has_last_diff_ = true;
	
	
  //  std::cout<<"velocity angular:"<<velocity_command_.angular.z<<endl;
    // In fast mode, if we close enough, call it 
    if(fast_mode_ == true)
    {
        if(std::fabs(diff_x_t) < goal_reached_threshold_x_ && 
            std::fabs(diff_y_t) < goal_reached_threshold_y_ && 
            std::fabs(diff_theta) < goal_reached_threshold_theta_)
            goal_reached = true;
    }

    // if goal is reached: action succeeded
    if(goal_reached)
    {
        mode_ = IDLE;
        base_is_busy_ = false;

        if(!fast_mode_)
        {
            velocity_command_.linear.x = 0;
            velocity_command_.linear.y = 0;
            velocity_command_.angular.z = 0;
        }

        ROS_WARN("Base movement finished.");
        move_base_server_->setSucceeded(moved_distance_);

		has_last_diff_ = false;
		last_diff_x_ = 0;
		last_diff_y_ = 0;
		last_diff_theta_ = 0;
    }
}

//########## UPDATE VELOCITY MODE ######################################################################################
void ModuleBaseController::updateVelocityMode()
{
    double time_passed = (ros::Time::now() - last_update_time_).toSec();

    if(time_passed > velocity_command_timeout_)
    {
        velocity_command_.linear.x = 0;
        velocity_command_.linear.y = 0;
        velocity_command_.angular.z = 0;

        mode_ = IDLE;
    }
}

//########## UPDATE ALIGN MODE #########################################################################################
void ModuleBaseController::updateAlignMode()
{
    // === CHECK TIMEOUT ===
    double delta_t = (ros::Time::now() - last_update_time_).toSec();

    if(delta_t > align_stop_timeout_)
    {
        if(!(velocity_command_.linear.x == 0 && velocity_command_.linear.y == 0 && velocity_command_.angular.z == 0))
        {
            ROS_WARN("No pose message received in %f seconds. Stopping.", align_stop_timeout_);
            velocity_command_.linear.x = 0;
            velocity_command_.linear.y = 0;
            velocity_command_.angular.z = 0;
        }
    }

    if(delta_t > align_fail_timeout_)
    {
        ROS_ERROR("No pose message received in %f seconds. Aborting.", align_fail_timeout_);

        mode_ = IDLE;
        base_is_busy_ = false;

        arcl_youbot_msgs::AlignBaseToPoseResult result;
        result.moved_distance.x = moved_distance_.x;
        result.moved_distance.y = moved_distance_.y;
        result.moved_distance.theta = moved_distance_.theta;
        align_base_server_->setAborted(result);
    }

}

//########## UPDATE APPROACH MODE ######################################################################################
void ModuleBaseController::updateApproachMode()
{
    // double diff_x_t, diff_y_t, diff_theta;

    // if(approach_goal_.front > 0)
    //     diff_x_t = distances_.front - approach_goal_.front;
    // else if(approach_goal_.back > 0)
    //     diff_x_t = approach_goal_.back - distances_.back;
    // else
    //     diff_x_t = 0;

    // if(approach_goal_.left > 0)
    //     diff_y_t = distances_.left - approach_goal_.left;
    // else if(approach_goal_.right > 0)
    //     diff_y_t = approach_goal_.right - distances_.right;
    // else
    //     diff_y_t = 0;

    // diff_theta = 0;

    // if(diff_theta > M_PI)
    //     diff_theta -= 2*M_PI;
    // else if(diff_theta < -M_PI)
    //     diff_theta += 2*M_PI;

    // bool goal_reached = true;

    // // check x
    // if(std::fabs(diff_x_t) > position_tolerance_x_)
    // {
    //     goal_reached = false;
    //     velocity_command_.linear.x = diff_x_t * velocity_p_factor_x_;

    //     velocity_command_.linear.x = std::min(velocity_command_.linear.x, max_velocity_x_);
    //     velocity_command_.linear.x = std::max(velocity_command_.linear.x, -max_velocity_x_);
    // }
    // else
    //     velocity_command_.linear.x = 0;

    // // check y
    // if(std::fabs(diff_y_t) > position_tolerance_y_)
    // {
    //     goal_reached = false;
    //     velocity_command_.linear.y = diff_y_t * velocity_p_factor_y_;

    //     velocity_command_.linear.y = std::min(velocity_command_.linear.y, max_velocity_y_);
    //     velocity_command_.linear.y = std::max(velocity_command_.linear.y, -max_velocity_y_);
    // }
    // else
    //     velocity_command_.linear.y = 0;

    // // check theta
    // if(std::fabs(diff_theta) > position_tolerance_theta_)
    // {
    //     goal_reached = false;
    //     velocity_command_.angular.z = diff_theta * velocity_p_factor_theta_;

    //     velocity_command_.angular.z = std::min(velocity_command_.angular.z, max_velocity_theta_);
    //     velocity_command_.angular.z = std::max(velocity_command_.angular.z, -max_velocity_theta_);
    // }
    // else
    //     velocity_command_.angular.z = 0;

    // if(goal_reached)
    // {
    //     mode_ = IDLE;
    //     base_is_busy_ = false;
    //     velocity_command_.linear.x = 0;
    //     velocity_command_.linear.y = 0;
    //     velocity_command_.angular.z = 0;

    //     ROS_INFO("Approach action finished.");
    //     arcl_youbot_msgs::ApproachBaseResult res;
    //     res.moved_distance.theta = moved_distance_.theta;
    //     res.moved_distance.x = moved_distance_.x;
    //     res.moved_distance.y = moved_distance_.y;
    //     mode_ = IDLE;
    //     approach_server_->setSucceeded(res);

    // }
}

//########## ACTIVATE ##################################################################################################
void ModuleBaseController::activate()
{
    activated_ = true;
}

//########## DEACTIVATE ################################################################################################
void ModuleBaseController::deactivate()
{
    activated_ = false;
}

//########## EMERGENCY STOP ############################################################################################
void ModuleBaseController::emergencyStop()
{
    ROS_WARN("Base emergency stop!");

    velocity_command_.linear.x = 0;
    velocity_command_.linear.y = 0;
    velocity_command_.angular.z = 0;

    if(mode_ == POSITION || mode_ == ALIGN || mode_ == APPROACH)
        base_is_busy_ = false;

    mode_ = IDLE;

    youbot_->base()->setVelocity(velocity_command_);

    if(move_base_server_->isActive())
        //move_base_server_->setAborted(moved_distance_);
    if(align_base_server_->isActive())
    {
        arcl_youbot_msgs::AlignBaseToPoseResult res;
        res.moved_distance.theta = moved_distance_.theta;
        res.moved_distance.x = moved_distance_.x;
        res.moved_distance.y = moved_distance_.y;
        align_base_server_->setAborted(res);
    }
    if(approach_server_->isActive())
    {
        arcl_youbot_msgs::ApproachBaseResult res;
        res.moved_distance.theta = moved_distance_.theta;
        res.moved_distance.x = moved_distance_.x;
        res.moved_distance.y = moved_distance_.y;
        approach_server_->setAborted(res);
    }
}

//###################### CALLBACK: PREEMPTION ##########################################################################
void ModuleBaseController::preemptCallback()
{
    boost::mutex::scoped_lock lock(base_mutex_);
    preempt();
}

//###################### CALLBACK: PREEMPT #############################################################################
void ModuleBaseController::preempt()
{    
    velocity_command_.linear.x = 0;
    velocity_command_.linear.y = 0;
    velocity_command_.angular.z = 0;
    ROS_WARN_STREAM("------------------------------------controler_0:Current action aborted-------------------------------");
    if(mode_ == POSITION || mode_ == ALIGN || mode_ == APPROACH)
        base_is_busy_ = false;

    mode_ = IDLE;

    youbot_->base()->setVelocity(velocity_command_);

    if(move_base_server_->isActive())
        move_base_server_->setPreempted(moved_distance_);
    if(align_base_server_->isActive())
    {
        arcl_youbot_msgs::AlignBaseToPoseResult res;
        res.moved_distance.theta = moved_distance_.theta;
        res.moved_distance.x = moved_distance_.x;
        res.moved_distance.y = moved_distance_.y;
        align_base_server_->setPreempted(res);
    }
    if(approach_server_->isActive())
    {
        arcl_youbot_msgs::ApproachBaseResult res;
        res.moved_distance.theta = moved_distance_.theta;
        res.moved_distance.x = moved_distance_.x;
        res.moved_distance.y = moved_distance_.y;
        approach_server_->setPreempted(res);
    }

    ROS_INFO("Preempted.");
}

//###################### CALLBACK: VELOCITY ############################################################################
void ModuleBaseController::velocityCallback(const geometry_msgs::Twist::ConstPtr &velocity_msg)
{
    boost::mutex::scoped_lock lock(base_mutex_);

    if(mode_ == POSITION || mode_ == ALIGN || mode_ == APPROACH)
    {
        ROS_WARN("Got velocity command. Preempting current action.");
        preempt();
    }

    velocity_command_ = *velocity_msg;

//    velocity_command_.linear.x = std::min(velocity_command_.linear.x, max_velocity_x_);
//    velocity_command_.linear.x = std::max(velocity_command_.linear.x, -max_velocity_x_);

//    velocity_command_.linear.y = std::min(velocity_command_.linear.y, max_velocity_y_);
//    velocity_command_.linear.y = std::max(velocity_command_.linear.y, -max_velocity_y_);

    double vx = velocity_command_.linear.x;
    double vy = velocity_command_.linear.y;
    double factor = sqrt(vx*vx / (max_velocity_x_ * max_velocity_x_) + vy*vy / (max_velocity_y_ * max_velocity_y_));

    if(factor > 1)
    {
        velocity_command_.linear.x = vx / factor;
        velocity_command_.linear.y = vy / factor;
    }

    velocity_command_.angular.z = std::min(velocity_command_.angular.z, max_velocity_theta_);
    velocity_command_.angular.z = std::max(velocity_command_.angular.z, -max_velocity_theta_);

    last_update_time_ = ros::Time::now();

    mode_ = VELOCITY;
}

//###################### CALLBACK: MOVE BASE ###########################################################################
void ModuleBaseController::moveBaseCallback()
{    
    boost::mutex::scoped_lock lock(base_mutex_);

    ROS_WARN("==== Module Base Controller new goal comming in====");
    ROS_WARN("==== Module Base Controller new goal comming in====");
    ROS_WARN("==== Module Base Controller new goal comming in====");

    if(mode_ == POSITION || mode_ == ALIGN || mode_ == APPROACH)
    {
        ROS_WARN("Got new move base goal. Preempting current action.");
        preempt();
    }

    goal_pose_ = *(move_base_server_->acceptNewGoal());

    if(!activated_ || base_is_busy_)
    {
        if(base_is_busy_)
            ROS_ERROR("Base is busy. Can't accept new goals.");
        else
            ROS_ERROR("Module is deactivated. Can't accept new goals.");

        move_base_server_->setAborted();
        return;
    }

    geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();
    current_pose_.x = base_pose.x;
    current_pose_.y = base_pose.y;
    current_pose_.theta = base_pose.theta;

	// Check whether we need to adjust from relative to absolute
    // TODO: 10*M_PI ?
	if(goal_pose_.theta > 10*M_PI)
	{
		goal_pose_.theta -= 20*M_PI;
		// Adjust to absolute x, y, theta
	    ROS_INFO("Adjusting from relative pose: [%f, %f, %f]", 
				goal_pose_.x, goal_pose_.y, goal_pose_.theta);
		
		goal_pose_.theta += current_pose_.theta;
		double tempx = current_pose_.x 
				+ goal_pose_.x * cos(current_pose_.theta)
				- goal_pose_.y * sin(current_pose_.theta);
		double tempy = current_pose_.y 
				+ goal_pose_.x *sin(current_pose_.theta)
				+ goal_pose_.y *cos(current_pose_.theta);
		goal_pose_.x = tempx;
		goal_pose_.y = tempy;
		
	}
    else if(goal_pose_.theta < -10*M_PI)
    {
        // Fast move
		goal_pose_.theta += 20*M_PI;
        node_->param("module_base_controller/position_tolerance_x_fast", position_tolerance_x_, 0.01);
        node_->param("module_base_controller/position_tolerance_y_fast", position_tolerance_y_, 0.01);
        node_->param("module_base_controller/position_tolerance_theta_fast", position_tolerance_theta_, 0.01);
        fast_mode_ = true;
    }
    else
    {
        node_->param("module_base_controller/position_tolerance_x", position_tolerance_x_, 0.01);
        node_->param("module_base_controller/position_tolerance_y", position_tolerance_y_, 0.01);
        node_->param("module_base_controller/position_tolerance_theta", position_tolerance_theta_, 0.01);
        fast_mode_ = false;
    }

    // default is fast mode
    if (goal_pose_.next_x == -111.0 && goal_pose_.next_y == -111.0 && goal_pose_.next_theta == 0.0)
    {
        fast_mode_ = false;
        ROS_INFO("Last pose, fast_mode = false");
    }
    else
    {
        fast_mode_ = true;
    }

    ROS_INFO("Starting pose: [%f, %f, %f]", current_pose_.x, current_pose_.y, current_pose_.theta);
    ROS_INFO("Request to move to pose: [%f, %f, %f]", goal_pose_.x, goal_pose_.y, goal_pose_.theta);

    start_pose_ = current_goal_pose_ = current_pose_;

	goal_distance_ = poseDistance(start_pose_, goal_pose_);
	goal_sin_angle_ = (goal_pose_.y - start_pose_.y) / goal_distance_;
	goal_cos_angle_ = (goal_pose_.x - start_pose_.x) / goal_distance_;

    mode_ = POSITION;
    base_is_busy_ = true;

	has_last_diff_ = false;
	last_diff_x_ = 0;
	last_diff_y_ = 0;
	last_diff_theta_ = 0;

}

//###################### CALLBACK: ALIGN BASE ##########################################################################
void ModuleBaseController::alignBaseCallback()
{
    boost::mutex::scoped_lock lock(base_mutex_);

    ROS_INFO("==== Module Base Controller ====");

    if(mode_ == POSITION || mode_ == ALIGN || mode_ == APPROACH)
    {
        ROS_WARN("Received align goal. Preempting current action.");
        preempt();
    }

    align_goal_ = *(align_base_server_->acceptNewGoal());

    // debug output
    ROS_INFO("Goal position in %s: [%f | %f]", align_goal_.frame_id.c_str(), align_goal_.x_goal, align_goal_.y_goal);

    if(!activated_ || base_is_busy_ || align_topic_.empty())
    {
        if(base_is_busy_)
            ROS_ERROR("Base is busy. Can't accept new goals.");
        else if(!activated_)
            ROS_ERROR("Module is deactivated. Can't accept new goals.");
        else
            ROS_ERROR("No Pose topic specified.");

        align_base_server_->setAborted();
        return;
    }

    // transform goal pose
    try
    {
        tf_listener_->lookupTransform("base_link", align_goal_.frame_id, ros::Time(0), align_pose_transform_);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        align_base_server_->setAborted();
    }

    tf::Vector3 goal_pos(align_goal_.x_goal, align_goal_.y_goal, 0.0);
    tf::Vector3 pos_trans = align_pose_transform_(goal_pos);
    align_goal_.x_goal = pos_trans.x();
    align_goal_.y_goal = pos_trans.y();
    align_goal_.frame_id = "base_link";

    // debug output
    ROS_INFO("Goal position in %s: [%f | %f]", align_goal_.frame_id.c_str(), align_goal_.x_goal, align_goal_.y_goal);

    // subscriber to pose topic


    ROS_INFO("Aligning base...");

    geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();
    current_pose_.x = base_pose.x;
    current_pose_.y = base_pose.y;
    current_pose_.theta = base_pose.theta;
    moved_distance_.x = 0;
    moved_distance_.y = 0;
    moved_distance_.theta = 0;

    // reset memory
    error_memory_.resize(memory_size_);
    time_memory_.assign(memory_size_, 0);

    memory_index_ = 0;

    accumulated_error_.x = 0;
    accumulated_error_.y = 0;
    accumulated_error_.theta = 0;

    BasePose zero_pose;
    zero_pose.x = 0;
    zero_pose.y = 0;
    zero_pose.theta = 0;
    last_two_errors_.push(zero_pose);
    last_two_errors_.push(zero_pose);

    last_update_time_ = ros::Time::now();

    start_pose_ = current_pose_;
    mode_ = ALIGN;
    base_is_busy_ = true;
}

//###################### CALLBACK: POSE ################################################################################
void ModuleBaseController::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    boost::mutex::scoped_lock lock(base_mutex_);

    if(mode_ != ALIGN)
        return;

    // === TRANSFORM POSE ===
    tf::Vector3 pos(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
    tf::Vector3 pos_trans = align_pose_transform_(pos);

    // === GET VELOCTY ===

    // pose difference
    double diff_x = pos_trans.x() - align_goal_.x_goal;
    double diff_y = pos_trans.y() - align_goal_.y_goal;

    //debug output
    ROS_INFO("error x: %f - %f = %f", pos_trans.x(), align_goal_.x_goal, diff_x);
    ROS_INFO("error y: %f - %f = %f", pos_trans.y(), align_goal_.y_goal, diff_y);

    // publish error
    geometry_msgs::Pose2D error_msg;
    error_msg.x = diff_x;
    error_msg.y = diff_y;
    error_publisher_.publish(error_msg);

    // check time diff for zeros
    bool has_zeros = false;
    for(uint i=0; i<time_memory_.size(); i++)
    {
        if(time_memory_[i] == 0)
        {
            has_zeros = true;
            break;
        }
    }

    // if all time diffs are valid: check velocity
    if(!has_zeros)
    {
        double time = 0;
        BasePose pos;
        pos.x = 0;
        pos.y = 0;
        pos.theta = 0;
        for(int i=0; i<memory_size_; i++)
        {
            time += time_memory_[i];
            pos.x += error_memory_[i].x;
            pos.y += error_memory_[i].y;
        }

        double velocity_x = pos.x / time;
        double velocity_y = pos.y / time;

        // debug output
        ROS_INFO("velocity x: %f / %f = %f", pos.x, time, velocity_x);
        ROS_INFO("velocity y: %f / %f = %f", pos.y, time, velocity_y);
        ROS_INFO("----------");

        // === CHECK TOLERANCES ===
        if(fabs(velocity_x) < align_goal_.x_vel_tolerance
                && fabs(velocity_y) < align_goal_.y_vel_tolerance
                && fabs(diff_x) < align_goal_.x_pos_tolerance
                && fabs(diff_y) < align_goal_.y_pos_tolerance)
        {
            ROS_INFO("Alignment succeeded.");

            arcl_youbot_msgs::AlignBaseToPoseResult result;
            result.moved_distance.x = moved_distance_.x;
            result.moved_distance.y = moved_distance_.y;
            result.moved_distance.theta = moved_distance_.theta;
            result.last_pose = *pose_msg;

            mode_ = IDLE;
            align_base_server_->setSucceeded(result);
            return;
        }

    }


    // === PID CONTROLLER ===

    // passed time
    ros::Time current_time = ros::Time::now();
    double delta_t = (current_time - last_update_time_).toSec();
    last_update_time_ = current_time;

    // PID controller
    velocity_command_.linear.x += (velocity_p_factor_x_ + velocity_d_factor_x_/delta_t) * diff_x
            + (velocity_i_factor_x_ * delta_t - velocity_p_factor_x_ - 2 * velocity_d_factor_x_ / delta_t)
            * last_two_errors_.back().x
            + velocity_d_factor_x_ / delta_t * last_two_errors_.front().x;

    velocity_command_.linear.y += (velocity_p_factor_y_ + velocity_d_factor_y_/delta_t) * diff_y
            + (velocity_i_factor_y_ * delta_t - velocity_p_factor_y_ - 2 * velocity_d_factor_y_ / delta_t)
            * last_two_errors_.back().y
            + velocity_d_factor_y_ / delta_t * last_two_errors_.front().y;

    // saturation
    velocity_command_.linear.x = std::min(velocity_command_.linear.x, max_velocity_x_);
    velocity_command_.linear.x = std::max(velocity_command_.linear.x, -max_velocity_x_);

    velocity_command_.linear.y = std::min(velocity_command_.linear.y, max_velocity_y_);
    velocity_command_.linear.y = std::max(velocity_command_.linear.y, -max_velocity_y_);


    // === UPDATE MEMORY ===
    BasePose current_error;
    current_error.theta = 0;
    current_error.x = diff_x;
    current_error.y = diff_y;
    last_two_errors_.push(current_error);
    last_two_errors_.pop();

    BasePose error_diff;
    error_diff.x = current_error.x - last_two_errors_.front().x;
    error_diff.y = current_error.y - last_two_errors_.front().y;
    error_diff.theta = current_error.theta - last_two_errors_.front().theta;

    time_memory_[memory_index_] = delta_t;
    error_memory_[memory_index_] = error_diff;
    memory_index_ = (memory_index_ + 1) % memory_size_;

    // === UPDATE MOVED DISTANCE ===
    geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();
    current_pose_.x = base_pose.x;
    current_pose_.y = base_pose.y;
    current_pose_.theta = base_pose.theta;

    // get difference between current pose and start pose
    double dx = current_pose_.x - start_pose_.x;
    double dy = current_pose_.y - start_pose_.y;
    double dtheta = current_pose_.theta - start_pose_.theta;

    double dx_t = dx * cos(base_pose.theta) + dy * sin(base_pose.theta);
    double dy_t = dy * cos(base_pose.theta) - dx * sin(base_pose.theta);

    moved_distance_.x = dx_t;
    moved_distance_.y = dy_t;
    moved_distance_.theta = dtheta;
}

//###################### CALLBACK: STOP ################################################################################
bool ModuleBaseController::stopCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    boost::mutex::scoped_lock lock(base_mutex_);

    emergencyStop();

    return true;
}

// //###################### CALLBACK: LASER WATCHDOG ######################################################################
// void ModuleBaseController::laserCallback(const luh_laser_watchdog::Distances::ConstPtr &distances)
// {
//     boost::mutex::scoped_lock lock(base_mutex_);

//     distances_ = *distances;
// }


//###################### CALLBACK: APPROACH ACTION #####################################################################
void ModuleBaseController::approachCallback()
{
    // boost::mutex::scoped_lock lock(base_mutex_);

    // ROS_INFO("==== Module Base Controller ====");
    // ROS_INFO("Approach goal received.");

    // if(mode_ == POSITION || mode_ == ALIGN || mode_ == APPROACH)
    // {
    //     ROS_WARN("Received approach goal. Preempting current action.");
    //     preempt();
    // }

    // approach_goal_ = *(approach_server_->acceptNewGoal());

    // if(!activated_ || base_is_busy_ || laser_subscriber_.getNumPublishers() == 0)
    // {
    //     if(base_is_busy_)
    //         ROS_ERROR("Base is busy. Can't accept new goals.");
    //     else if(!activated_)
    //         ROS_ERROR("Module is deactivated. Can't accept new goals.");
    //     else
    //         ROS_ERROR("No laser scanner data available. Can't accept new goals.");

    //     approach_server_->setAborted();
    //     return;
    // }

    // // eliminate conflicting goals
    // if(approach_goal_.back > 0 && approach_goal_.front > 0)
    //     approach_goal_.back = 0;
    // if(approach_goal_.right > 0 && approach_goal_.left > 0)
    //     approach_goal_.left = 0;

    // ROS_INFO("Approaching...");

    // geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();
    // current_pose_.x = base_pose.x;
    // current_pose_.y = base_pose.y;
    // current_pose_.theta = base_pose.theta;

    // moved_distance_.x = 0;
    // moved_distance_.y = 0;
    // moved_distance_.theta = 0;

    // start_pose_ = current_pose_;
    // mode_ = APPROACH;
    // base_is_busy_ = true;
}

//###################### CALLBACK: GET BASE POSE #######################################################################
bool ModuleBaseController::getPoseCallback(arcl_youbot_msgs::GetBasePose::Request &req,
                                           arcl_youbot_msgs::GetBasePose::Response &res)
{
    boost::mutex::scoped_lock lock(base_mutex_);

    geometry_msgs::Pose2D base_pose = youbot_->base()->getPose();

    res.x = base_pose.x;
    res.y = base_pose.y;
    res.theta = base_pose.theta;

    return true;
}
