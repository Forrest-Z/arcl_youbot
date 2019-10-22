//ros
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/scoped_ptr.hpp>
#include <pluginlib/class_loader.h>


//arc_youbot_planner
#include <arcl_youbot_application/ManipulationAction.h>
#include <luh_youbot_controller_api/controller_api.h>
#include <arcl_youbot_kinematics/arm_kinematics.h>
#include <arcl_youbot_kinematics/constants.h>
#include <arcl_youbot_application/PlanningSceneMsg.h>
#include <arcl_youbot_application/utilities.hpp>
#include <arcl_youbot_application/planningScene.hpp>
//#include <gazebo_util/gazeboUtility.hpp>

//youbot_grasp
#include <youbot_grasp/grasp_generator.h>
#include "youbot_grasp/GraspPlanning.h"
#include "youbot_grasp/PlannedGrasp.h"
#include "youbot_grasp/PlanningScene.h"
#include "youbot_grasp/PlannedGrasp_vector.h"


//geometric_shapes from ros-planning
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
//#include "geometric_shapes/mesh_operations.h"
// #include "geometric_shapes/shape_operations.h"





#define NUM_OBJECT 40    // number of objects in vrep environment
#define BASE_LENGTH 0.623
#define BASE_WIDTH  0.46
#define YOUBOT_BASE_LENGTH  0.623
#define YOUBOT_BASE_WIDTH  0.46
#define USE_GAZEBO_
#define DEBUG_
using namespace youbot_api;


class ManipulationServer
{
public:
    ManipulationServer(std::string name);
    ~ManipulationServer(void);
    void goalCB();
    bool planInAdvance(geometry_msgs::Pose final_base_pose, std::vector<double> test_group_variable_values, ros::NodeHandle& node_handle);
    void preemptCB();
    void ManipulationSceneToGraspScene(arcl_youbot_application::PlanningSceneMsg m_scene, youbot_grasp::PlanningScene& g_scene);
    void FromGraspPoseToBasePose(geometry_msgs::Pose gripper_pose, std::vector<double> gripper_dir, double cylin_r, double q1, double q5, double theta, geometry_msgs::Pose &base_pose);
    void GenerateSamplesForArmConf(geometry_msgs::Pose gripper_pose, std::vector<double> gripper_dir,
    double &min_cylin_r, double &max_cylin_r, double &cylin_z, std::vector<double>& q1_list, std::vector<double>& q5_list, double& theta);
protected:
    
    ros::NodeHandle nh_;
    // ManipulationAction 
    actionlib::SimpleActionServer<arcl_youbot_application::ManipulationAction> as_;
    std::string action_name_;
    arcl_youbot_application::ManipulationFeedback feedback_;
    arcl_youbot_application::ManipulationResult result_;
    arcl_youbot_application::ManipulationGoal goal_;
    arcl_youbot_application::PlanningSceneMsg manipulation_scene_;
    int last_target_object_index_;
    std_msgs::String target_object_name_;
    std_msgs::String last_target_object_name_;
    std_msgs::String target_object_type_;
    geometry_msgs::Pose target_object_pose_;
    geometry_msgs::Pose rest_base_pose_;
    bool is_synchronize_;
    //grasp planning
    ros::ServiceClient grasp_client_;
    youbot_grasp::PlanningScene grasp_scene_;  
    arc::polygon_2 foot_print_;

    //moveit
    // moveit::planning_interface::MoveGroup::Plan my_plan;
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // moveit_msgs::DisplayTrajectory display_trajectory;
    std::vector<std::string> cube_name_list = {"cub0","cub1","cub2","cub3","cub4","cub5","cub6","cub7","cub8","cub9","cub10","cub11","cub12","cub13","cub14"};
    geometry_msgs::PoseStamped cube_pose_list[NUM_OBJECT];

    
    
};