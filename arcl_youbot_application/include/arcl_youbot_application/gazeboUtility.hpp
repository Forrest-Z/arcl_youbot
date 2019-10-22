#ifndef __ARC_GAZEBO_UTILITY__
#define __ARC_GAZEBO_UTILITY__

// ROS and gazebo 
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <arcl_youbot_application/utilities.hpp>
/**
 * Class for handling the youbot 
 */

namespace arc{

namespace gazeboUtility{

const std::string GAZEBO_COLORS[] = {
"Gazebo/White",
"Gazebo/Grey",
"Gazebo/Red",
"Gazebo/Green",
"Gazebo/Yellow",
"Gazebo/Purple",
"Gazebo/Turquoise",
"Gazebo/Blue",
"Gazebo/Gold",
"Gazebo/FlatBlack"
};


/**
 * Spawn an SDF model at given pose. If objectName is empty, then do not set it (using the one 
 * specified in the model)
 */
void spawnSDFModel(ros::NodeHandle &node_handle_, const std::string& SDFFile, 
geometry_msgs::Pose& pose, const std::string& objectName);

/**
 * Spawn an SDF model at given location (x, y, z)  and orientation (qx, qy, qz, qw), a quaternion.
 * If objectName is empty, then do not set it (using the one specified in the model)
 */
void spawnSDFModel(ros::NodeHandle &node_handle_, const std::string& SDFFile, 
double x, double y, double z, 
double qx, double qy, double qz, double qw, 
 const std::string& objectName);

void spawnURDFModel(ros::NodeHandle &node_handle_, const std::string& SDFFile, 
geometry_msgs::Pose& pose, const std::string& objectName);

/**
 * Spawn an SDF model at given location (x, y, z)  and orientation (qx, qy, qz, qw), a quaternion.
 * If objectName is empty, then do not set it (using the one specified in the model)
 */
void spawnURDFModel(ros::NodeHandle &node_handle_, const std::string& SDFFile, 
double x, double y, double z, 
double qx, double qy, double qz, double qw, 
 const std::string& objectName);
/**
 * Spawn one cuboid with given dimension (dx, dy, dz) at given pose 
 */
void spawnCuboid(ros::NodeHandle &node_handle_, double mass, double dx, double dy, 
double dz, geometry_msgs::Pose& pose, const std::string& material, const std::string& objectName);

void spawnCuboid(ros::NodeHandle &node_handle_, double mass, double dx, double dy, double dz,arc::polygon_2 footprint, const std::string& material, const std::string& objectName);

void spawnCuboid(ros::NodeHandle &node_handle_, double mass, double dx, double dy, double dz, polygon_2 footprint, const std::string& material, const std::string& objectName, double height);
/**
 * Spawn one cuboid with given dimension (dx, dy, dz) at given pose 
 */
void spawnPassThroughCuboid(ros::NodeHandle &node_handle_, double dx, double dy, double dz, 
geometry_msgs::Pose& pose, const std::string& objectName, bool colorRed = false);

/**
 * Spawn one cuboid with given dimension (dx, dy, dz) at given location (x, y, z) 
 * and orientation (qx, qy, qz, qw), a quaternion 
 */
void spawnCuboid(ros::NodeHandle &node_handle_, 
double mass, 
double dx, double dy, double dz, 
double x, double y, double z, 
double qx, double qy, double qz, double qw, 
const std::string& material,
const std::string& objectName);

/**
 * Spawn one cuboid with given dimension (dx, dy, dz) at given location (x, y, z) 
 * and orientation (qx, qy, qz, qw), a quaternion 
 */
void spawnPassThroughCuboid(ros::NodeHandle &node_handle_, 
double dx, double dy, double dz, 
double x, double y, double z, 
double qx, double qy, double qz, double qw, 
const std::string& objectName, bool colorRed = false);

/**
 * Spawn a number of cuboid objects with each having a name in the form of 
 * "cuboid_x" where "x" is the number of the cuboid startin from 1. The position 
 * and orientation of the objects are randomized. 
 */
void spawnRandomCuboids(ros::NodeHandle &node_handle_, double length, double width, int numberOfObjects);

/**
 * Delete a model from gazebo by name
 */
void deleteModel(ros::NodeHandle &node_handle_,std::string &modelName);

/**
 * Retrieve the pose of one object by name 
 */
bool getObjectPose(geometry_msgs::Pose& pose, const std::string& objectName);

/**
 * Retrieve the poses of all objects as a map of name-pose pairs 
 */
void getObjectPoses(std::set< std::string >* pObjectNameSet,
std::map< std::string, geometry_msgs::Pose >& objectNamePoseMap);

/**
 * Retrieve the pose of the youbot base. We assume that the based will be on the  
 * ground so we are really retrieving a 2D pose. The function will return false if it 
 * cannot find a youbot. This wll need change if we have multiple youbots. 
 */
bool getYoubotBasePose(double& x, double& y, double& theta, 
    const std::string & botName = std::string("youbot"), bool print = false);

/**
 * Set base location for youbot
 */
void setYoubotBasePose(ros::NodeHandle &node_handle_, 
    double& x, double& y, double& theta, const std::string & botName = std::string("youbot"));

void setObjectPose(ros::NodeHandle &node_handle_, double c_x, double c_y, 
    double c_z, double q_x, double q_y, double q_z, double q_w, const std::string name);

geometry_msgs::Pose getYoubotCurrentPose(std::string robot_name);

}



}

#endif // __ARC_GAZEBO_UTILITY__