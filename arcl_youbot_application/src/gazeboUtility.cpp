#include "arcl_youbot_application/gazeboUtility.hpp"

// Standard C++
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <fstream>

// ROS related
#include <ros/ros.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

#include "arcl_youbot_application/utilities.hpp"

namespace arc{

namespace gazeboUtility{


/**
 * Spawn an SDF model at given pose. If objectName is empty, then do not set it (using the one 
 * specified in the model)
 */
void spawnSDFModel(ros::NodeHandle &node_handle_, const std::string& SDFFile, 
geometry_msgs::Pose& pose, const std::string& objectName)
{
    spawnSDFModel(node_handle_, SDFFile, pose.position.x, pose.position.y, pose.position.z, 
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, objectName);
}

/**
 * Spawn an SDF model at given location (x, y, z)  and orientation (qx, qy, qz, qw), a quaternion.
 * If objectName is empty, then do not set it (using the one specified in the model)
 */
void spawnSDFModel(ros::NodeHandle &node_handle_, const std::string& SDFFile, 
double x, double y, double z, 
double qx, double qy, double qz, double qw, 
 const std::string& objectName)
 {
	ros::ServiceClient gazebo_spawn_clt_ 
		= node_handle_.serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");

	gazebo_msgs::SpawnModel model;
	std::ifstream ifs(SDFFile.c_str());
	
	std::string line;
	while(!ifs.eof()) // Parse the contents of the given urdf in a string
    {
		std::getline(ifs,line);
		model.request.model_xml+=line;
    }
	ifs.close();

	model.request.model_name = objectName;
	model.request.reference_frame = "world";
    model.request.initial_pose.position.x = x;
    model.request.initial_pose.position.y = y;
    model.request.initial_pose.position.z = z;
    model.request.initial_pose.orientation.x = qx;
    model.request.initial_pose.orientation.y = qy;
    model.request.initial_pose.orientation.z = qz;
    model.request.initial_pose.orientation.w = qw;

	gazebo_spawn_clt_.call(model); 
 }

void spawnURDFModel(ros::NodeHandle &node_handle_, const std::string& SDFFile, 
geometry_msgs::Pose& pose, const std::string& objectName)
{
    spawnURDFModel(node_handle_, SDFFile, pose.position.x, pose.position.y, pose.position.z, 
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, objectName);
}

/**
 * Spawn an SDF model at given location (x, y, z)  and orientation (qx, qy, qz, qw), a quaternion.
 * If objectName is empty, then do not set it (using the one specified in the model)
 */
void spawnURDFModel(ros::NodeHandle &node_handle_, const std::string& SDFFile, 
double x, double y, double z, 
double qx, double qy, double qz, double qw, 
 const std::string& objectName)
 {
    ros::ServiceClient gazebo_spawn_clt_ 
        = node_handle_.serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");

    gazebo_msgs::SpawnModel model;
    std::ifstream ifs(SDFFile.c_str());
    
    std::string line;
    while(!ifs.eof()) // Parse the contents of the given urdf in a string
    {
        std::getline(ifs,line);
        model.request.model_xml+=line;
    }
    ifs.close();

    model.request.model_name = objectName;
    model.request.reference_frame = "world";
    model.request.initial_pose.position.x = x;
    model.request.initial_pose.position.y = y;
    model.request.initial_pose.position.z = z;
    model.request.initial_pose.orientation.x = qx;
    model.request.initial_pose.orientation.y = qy;
    model.request.initial_pose.orientation.z = qz;
    model.request.initial_pose.orientation.w = qw;

    gazebo_spawn_clt_.call(model); 
 }

/**
 * Spawn one cuboid with given dimension (dx, dy, dz) at given pose 
 */
void spawnCuboid(ros::NodeHandle &node_handle_, double mass, double dx, double dy, 
double dz, geometry_msgs::Pose& pose, const std::string& material, const std::string& objectName)
{
    spawnCuboid(node_handle_, mass, dx, dy, dz, pose.position.x, pose.position.y, pose.position.z, 
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, material, objectName);
}

void spawnCuboid(ros::NodeHandle &node_handle_, double mass, double dx, double dy, double dz, polygon_2 footprint, const std::string& material, const std::string& objectName, double height){
    point_2 corner_0 = footprint.outer()[0];
    point_2 corner_1 = footprint.outer()[1];
    point_2 corner_2 = footprint.outer()[2];
    point_2 corner_3 = footprint.outer()[3];
    double center_x, center_y, center_z, q_x, q_y, q_z, q_w;
    center_x = 0;
    center_y = 0;
    q_x = 0;
    q_y = 0;
    q_z = 0;
    q_w = 1;
    center_x = (corner_0.get<0>() + corner_1.get<0>() + corner_2.get<0>() + corner_3.get<0>()) / 4;
    center_y = (corner_0.get<1>() + corner_1.get<1>() + corner_2.get<1>() + corner_3.get<1>()) / 4;
    center_z = height;
    double yaw = getYawFromPolygon(footprint);
    tf::Quaternion q = getChangedPoseFromYaw(yaw);
    if(std::abs(yaw- 0.0)<0.01){
        q_x = 0;
        q_y =  0.7071068;
        q_z =  0;
        q_w =  0.7071068;
    }else if(std::abs(yaw- 1.57)<0.01){
        q_x =  0.7071068;
        q_y = 0;
        q_z =  0;
        q_w =  0.7071068;
    }else if(std::abs(yaw+ 1.57)<0.01){
       q_x =  0.7071068;
        q_y = 0;
        q_z =  0;
        q_w =  0.7071068;
    }else{
        q_x = q.x();
        q_y = q.y();
        q_z = q.z();
        q_w = q.getW();
    }
    // if(std::abs(q.x()+0.5)<0.01){
    //     tf::Quaternion q(-0.5, -0.5, -0.5, 0.5);
    // }

    // q_x = q.x();
    // q_y = q.y();
    // q_z = q.z();
    // q_w = q.getW();
    ROS_WARN_STREAM("object:"<<objectName <<" quaternion:"<<q_x<<","<<q_y<<","<<q_z<<","<<q_w);

    spawnCuboid(node_handle_, mass, dx, dy, dz, center_x, center_y, center_z, 
    q_x, q_y, q_z, q_w, material, objectName);
}
// Spawn one cuboid from a given footprint, in the form of polygon_2
void spawnCuboid(ros::NodeHandle &node_handle_, double mass, double dx, double dy, double dz, polygon_2 footprint, const std::string& material, const std::string& objectName){
    //double dx = 0.032;
    //double dy = 0.032;
    //double dz = 0.15;

    point_2 corner_0 = footprint.outer()[0];
    point_2 corner_1 = footprint.outer()[1];
    point_2 corner_2 = footprint.outer()[2];
    point_2 corner_3 = footprint.outer()[3];
    double center_x, center_y, center_z, q_x, q_y, q_z, q_w;
    center_x = 0;
    center_y = 0;
    q_x = 0;
    q_y = 0;
    q_z = 0;
    q_w = 1;
    center_x = (corner_0.get<0>() + corner_1.get<0>() + corner_2.get<0>() + corner_3.get<0>()) / 4;
    center_y = (corner_0.get<1>() + corner_1.get<1>() + corner_2.get<1>() + corner_3.get<1>()) / 4;
    center_z = 0.05;
    double yaw = getYawFromPolygon(footprint);
    tf::Quaternion q = getChangedPoseFromYaw(yaw);
    if(std::abs(yaw- 0.0)<0.01){
        q_x = 0;
        q_y =  0.7071068;
        q_z =  0;
        q_w =  0.7071068;
    }else if(std::abs(yaw- 1.57)<0.01){
        q_x =  0.7071068;
        q_y = 0;
        q_z =  0;
        q_w =  0.7071068;
    }else if(std::abs(yaw+ 1.57)<0.01){
       q_x =  0.7071068;
        q_y = 0;
        q_z =  0;
        q_w =  0.7071068;
    }else{
        q_x = q.x();
        q_y = q.y();
        q_z = q.z();
        q_w = q.getW();
    }
    // if(std::abs(q.x()+0.5)<0.01){
    //     tf::Quaternion q(-0.5, -0.5, -0.5, 0.5);
    // }

    // q_x = q.x();
    // q_y = q.y();
    // q_z = q.z();
    // q_w = q.getW();
    ROS_WARN_STREAM("object:"<<objectName <<" quaternion:"<<q_x<<","<<q_y<<","<<q_z<<","<<q_w);

    spawnCuboid(node_handle_, mass, dx, dy, dz, center_x, center_y, center_z, 
    q_x, q_y, q_z, q_w, material, objectName);
}



/**
 * Spawn one cuboid with given dimension (dx, dy, dz) at given location (x, y, z) and orientation
 * (qx, qy, qz, qw), a quaternion 
 */
void spawnCuboid(ros::NodeHandle &node_handle_, double mass, double dx, double dy, double dz, 
double x, double y, double z, double qx, double qy, double qz, double qw, 
const std::string& material, const std::string& objectName)
{
    // Prepare the model
    ROS_WARN_STREAM("dx:"<<dx<<", dy:"<<dy<<", dz:"<<dz);
	ros::ServiceClient gazebo_spawn_clt_ 
		= node_handle_.serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");
	gazebo_msgs::SpawnModel model;

    // Populate basic XML properties 
    std::string ss = "<robot><link name='box'><inertial><origin xyz='0 0 0' /><mass value='0.3' /><inertia  ixx='0.002625115' ixy='0.0'  ixz='0.0'  iyy='0.002625115'  iyz='0.0'  izz='0.000023563' /></inertial><visual><origin xyz='0 0 0'/><geometry><box size='0.0376 0.0376 0.56' /></geometry></visual><collision><geometry><box size='0.0376 0.0376 0.56' /></geometry></collision></link><gazebo reference='box'><maxVel>0.0</maxVel><kp>10000.0</kp><kd>1.0</kd><minDepth>0.001</minDepth><mu1>200.0</mu1><mu2>200.0</mu2><material>Gazebo/Red</material></gazebo></robot>";

    //mass = mass*0.1;
    // ss  << "<robot><link name='box'><inertial><origin xyz='0 0 0' /><mass value='" << mass<< "' />"
    //     << "<inertia  ixx='" << (1./12)*(dy*dy + dz*dz)*0.1 << "' ixy='0.0'  ixz='0.0'  iyy='" 
    //     << (1./12)*(dx*dx + dz*dz)*0.1 << "'  iyz='0.0'  izz='"  << (1./12)*(dx*dx + dy*dy)*0.1 << "' /></inertial>"
    //     << "<visual><origin xyz='0 0 0'/><geometry>"
    //     << "<box size='" << dx << " " << dy << " " << dz << "' /></geometry></visual>"
    //     << "<collision><geometry><box size='" << dx << " " << dy << " " << dz << "' /></geometry>"
    //     << "</collision></link>"
    //     << "<gazebo reference='box'><maxVel>0.0</maxVel><kp>10000.0</kp><kd>1.0</kd><minDepth>0.001</minDepth><mu1>200.0</mu1>"
    //     << "<mu2>200.0</mu2><material>" << material << "</material></gazebo></robot>";
    
    ROS_WARN("1");
    
    // ss  << "<robot><link name='box'><inertial><origin xyz='0 0 0' /><mass value='" << mass<< "' /><inertia  ixx='" << (1./12)*(dy*dy + dz*dz)*0.1 << "' ixy='0.0'  ixz='0.0'  iyy='" 
    //     << (1./12)*(dx*dx + dz*dz)*0.1 << "'  iyz='0.0'  izz='"  << (1./12)*(dx*dx + dy*dy)*0.1 << "' /></inertial><visual><origin xyz='0 0 0'/><geometry><box size='" << dx << " " << dy << " " << dz << "' /></geometry></visual><collision><geometry><box size='" << dx << " " << dy << " " << dz << "' /></geometry></collision></link><gazebo reference='box'><maxVel>0.0</maxVel><kp>10000.0</kp><kd>1.0</kd><minDepth>0.001</minDepth><mu1>200.0</mu1><mu2>200.0</mu2><material>" << material << "</material></gazebo></robot>";
    ROS_WARN("2");
    
    model.request.model_xml = ss;
    ROS_WARN("3");
    
    // Debugging
    // std::cout << model.request.model_xml.c_str() << std::endl;

    // Other model properties
    ROS_WARN_STREAM("object:quaternion:"<<qx<<","<<qy<<","<<qz<<","<<qw);
    model.request.model_name.append(objectName);
    model.request.reference_frame = "world";
    model.request.initial_pose.position.x = x;
    model.request.initial_pose.position.y = y;
    model.request.initial_pose.position.z = z;
    model.request.initial_pose.orientation.x = qx;
    model.request.initial_pose.orientation.y = qy;
    model.request.initial_pose.orientation.z = qz;
    model.request.initial_pose.orientation.w = qw;

    // Spawn the object
    gazebo_spawn_clt_.call(model); 
    setObjectPose(node_handle_, x, y, z, qx, qy, qz, qw, objectName);
}
// void spawnCuboid(ros::NodeHandle &node_handle_, double mass, double dx, double dy, double dz, 
// double x, double y, double z, double qx, double qy, double qz, double qw, 
// const std::string& material, const std::string& objectName)
// {
//     // Prepare the model
// 	ros::ServiceClient gazebo_spawn_clt_ 
// 		= node_handle_.serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
// 	gazebo_msgs::SpawnModel model;
    
//     // Populate basic XML properties 
//     std::stringstream ss;
    

//     ss<<"<?xml version='1.0' ?><sdf version='1.4'><model name='box'><static>false</static>" 
//     <<"<link name='box'>" 
//     << "<inertial><mass>0.1</mass><inertia>  <ixx>"<< (1./12)*(dy*dy + dz*dz) << "</ixx> <ixy>0.0</ixy>  <ixz>0.0</ixz>  <iyy>" 
//     << (1./12)*(dx*dx + dz*dz) << "</iyy> <iyz>0.0</iyz>  <izz>"  << (1./12)*(dx*dx + dy*dy) << "</izz></inertia></inertial>"
//     << "<visual name='box'><geometry>"
//     << "<box><size>" << dx << " " << dy << " " << dz << "</size></box></geometry></visual>"
//     << "<collision name='box'><geometry><box><size>" << dx << " " << dy << " " << dz << "</size></box></geometry>"
//     <<"<surface><friction><ode><mu>500</mu><mu2>500</mu2>" 
//     <<"<fdir1>0.300000 0.300000 0.300000</fdir1><slip1>0.5</slip1><slip2>0.5</slip2></ode></friction>" 
//     <<"<bounce><restitution_coefficient>0.200000</restitution_coefficient><threshold>100000.000000</threshold></bounce><contact><ode><soft_cfm>0.000000</soft_cfm><soft_erp>0.200000</soft_erp><kp>10000000</kp><kd>1</kd><max_vel>100.000000</max_vel><min_depth>0.001</min_depth></ode></contact></surface>" 
//     <<"<surface><friction><ode><mu>100000.0</mu><mu2>100000.0</mu2></ode></friction></surface>"
//     << "</collision></link>"
//     <<"</link></model></sdf>";
//     model.request.model_xml = ss.str();

//     // Debugging
//     // std::cout << model.request.model_xml.c_str() << std::endl;

//     // Other model properties
//     model.request.model_name.append(objectName);
//     model.request.reference_frame = "world";
//     model.request.initial_pose.position.x = x;
//     model.request.initial_pose.position.y = y;
//     model.request.initial_pose.position.z = z;
//     model.request.initial_pose.orientation.x = qx;
//     model.request.initial_pose.orientation.y = qy;
//     model.request.initial_pose.orientation.z = qz;
//     model.request.initial_pose.orientation.w = qw;

//     // Spawn the object
//     gazebo_spawn_clt_.call(model); 
// }


/**
 * Spawn one cuboid with given dimension (dx, dy, dz) at given pose 
 */
void spawnPassThroughCuboid(ros::NodeHandle &node_handle_, double dx, double dy, double dz, 
geometry_msgs::Pose& pose, const std::string& objectName, bool colorRed)
{
    spawnPassThroughCuboid(node_handle_, dx, dy, dz, pose.position.x, pose.position.y, pose.position.z, 
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, objectName,colorRed);
}


/**
 * Spawn one static cuboid with given dimension (dx, dy, dz) at given location (x, y, z) 
 * and orientation (qx, qy, qz, qw), a quaternion. No collision!
 */
void spawnPassThroughCuboid(ros::NodeHandle &node_handle_, 
double dx, double dy, double dz, 
double x, double y, double z, 
double qx, double qy, double qz, double qw, 
const std::string& objectName, bool colorRed)
{
    // Prepare the model
	ros::ServiceClient gazebo_spawn_clt_ 
		= node_handle_.serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
	gazebo_msgs::SpawnModel model;
    
    // Populate basic XML properties 

    std::stringstream ss;
    if(colorRed)
    {
        ss  << "<?xml version='1.0'?><sdf version='1.4'><model><static>true</static><link name='link'>"
            << "<visual name='visual'><geometry><box><size>"<< dx << " " << dy << " " << dz << "</size>"
            << "</box></geometry><material><script><name>Gazebo/Red</name></script></material>"
            << "</visual></link></model></sdf>";
    }
    else
    {
        ss  << "<?xml version='1.0'?><sdf version='1.4'><model><static>true</static><link name='link'>"
            << "<visual name='visual'><geometry><box><size>"<< dx << " " << dy << " " << dz << "</size>"
            << "</box></geometry><material><script><name>Gazebo/Blue</name></script></material>"
            << "</visual></link></model></sdf>";
    }
    
    model.request.model_xml = ss.str();

    // Debugging
    // std::cout << model.request.model_xml.c_str() << std::endl;

    // Other model properties
    model.request.model_name.append(objectName);
    model.request.reference_frame = "world";
    model.request.initial_pose.position.x = x;
    model.request.initial_pose.position.y = y;
    model.request.initial_pose.position.z = z;
    model.request.initial_pose.orientation.x = qx;
    model.request.initial_pose.orientation.y = qy;
    model.request.initial_pose.orientation.z = qz;
    model.request.initial_pose.orientation.w = qw;

    // Spawn the object
    gazebo_spawn_clt_.call(model); 



}


/**
 * Spawn a bunch of cuboids with mass = 0.1 and size dx = dy = 0.025, dz = 0.15 at randomized 
 * locations 
 */
void spawnRandomCuboids(ros::NodeHandle &node_handle_, double length, double width, int numberOfObjects) 
{
	for(int i = 0; i < numberOfObjects;i ++){
		std::stringstream objectNameSS;
		objectNameSS << "cuboid_" << (i+1); 

        spawnCuboid(node_handle_, 0.1, 0.025, 0.025, 0.15, 
            (rand() - RAND_MAX/2.)/RAND_MAX*length,
            (rand() - RAND_MAX/2.)/RAND_MAX*width,
            (rand())*1.0/RAND_MAX + 0.2,
            (rand() - RAND_MAX/2.)/RAND_MAX, 
            (rand() - RAND_MAX/2.)/RAND_MAX,
            (rand() - RAND_MAX/2.)/RAND_MAX,
            1,
            GAZEBO_COLORS[i%10],
            objectNameSS.str());
	}
}


/**
 * Delete a model from gazebo by name
 */
void deleteModel(ros::NodeHandle &node_handle_, std::string &modelName)
{
	ros::ServiceClient gazebo_delete_ctl_
		= node_handle_.serviceClient< gazebo_msgs::DeleteModel> ("gazebo/delete_model");
    gazebo_msgs::DeleteModel model;
    model.request.model_name = modelName;
    gazebo_delete_ctl_.call(model);
}

/**
 * Retrieve the pose of one object by name 
 */
bool getObjectPose(geometry_msgs::Pose& pose, const std::string& objectName)
{
	boost::shared_ptr<const gazebo_msgs::ModelStates_<std::allocator<void> > > pModelStates
		= ros::topic::waitForMessage<gazebo_msgs::ModelStates>("gazebo/model_states");

	if(pModelStates != NULL)
	{	
		for(size_t i = 0; i < pModelStates->name.size(); ++i){
			if(pModelStates->name[i] == objectName)
			{
				pose = pModelStates->pose[i];
			}
		}
	}
    return false;
}

/**
 * Retrieve the poses of all objects as a map of name-pose pairs 
 */
void getObjectPoses(std::set< std::string >* pObjectNameSet,
std::map< std::string, geometry_msgs::Pose >& objectNamePoseMap)
{
	boost::shared_ptr<const gazebo_msgs::ModelStates_<std::allocator<void> > > pModelStates
		= ros::topic::waitForMessage<gazebo_msgs::ModelStates>("gazebo/model_states");

	if(pModelStates != NULL)
	{	
		for(size_t i = 0; i < pModelStates->name.size(); ++i){
			if(pObjectNameSet == NULL || 
                pObjectNameSet->find(pModelStates->name[i]) != pObjectNameSet->end())
			{
                objectNamePoseMap[pModelStates->name[i]] = pModelStates->pose[i];
			}
		}
	}
}


/**
 * Retrieve the pose of the youbot base. We assume that the based will be on the 
 * ground so we are really retrieving a 2D pose 
 */
bool getYoubotBasePose(double& x, double& y, double& theta, const std::string& botName, bool print)
{
	boost::shared_ptr<const gazebo_msgs::ModelStates_<std::allocator<void> > > pModelStates
		= ros::topic::waitForMessage<gazebo_msgs::ModelStates>("gazebo/model_states");

	if(pModelStates != NULL)
	{	
		for(size_t i = 0; i < pModelStates->name.size(); ++i){
			if(pModelStates->name[i] == botName)
			{
				x = pModelStates->pose[i].position.x; 
				y = pModelStates->pose[i].position.y;
				theta = tf::getYaw(pModelStates->pose[i].orientation);
				if(print==true)
					ROS_INFO("Retrieved youbot pose [%f, %f, %f]", x, y, theta);	
                return true;	
			}
		}
	}
    return false;
}
geometry_msgs::Pose getYoubotCurrentPose(std::string robot_name){
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    arc::gazeboUtility::getYoubotBasePose(x, y, theta, robot_name);
    return arc::SE2ToGpose(arc::SE2Pose(x, y, theta));
}
/**
 * Set youbot base location 
 */
void setYoubotBasePose(ros::NodeHandle &node_handle_, double& x, double& y, 
    double& theta, const std::string& botName)
{
	ros::ServiceClient set_model_state_client_ 
		= node_handle_.serviceClient< gazebo_msgs::SetModelState> ("/gazebo/set_model_state");
	gazebo_msgs::SetModelState setModelState;

    setModelState.request.model_state.model_name = botName;
    setModelState.request.model_state.reference_frame = "world";
    setModelState.request.model_state.pose.position.x = x;
    setModelState.request.model_state.pose.position.y = y;
    setModelState.request.model_state.pose.position.z = 0.05;

    tf::Quaternion orientation = getPoseFromYaw(theta);

    setModelState.request.model_state.pose.orientation.x = orientation.getX();
    setModelState.request.model_state.pose.orientation.y = orientation.getY();
    setModelState.request.model_state.pose.orientation.z = orientation.getZ();
    setModelState.request.model_state.pose.orientation.w = orientation.getW();

    set_model_state_client_.call(setModelState);

}

void setObjectPose(ros::NodeHandle &node_handle_, double c_x, double c_y, 
    double c_z, double q_x, double q_y, double q_z, double q_w, const std::string name){
    ros::ServiceClient set_model_state_client_ 
        = node_handle_.serviceClient< gazebo_msgs::SetModelState> ("/gazebo/set_model_state");
    gazebo_msgs::SetModelState setModelState;

    setModelState.request.model_state.model_name = name;
    setModelState.request.model_state.reference_frame = "world";
    setModelState.request.model_state.pose.position.x = c_x;
    setModelState.request.model_state.pose.position.y = c_y;
    setModelState.request.model_state.pose.position.z = c_z;


    setModelState.request.model_state.pose.orientation.x = q_x;
    setModelState.request.model_state.pose.orientation.y = q_y;
    setModelState.request.model_state.pose.orientation.z = q_z;
    setModelState.request.model_state.pose.orientation.w = q_w;

    set_model_state_client_.call(setModelState);

}

}

}
