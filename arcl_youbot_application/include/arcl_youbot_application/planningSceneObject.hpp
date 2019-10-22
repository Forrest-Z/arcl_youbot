#ifndef __ARC_PLANNING_SCENCE_OBJECT__
#define __ARC_PLANNING_SCENCE_OBJECT__ 

/**
 * Class representing an object (e.g., obstacles) in the scence. 
 */

#include <ros/ros.h>
#include <vector>
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"

#include "utilities.hpp"

namespace arc{

/**
 * Basic scene object that can be inherited to be more specific types of objects
 */
class PlanningSceneObject
{
    public:
		// Object construction type
		enum PlanningSceneObjectMeshType{
			BOX,		// A cubiod
			MESH		// A mesh based object
		};

		// Object name and ID
		const std::string object_name_;
		const int object_id_;
		const std::string object_type_;
		// Mesh file name 
		const std::string mesh_filename_;			

		// Footprint and the bounding box for the footprint
		polygon_2 object_footprint_;
		box_2 object_footprint_bounding_box_;
		double length_;
		double width_;
		double height_;
		// For constructing a box object
		PlanningSceneObject(const std::string& name, const std::string& type, double dx, double dy, double dz, const geometry_msgs::Pose& pose); 

		// For constructing a mesh based object
		PlanningSceneObject(const std::string& name, const std::string& mesh_filename, const geometry_msgs::Pose& pose);

		// Retrive a reference to object footprint, useful for colision checking with 
		// moving robot base 
		// std::vector<polygon_2> & get_object_footprint_(){return object_footprint_;}
		polygon_2 & get_object_footprint_(){return object_footprint_;}

		// Retrieve footprint bounding box 
		box_2 & get_object_footprint_bounding_box_(){return object_footprint_bounding_box_;}

		// Update object pose
		void updateObjectPose(const geometry_msgs::Pose& pose);

		void getObjectPose(geometry_msgs::Pose& pose);
    protected: 
		// Object type 
		PlanningSceneObjectMeshType object_mesh_type_;

		// For box object
		

		// vector of polygons storing the foot print of the object, for future use
		// std::vector<polygon_2> object_footprint_;

		// Last pose 
		geometry_msgs::Pose last_pose_;

		// Compute footprint
		void computeFootprint();

	private: 
		// A static id that will be assigned uniquely to objects
		static int OBJECT_ID_COUNTER;
};

}

#endif //  __ARC_PLANNING_SCENCE_OBJECT__ 