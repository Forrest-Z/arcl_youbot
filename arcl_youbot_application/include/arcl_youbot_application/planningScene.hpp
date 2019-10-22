#ifndef __ARC_PLANNING_SCENCE__
#define __ARC_PLANNING_SCENCE__ 

/**
 * Class representing the planning scene incluidng all objects that are 
 * relevant to the planning task. 
 */

#include <ros/ros.h>
#include <iostream>
#include <ios>
#include "planningSceneObject.hpp"
#include "arcl_youbot_application/PlanningSceneMsg.h"
#include <fstream>
#include "gazeboUtility.hpp"
#include <tuple>
namespace arc{

class PlanningScene
{
#define COLLISION_CELLS_X 50
#define COLLISION_CELLS_Y 50

	protected:
		std::vector<PlanningSceneObject*>  scene_object_list_;
		std::set<std::string>  scene_object_name_set_;

		double x_min_;
		double x_max_;
		double y_min_;
		double y_max_;

		ros::NodeHandle &node_handle_;

		// For collision hashing
		polygon_2 cell_bounding_polygon_[COLLISION_CELLS_X][COLLISION_CELLS_Y];
		std::vector<PlanningSceneObject*> scene_object_pointers_[COLLISION_CELLS_X][COLLISION_CELLS_Y];

	public:
		PlanningScene(ros::NodeHandle &node_handle, double xMin, double xMax, 
			double yMin, double yMax);
		virtual ~PlanningScene();
		
		// Retrieving bounds
		double getXMin(){return x_min_;}
		double getXMax(){return x_max_;}
		double getYMin(){return y_min_;}
		double getYMax(){return y_max_;}
		void printDebugInfo();
		
		// Create gazebo test scence with a bunch of cuboids
		void createGazeboYoubotTestScene(int numberOfObjects,
			double length, double width,  double dx = 0.032, double dy = 0.032, double dz = 0.15);
		
		std::vector<std::pair<int, std::string>> createGazeboYoubotTestSceneFromFile(int numberOfObjects, double length, double width, double dx, double dy, double dz, std::string filename);

		std::vector<std::tuple<int, int, std::string>> createGazeboYoubotTestSceneFromFileMulti(int numberOfObjects, double length, double width, double dx, double dy, double dz, std::string filename);

		std::map<int, std::vector<std::pair<int, std::string>>> createGazeboYoubotTestSceneFromFileTwoRobots(int numberOfObjects, double length, double width, double dx, double dy, double dz, std::string filename);

		// Scene cleanup
		void cleanGazeboYoubotTestScene();

		// Retrieve and update scene objects and robot’s newest pose
		void updateScene();

		// Retrieve and update scene objects and robot’s newest pose
		void updateCollisionHash();

		// Whether there is collision between objects and robots given current poses
		bool isCollisionFree(const polygon_2& robot, bool debugInfo = false);    

		bool isCollisionFreeWithExceptions(const polygon_2& robot, std::vector<std::string> exception_names,bool debugInfo = false);
	

		// Whether there is collision between objects and robots given current poses
		bool isCollisionFreeSlow(const polygon_2& robot);    

        // Locate a collision free pose
        void findCollsionFreePose(const polygon_2& robot, 
	        double &x, double &y, double &t, double dx, double dy);

        // Testing of collision checking capabilities 
        void collisionCheckTest(const polygon_2& robot, int numberOfChecks);

		void InitFromMsg(arcl_youbot_application::PlanningSceneMsg msg);
		
		void getSceneObjectList(std::vector<PlanningSceneObject*> &scene_object_list);

	protected:
		// Compute the affected indices in the collision structure for a given bounding box
		void getCollisionArrayIndices(int& minXI, int& maxXI, int& minYI, int& maxYI,
			double minX, double maxX, double minY, double maxY);
};

}

#endif // __ARC_PLANNING_SCENCE__ 