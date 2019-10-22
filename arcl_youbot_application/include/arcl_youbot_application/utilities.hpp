#ifndef _ARC_UTILITIES__
#define _ARC_UTILITIES__

// C++ 
#include <cstddef>
#include <iostream>

// boost::geometry library
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/multi/geometries/multi_point.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/strategies/cartesian/point_in_poly_franklin.hpp>

// ROS related
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>

//luh_youbot dependency
#include <arcl_youbot_kinematics/arm_kinematics.h>


// ompl
// #include <ompl/base/SpaceInformation.h>
// #include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/base/objectives/StateCostIntegralObjective.h>
// #include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
// #include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <ompl/base/spaces/RealVectorBounds.h>
// #include <ompl/base/spaces/SE2StateSpace.h>
// #include <ompl/util/Console.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
// #include <ompl/geometric/planners/rrt/RRTConnect.h>

//#define SPECIAL_GREEDY
#define NUM_ROBOTS 2
//#define TWO_ROBOT
#define SINGLE_EXIT
//#define MULTI_EXIT
#define MULTI_COLOR
//#define STACK
//#define WITH_OBS
#define CHECK_COLLISION_INTERVAL 0.1
//#define USE_RRT_STAR
#define USE_VISI

// namespace ob = ompl::base;
// namespace og = ompl::geometric;
namespace bg = boost::geometry;
namespace ykin = arcl_youbot_kinematics;
namespace arc
{

// Geometric primitives
typedef bg::model::point<double, 2, bg::cs::cartesian> point_2;
typedef bg::model::point<double, 3, bg::cs::cartesian> point_3;

typedef bg::model::multi_point<point_2> multi_point_2;
typedef bg::model::polygon<point_2> polygon_2;
typedef bg::model::segment<point_2> segment_2;
typedef bg::model::box<point_2> box_2;

typedef bg::model::point<double, 3, bg::cs::cartesian> point_se2;
typedef bg::strategy::within::franklin<point_2, point_2, void> fran;
// Translating and rotating a polygon from the origin 
void movePolygon(const polygon_2& poly, polygon_2& outPoly, double x, 
    double y, double theta);

// Compute SE2 distance 
double computeSE2Distance(double x_1, double y_1, double t_1, double x_2, double y_2, double t_2);
double computeSE2Distance(point_se2& p1, point_se2 p2);

enum optimalPlanner
{
    PLANNER_BFMTSTAR,
    PLANNER_BITSTAR,
    PLANNER_CFOREST,
    PLANNER_FMTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_SORRTSTAR,
};


// An enum of the supported optimization objectives, alphabetical order
enum planningObjective
{
    OBJECTIVE_PATHCLEARANCE,
    OBJECTIVE_PATHLENGTH,
    OBJECTIVE_THRESHOLDPATHLENGTH,
    OBJECTIVE_WEIGHTEDCOMBO
};

struct SE2Pose
{
	double x;
	double y;
	double yaw;

    SE2Pose(){};
    SE2Pose(double x_, double y_, double yaw_)
    {
        x = x_;
        y = y_;
        yaw = yaw_;
    }
    SE2Pose(const SE2Pose& a)
    {
        x = a.x;
        y = a.y;
        yaw = a.yaw;
    }
};

geometry_msgs::Pose assignGeometryPose(double x, double y, double z, double q_x, double q_y, double q_z, double q_w);

// ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

SE2Pose GPoseToSE2(geometry_msgs::Pose pose);

double getYawFromPose(tf::Matrix3x3 m);

geometry_msgs::Pose SE2ToGpose(SE2Pose);

tf::Quaternion getPoseFromYaw(double yaw);

double getYawFromPose(tf::Quaternion q);

double angleToRotateToTarget(tf::Matrix3x3 target_matrix_, tf::Matrix3x3 current_matrix);

void convertGlobalMoveToLocalMove(tf::Matrix3x3 current_matrix, double &global_move_x, double &global_move_y, double &local_move_x, double &local_move_y);

tf::Matrix3x3 GetMatrixFromPose(geometry_msgs::Pose pose);

double angleToRotateToTarget(geometry_msgs::Pose target, tf::Matrix3x3 current_matrix);

double getYawFromPolygon(polygon_2 poly);

tf::Quaternion getChangedPoseFromYaw(double yaw);
tf::Quaternion getPoseFromPolygon(polygon_2 poly);

double getDistance(std::pair<double, double> pt_0, std::pair<double, double> pt_1);
double getDistance(std::tuple<double, double, double> pt_0, std::tuple<double, double, double> pt_1);
bool isPointVeryClose(double limit, std::tuple<double, double, double> pt_0, std::tuple<double, double, double> pt_1);
bool isTwoSegmentClose(arc::segment_2 seg0, arc::segment_2 seg1, double dist);

};

#endif // _ARC_UTILITIES__