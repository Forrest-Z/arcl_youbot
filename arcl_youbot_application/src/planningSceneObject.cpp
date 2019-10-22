#include "arcl_youbot_application/planningSceneObject.hpp"

#include <boost/geometry/io/dsv/write.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>


namespace arc{

int PlanningSceneObject::OBJECT_ID_COUNTER = 1;

PlanningSceneObject::PlanningSceneObject(const std::string& name, const std::string& type,
double dx, double dy, double dz, const geometry_msgs::Pose& pose)
:object_name_(name),object_type_(type),last_pose_(pose),object_mesh_type_(BOX),mesh_filename_(""),
length_(dx),width_(dy),height_(dz),object_id_(OBJECT_ID_COUNTER++)
{
	computeFootprint();
}

PlanningSceneObject::PlanningSceneObject(const std::string& name, 
const std::string& mesh_filename, const geometry_msgs::Pose& pose)
:object_name_(name),last_pose_(pose),object_mesh_type_(MESH),
mesh_filename_(mesh_filename),object_id_(OBJECT_ID_COUNTER++)
{
	computeFootprint();
}

/**
 * Update object if it moved somewhat significantly 
 */
void PlanningSceneObject::updateObjectPose(const geometry_msgs::Pose&  pose){
	// If pose has not changed much, do nothing!
	if(fabs(pose.position.x - last_pose_.position.x) < 0.001 &&
		fabs(pose.position.y - last_pose_.position.y) < 0.001 &&
		fabs(pose.position.z - last_pose_.position.z) < 0.001 &&
		fabs(pose.orientation.x - last_pose_.orientation.x) < 0.01 &&
		fabs(pose.orientation.y - last_pose_.orientation.y) < 0.01 &&
		fabs(pose.orientation.z - last_pose_.orientation.z) < 0.01 &&
		fabs(pose.orientation.w - last_pose_.orientation.w) < 0.01)
		return;

	// Update pose
	last_pose_ = pose;

	// Update footprint

		computeFootprint();
	
}

void PlanningSceneObject::getObjectPose(geometry_msgs::Pose& pose){
	pose = last_pose_;
}



/**
 * Updating footprint of cubes. Logic: 
 * 1. Compute the transformed object (rotating the cube then translate 8 vertices)
 * 2. Collect vertices (only x and y) into boost::geometry::model::multi_point 
 * 	  and compute 2d comvex hull
 * 3. Go through the hull and remove points that are almost duplicates. 
 */
void PlanningSceneObject::computeFootprint()
{
	// Get oritentation 
	tf::Matrix3x3 rotationMatrix(tf::Quaternion(last_pose_.orientation.x, last_pose_.orientation.y, 
		last_pose_.orientation.z, last_pose_.orientation.w));

	// Rotate and translate the vertices 
	multi_point_2 points; 
	if(object_mesh_type_ == BOX){
		if(object_type_ == "cube"){ 
			tf::Vector3 vertices[8];
			for(int i = 0; i < 8; i ++){
				vertices[i] = rotationMatrix * tf::Vector3(((i&1)==1?1:-1)*0.5*length_, 
						((i&2)==2?1:-1)*0.5*width_, ((i&4)==4?1:-1)*0.5*height_);
				boost::geometry::append(points, point_2(vertices[i].x() + last_pose_.position.x, 
					vertices[i].y() + last_pose_.position.y));
			}
		}else if(object_type_ == "L_block"){
			double unit = 0.0324;
			tf::Vector3 vec;
			vec = rotationMatrix * tf::Vector3(-2*unit, -unit, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(-2*unit, unit, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(2*unit, unit, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(2*unit, 0, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(-unit, 0, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(-unit, -unit, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));

			bg::correct(object_footprint_);
		}else if(object_type_ == "T_block"){
			double unit = 0.0324;
			tf::Vector3 vec;
			vec = rotationMatrix * tf::Vector3(-0.5*unit, -2.5*unit, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(-1.5*unit, -2.5*unit, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(-1.5*unit, 2.5*unit, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(-0.5*unit, 2.5*unit, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(-0.5*unit, 0.5*unit, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(1.5*unit, 0.5*unit, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(1.5*unit, -0.5*unit, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(-0.5*unit, -0.5*unit, 0);
			boost::geometry::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			bg::correct(object_footprint_);
			
		}else if(object_type_ == "wall"){
			
			double dx = length_;
			double dy = width_;
			double dz = height_;
			tf::Vector3 vec;
			vec = rotationMatrix * tf::Vector3(-0.5*dx, -0.5*dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(-0.5*dx, 0.5*dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(0.5*dx, 0.5*dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(0.5*dx, -0.5*dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			bg::correct(object_footprint_);
		}else if(object_type_ == "floor"){
			double dx = 0.1;
			double dy = 0.1;
			double dz = 0.01;
			tf::Vector3 vec;
			vec = rotationMatrix * tf::Vector3(-0.5*dx, -0.5*dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(-0.5*dx, 0.5*dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(0.5*dx, 0.5*dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(0.5*dx, -0.5*dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			bg::correct(object_footprint_);
		}else if(object_type_ == "pillar"){
			double dx = length_;
			double dy = width_;
			double dz = height_;
			tf::Vector3 vec;
			vec = rotationMatrix * tf::Vector3(-dx, 0, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(-0.707*dx, 0.707*dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(0.0, dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(0.707*dx, 0.707*dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(dx, 0, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(0.707*dx, -0.707*dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(0.0, -dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			vec = rotationMatrix * tf::Vector3(-0.707*dx, -0.707*dy, 0);
			bg::append(object_footprint_, point_2(vec.getX()+last_pose_.position.x, vec.getY()+ last_pose_.position.y));
			bg::correct(object_footprint_);
		}

	}


	// Construct convex hull for all points
	if(object_type_ == "cube"){ 
		polygon_2 poly;
		bg::convex_hull(points, poly);

		// Remove duplicates from the hull 
		std::vector<point_2> const& hullPoints = poly.outer();
		point_2 tempPoint;
		object_footprint_.clear();
		for(int i = 0; i < hullPoints.size(); i ++){
			if(i == 0){
				boost::geometry::append(object_footprint_,hullPoints[i]);
				tempPoint = hullPoints[i];
			}
			else{
				if((fabs(hullPoints[i].get<0>() - tempPoint.get<0>()) < 0.002 && 
					fabs(hullPoints[i].get<1>() - tempPoint.get<1>()) < 0.002) ||
					(fabs(hullPoints[i].get<0>() - hullPoints[0].get<0>()) < 0.002 && 
					fabs(hullPoints[i].get<1>() - hullPoints[0].get<1>()) < 0.002))
				{
					continue;
				}
				else{
					boost::geometry::append(object_footprint_,hullPoints[i]);
					tempPoint = hullPoints[i];
				}
			}
		}
		bg::correct(object_footprint_);
	}
	// Update the bounding box
	object_footprint_bounding_box_ = 
		bg::return_envelope<box_2, polygon_2>(object_footprint_);

    // std::cout << object_name_ << boost::geometry::dsv(object_footprint_) << std::endl;
    // std::cout << object_name_ << boost::geometry::dsv(object_footprint_bounding_box_) << std::endl;

}

}

