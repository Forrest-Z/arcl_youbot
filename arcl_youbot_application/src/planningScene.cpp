
/**
 * Class representing the planning scene incluidng all objects that are 
 * relevant to the planning task. 
 */

#include <ros/ros.h>
#include "arcl_youbot_application/planningScene.hpp"
#include "arcl_youbot_application/gazeboUtility.hpp"
#include "arcl_youbot_application/utilities.hpp"
#include <time.h>
#include <boost/geometry/io/dsv/write.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>

namespace arc{

PlanningScene::PlanningScene(ros::NodeHandle &node_handle, double xMin, double xMax, 
	double yMin, double yMax)
:node_handle_(node_handle), x_min_(xMin), x_max_(xMax), y_min_(yMin), y_max_(yMax)
{
	// Construct polygon for collsion checking 
	double xInc = (x_max_ - x_min_)/COLLISION_CELLS_X;
	double yInc = (y_max_ - y_min_)/COLLISION_CELLS_Y;
	for(int ix = 0; ix < COLLISION_CELLS_X; ix ++)
	{
		for(int iy = 0; iy < COLLISION_CELLS_Y; iy ++)
		{
			double minX = x_min_ + xInc*ix;
			double maxX = x_min_ + xInc*(ix+1);
			double minY = y_min_ + yInc*iy;
			double maxY = y_min_ + yInc*(iy+1);
			cell_bounding_polygon_[ix][iy].outer().push_back(point_2(minX, minY));
			cell_bounding_polygon_[ix][iy].outer().push_back(point_2(minX, maxY));
			cell_bounding_polygon_[ix][iy].outer().push_back(point_2(maxX, maxY));
			cell_bounding_polygon_[ix][iy].outer().push_back(point_2(maxX, minY));
			bg::correct(cell_bounding_polygon_[ix][iy]);

			// std::cout << minX << " " << maxX << " " << minY << " " << maxY << std::endl;
		}
	}
}

PlanningScene::~PlanningScene()
{
	ROS_WARN_STREAM("~PlanningScene");
	for(std::vector<PlanningSceneObject*>::iterator it = scene_object_list_.begin();
		it != scene_object_list_.end(); it++){
		delete *it;
	}

}

void PlanningScene::InitFromMsg(arcl_youbot_application::PlanningSceneMsg msg)
{

	for(int i = 0; i < msg.scene_object_list.size(); i ++)
	{
		scene_object_list_.push_back(new PlanningSceneObject(msg.scene_object_list[i].object_name.data, msg.scene_object_list[i].object_type.data,msg.scene_object_list[i].dx, msg.scene_object_list[i].dy, msg.scene_object_list[i].dz, msg.scene_object_list[i].object_pose));
		scene_object_name_set_.insert(msg.scene_object_list[i].object_name.data);
		
	}
	
}

void PlanningScene::printDebugInfo()
{
	ROS_WARN_STREAM("DEBUG PlanningScene:");
	for(auto i = scene_object_list_.begin();i != scene_object_list_.end();i ++){
		geometry_msgs::Pose temp;
		(*i)->getObjectPose(temp);
		ROS_WARN_STREAM("object name:"<<(*i)->object_name_<<", object pose:"<<temp.position.x<<","<<temp.position.y<<","<<temp.position.z<<", size:"<<(*i)->length_<<","<<(*i)->width_<<","<<(*i)->height_);
	}
}


// void PlanningScene::createGazeboSceneFromFile(std::string env_filename){
// 	std::ifstream ifs(env_filename);
// 	int numberOfObjects = 0;
// 	ifs >> numberOfObjects;
// 	for(int i = 0; i < numberOfObjects; i++){
// 		double pos_x, pos_y;
// 		int numberOfVertex = 0;
// 		ifs >> numberOfVertex;
// 		for(int j = 0; j < numberOfVertex; j++){
			
// 		}
// 		arc::gazeboUtility::spawnCuboid(node_handle_, 0.1, 0.032, 0.032, 0.15, 
//             (rand() - RAND_MAX/2.)/RAND_MAX*length,
//             (rand() - RAND_MAX/2.)/RAND_MAX*width,
//            	0.016,
//             (rand() - RAND_MAX/2.)/RAND_MAX, 
//             (rand() - RAND_MAX/2.)/RAND_MAX,
//             (rand() - RAND_MAX/2.)/RAND_MAX,
//             1,
//             arc::gazeboUtility::GAZEBO_COLORS[i%10],
//             name);		
// 	}
// }
/**
 * Create gazebo test scence with a bunch of cuboids
 */
void PlanningScene::createGazeboYoubotTestScene(
int numberOfObjects, double length, double width, double dx, double dy, double dz)
{
	// // Spawn some cuboids and create the scence
	// for(int i = 0; i < numberOfObjects; i ++){
	// 	std::stringstream ss; 
	// 	ss << "cuboid_" << (i+1);
	// 	std::string name = ss.str();

	// 	arc::gazeboUtility::spawnCuboid(node_handle_, 0.1, dx, dy, dz, 
    //         (rand() - RAND_MAX/2.)/RAND_MAX*length,
    //         (rand() - RAND_MAX/2.)/RAND_MAX*width,
    //         (rand())*1.0/RAND_MAX + 0.2,
	// 		// 0,
	// 		// 0,
	// 		// 0,
    //         (rand() - RAND_MAX/2.)/RAND_MAX, 
    //         (rand() - RAND_MAX/2.)/RAND_MAX,
    //         (rand() - RAND_MAX/2.)/RAND_MAX,
    //         1,
    //         arc::gazeboUtility::GAZEBO_COLORS[i%10],
    //         name);		
	// 	geometry_msgs::Pose pose;
	// 	arc::gazeboUtility::getObjectPose(pose, name);
	// 	scene_object_list_.push_back(new PlanningSceneObject(name, "cube",dx, dy, dz, pose));
	// 	scene_object_name_set_.insert(name);
	// }
}

std::map<int, std::vector<std::pair<int, std::string>>> PlanningScene::createGazeboYoubotTestSceneFromFileTwoRobots
(int numberOfObjects, double length, double width, double dx, double dy, double dz, std::string filename)
{
	std::vector<int> color_sequence = {0, 4, 3, 10, 9, 4, 3, 2, 3, 8,2,1,3, 5, 10, 4, 6, 3, 1, 2, 9, 8, 4, 4, 1};
	int color_index = 0;
	int m_numRobots = 0;
	// Clean up
	std::vector<arc::polygon_2> polygon_list;

	const char* c_filename = filename.c_str();
	// Reading in the environment, first the raidus of the (disc) robot
	std::ifstream ifs(c_filename);
	double radius;
	ifs >> radius;

	// Then the number of obstacle polygons
	int numberOfPolygons = 0;
	ifs >> numberOfPolygons;
	m_numRobots = numberOfPolygons;
	polygon_list.clear();
	ROS_WARN_STREAM("polygon_list size:"<<numberOfPolygons);
	// Then read in all obstacle polygons and compute the configuration space for a single disc
	for (int i = 0; i < numberOfPolygons; i++) {
		// Load polygon
		arc::polygon_2 tp;
		int numberOfVertex;
		ifs >> numberOfVertex;
		//Point_2 firstOne_1;
		for (int j = 0; j < numberOfVertex; j++) {
			arc::point_2 p;
			double p_x, p_y;
			ifs >> p_x >> p_y;
			p_x = p_x / 400.0;
			p_y = 2000 - p_y;
			p_y = p_y / 400.0;
			p_x = (p_x - 2.5) ;
			//p_y = (p_y - 1.175) ;
			// p_x = p_x / 500.0;
			// p_y = p_y / 500.0;
			// p_x = (p_x - 2) ;
			// p_y = (p_y - 2) ;
			p.set<0>(p_x);
			p.set<1>(p_y);
			bg::append(tp.outer(), p);
		}

		// bg::append(tp.outer(), firstOne_1);
		bg::correct(tp);
		polygon_list.push_back(tp);


		// // Add raw obstacle to scene and set fill to be true with fill transparency
		// AdvancedGraphicsItem<Polygon_2>* pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envPolyList.back()));

		// // m_scene.addItem(pAGI);
		// pAGI->m_bShowEdge = true;
		// pAGI->m_bShowVertices = true;
		// pAGI->m_bFill = false;
		// pAGI->m_fillBrush = QColor(16, 16, 16, 192);
		// m_PolyAGIList.push_back(pAGI);

		// Computing the Minkowski sum
		// int split_num = 8;
		// Polygon_2 ep = growPolygonByRadius(tp, radius, split_num);
		// bg::correct(ep);
		// //split_num = 4;
		// //Polygon_2 vp = growPolygonByRadius(tp, radius, split_num);
		// //bg::correct(vp);

		// m_envPolyVoronoiList.push_back(ep);
		// m_envObsPolyList.push_back(ep);

		// // Add the configuration space obstacle to the scene
		// pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envObsPolyList.back()));

		// // m_scene.addItem(pAGI);
		// pAGI->m_bFill = true;
		// pAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		// pAGI->m_vertexPen = QPen(Qt::black, 0.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		// m_PolyAGIList.push_back(pAGI);
	}


	// Then read the bounding rectangle (configuration space)
	int numberOfBoundingVertex;
	arc::polygon_2 m_boundingPoly;
	ifs >> numberOfBoundingVertex;
	for (int j = 0; j < numberOfBoundingVertex; j++) {
		arc::point_2 p;
		double p_x, p_y;
		ifs >> p_x >> p_y;
		
		ROS_WARN_STREAM("point");
		p.set<0>(p_x);
		p.set<1>(p_y);
		bg::append(m_boundingPoly.outer(), p);
	}
	bg::correct(m_boundingPoly);

	// Add to scence

	// m_pBoundingPolyAGI = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly);
	// m_pBoundingPolyAGI->m_bFill = false;
	// m_pBoundingPolyAGI->m_bShowVertices = false;
	// m_pBoundingPolyAGI->m_bShowEdge = true;
	// m_pBoundingPolyAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	// m_PolyAGIList.push_back(m_pBoundingPolyAGI);
	// m_scene.addItem(m_pBoundingPolyAGI);

	// Then compute the outside bounding rectangle
	// double x1, y1, x2, y2;
	// std::vector<Point_2> const& points = m_boundingPoly.outer();

	// x1 = points[0].get<0>(); y1 = points[0].get<1>();
	// x2 = points[2].get<0>(); y2 = points[2].get<1>();
	// bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	// bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y2 + radius));
	// bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y2 + radius));
	// bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y1 - radius));
	// //	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	// bg::correct(m_boundingPoly2);
	// // Add to scene
	// m_pBoundingPolyAGI2 = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly2);
	// m_pBoundingPolyAGI2->m_bFill = false;
	// m_pBoundingPolyAGI2->m_bShowVertices = false;
	// m_pBoundingPolyAGI2->m_bShowEdge = true;;
	// m_pBoundingPolyAGI2->m_edgePen = QPen(Qt::black, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	// m_PolyAGIList.push_back(m_pBoundingPolyAGI2);
	// // m_scene.addItem(m_pBoundingPolyAGI2);

	// m_radius = radius;

	// drawBasicEnvironment();

	// // Do roadmap building setup
	// m_roadmap.buildRoadmap(&m_envObsPolyList, &m_envPolyList, &m_boundingPoly, &m_envPolyVoronoiList, radius, m_scene);
	// // m_roadmap.addToScene(m_scene);

	// m_boundingRect = QRectF(x1 - radius * 4, y1 - radius * 4, x2 + radius * 4, y2 + radius * 4);
	std::map<int, std::vector<std::pair<int, std::string>>> picking_sequence;
	int robot_index = 0;
	ifs >> robot_index;
	robot_index = 0 - robot_index;
	while(1){ 
		std::pair<int, std::string> p;
		ifs >> p.first;
		if(p.first > 0){ 
			picking_sequence[robot_index].push_back(p);
		}else{
			robot_index = - p.first;
			break;
		}
	}
	while(1){ 
		std::pair<int, std::string> p;
		ifs >> p.first;
		if(p.first > 0){ 
			picking_sequence[robot_index].push_back(p);
		}else{
			break;
		}
	}


	std::vector<arc::polygon_2> generated_polys;
	// Spawn some cuboids and create the scence
	for(int i = 0; i < numberOfPolygons; i ++){
		std::stringstream ss; 
		ss << "cuboid_" << (i+1);
		std::string name = ss.str();
		arc::polygon_2 current_poly = polygon_list[i];
		std::string object_type;
			
		for(int j = 0; j < picking_sequence[1].size(); j++){
			if(picking_sequence[1][j].first == (i+1)){
				if(current_poly.outer().size() == 9){
					picking_sequence[1][j].second = "T_block";
				}else if(current_poly.outer().size() == 7){
					picking_sequence[1][j].second = "L_block";
				}else if(current_poly.outer().size() == 5){
					picking_sequence[1][j].second = "cube";
				}
			}
		}
		for(int j = 0; j < picking_sequence[2].size(); j++){
			if(picking_sequence[2][j].first == (i+1)){
				if(current_poly.outer().size() == 9){
					picking_sequence[2][j].second = "T_block";
				}else if(current_poly.outer().size() == 7){
					picking_sequence[2][j].second = "L_block";
				}else if(current_poly.outer().size() == 5){
					picking_sequence[2][j].second = "cube";
				}
			}
		}		
		geometry_msgs::Pose pose;
		ROS_WARN_STREAM("current poly size:"<<current_poly.outer().size());
		if(current_poly.outer().size() == 9){
		// T shape
			geometry_msgs::Pose T_pose;
			T_pose.position.x = ((current_poly.outer()[0].get<0>() + current_poly.outer()[7].get<0>())/2.0 + (current_poly.outer()[5].get<0>() + current_poly.outer()[6].get<0>())/2.0) / 2.0;
			T_pose.position.y = ((current_poly.outer()[0].get<1>() + current_poly.outer()[7].get<1>())/2.0 + (current_poly.outer()[5].get<1>() + current_poly.outer()[6].get<1>())/2.0) / 2.0;
			T_pose.position.z = 0.0324/2.0;

    		tf::Quaternion q = arc::getPoseFromPolygon(current_poly);
	    	double q_x = q.x();
    		double q_y = q.y();
    		double q_z = q.z();
    		double q_w = q.getW();
			T_pose.orientation.x = q_x;
			T_pose.orientation.y = q_y;
			T_pose.orientation.z = q_z;
			T_pose.orientation.w = q_w;
			if(color_sequence[color_index] == 10)
				arc::gazeboUtility::spawnURDFModel(node_handle_, "/home/wei/catkin_gazebo_ws/src/luh_youbot_description/urdf/model_T_orange.urdf", T_pose, name);
			else{  
				arc::gazeboUtility::spawnURDFModel(node_handle_, "/home/wei/catkin_gazebo_ws/src/luh_youbot_description/urdf/model_T_red.urdf", T_pose, name);
			}
			//arc::gazeboUtility::spawnSDFModel(node_handle_, "/home/wei/.gazebo/models/006_mustard_bottle/model_T.sdf", T_pose, name);
			object_type = "T_block";
			pose = T_pose;
			ROS_WARN_STREAM("object:"<<name<<", quatenion:"<<q_x<<","<<q_y<<","<<q_z<<","<<q_w);

		}else if(current_poly.outer().size() == 7){
		//L shape
			geometry_msgs::Pose L_pose;
			L_pose.position.x = (current_poly.outer()[1].get<0>() + current_poly.outer()[3].get<0>())/2.0;
			L_pose.position.y = (current_poly.outer()[1].get<1>() + current_poly.outer()[3].get<1>())/2.0;
			L_pose.position.z = 0.0324/2.0;


    		tf::Quaternion q = arc::getPoseFromPolygon(current_poly);
    		double q_x = q.x();
    		double q_y = q.y();
    		double q_z = q.z();
    		double q_w = q.getW();

			L_pose.orientation.x = q_x;
			L_pose.orientation.y = q_y;
			L_pose.orientation.z = q_z;
			L_pose.orientation.w = q_w;
			if(color_sequence[color_index] == 3)
				arc::gazeboUtility::spawnURDFModel(node_handle_, "/home/wei/catkin_gazebo_ws/src/luh_youbot_description/urdf/model_L_green.urdf", L_pose, name);
			else{  
				arc::gazeboUtility::spawnURDFModel(node_handle_, "/home/wei/catkin_gazebo_ws/src/luh_youbot_description/urdf/model_L_yellow.urdf", L_pose, name);
			}
			object_type = "L_block";
			pose = L_pose;
			ROS_WARN_STREAM("object:"<<name<<", quatenion:"<<q_x<<","<<q_y<<","<<q_z<<","<<q_w);
		}else if(current_poly.outer().size() == 5){
		//rect
			bool higher = false;
			for(int k = 0; k < generated_polys.size(); k ++){
				if(bg::intersects(current_poly, generated_polys[k])){
					higher = true;
					break;
				}

			}
			double long_x, long_y, short_x, short_y;
	        double long_seg, short_seg;
	        double temp_seg = 0;
	        double temp_x, temp_y;
	        double yaw = 0;
	        long_x = current_poly.outer()[0].get<0>() - current_poly.outer()[3].get<0>();
	        long_y = current_poly.outer()[0].get<1>() - current_poly.outer()[3].get<1>();
	        long_seg = std::sqrt(long_x*long_x + long_y*long_y);

	        short_x = current_poly.outer()[0].get<0>() - current_poly.outer()[1].get<0>();
	        short_y = current_poly.outer()[0].get<1>() - current_poly.outer()[1].get<1>();
	        short_seg = std::sqrt(short_x*short_x + short_y*short_y);

	        if(short_seg > long_seg){
	            temp_seg = short_seg;
	            temp_x = short_x;
	            temp_y = short_y;
	            short_seg = long_seg;
	            short_x = long_x;
	            short_y = long_y;
	            long_seg = temp_seg;
	            long_x = temp_x;
	            long_y = temp_y;

	        }
			short_seg = 0.0376; // suitable for the youbot gripper wodth

			if(!higher){ 
				arc::gazeboUtility::spawnCuboid(node_handle_, 0.3, short_seg, short_seg, long_seg, 
	            current_poly,
	            arc::gazeboUtility::GAZEBO_COLORS[color_sequence[color_index]],     
	            name);	
	            ROS_WARN_STREAM("object:"<<name<<" is on the ground");
			}else{
				arc::gazeboUtility::spawnCuboid(node_handle_, 0.3, short_seg, short_seg, long_seg, 
	            current_poly,
	            arc::gazeboUtility::GAZEBO_COLORS[rand()%10],     
	            name, 0.05);
	            ROS_WARN_STREAM("object:"<<name<<" is higher");

			}
			generated_polys.push_back(current_poly);
			object_type = "cube";
			arc::gazeboUtility::getObjectPose(pose, name);

			dx = short_seg;
			dy = short_seg;
			dz = long_seg;
			ROS_WARN_STREAM("object:"<<name<<" size:"<<dx<<","<<dy<<","<<dz);
		}
		
		color_index ++;
		scene_object_list_.push_back(new PlanningSceneObject(name, object_type, dx, dy, dz, pose));
		scene_object_name_set_.insert(name);
	}

	

		


		std::string name = "wall_0";
		geometry_msgs::Pose pose;
		pose = arc::assignGeometryPose(-1.65, -0.1, 0.075, 0, 0, 0, 1);
		dx = 1.9;
		dy = 0.144018;
		dz = 0.3;
		PlanningSceneObject * wall_0 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
		scene_object_list_.push_back(wall_0);
		scene_object_name_set_.insert(name);
		name = "wall_1";
		pose = arc::assignGeometryPose(1.65, -0.1, 0.075, 0, 0, 0, 1);
		dx = 1.9;
		dy = 0.144018;
		dz = 0.3;
		PlanningSceneObject * wall_1 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
		scene_object_list_.push_back(wall_1);
		scene_object_name_set_.insert(name);
		name = "wall_2";     
		pose = arc::assignGeometryPose(2.5, 2.5, 0.1500001, 0, 0, 0.7071068, 0.7071068);
		dx = 5;
		dy = 0.144018;
		dz = 0.3;
		PlanningSceneObject * wall_2 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
		scene_object_list_.push_back(wall_2);
		scene_object_name_set_.insert(name);
		name = "wall_3";       
		pose = arc::assignGeometryPose(-2.5, 2.5, 0.15, 0, 0, 0.7071068, 0.7071068);
		dx = 5;
		dy = 0.144018;
		dz = 0.3;
		PlanningSceneObject * wall_3 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
		scene_object_list_.push_back(wall_3);
		scene_object_name_set_.insert(name);
		name = "wall_4";    
		pose = arc::assignGeometryPose(0, 5 ,0.075, 0,0,0,1);
		dx = 5;       
		dy = 0.14227;
		dz = 0.3;
		PlanningSceneObject * wall_4 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
		scene_object_list_.push_back(wall_4);
		scene_object_name_set_.insert(name);
		
		#ifdef WITH_OBS
		name = "pillar_0";
		pose = arc::assignGeometryPose(-0.7, -0.7, 0.2, 0, 0, 0, 1);
		dx = 0.1;
		dy = 0.1;
		dz = 0.4;
		PlanningSceneObject * pill_0 = new PlanningSceneObject(name, "pillar", dx, dy, dz, pose);
		scene_object_list_.push_back(pill_0);
		scene_object_name_set_.insert(name);
		name = "pillar_1";
		pose = arc::assignGeometryPose(-0.08, 0.2, 0.2, 0, 0, 0, 1);
		dx = 0.1;
		dy = 0.1;
		dz = 0.4;
		PlanningSceneObject * pill_1 = new PlanningSceneObject(name, "pillar", dx, dy, dz, pose);
		scene_object_list_.push_back(pill_1);
		scene_object_name_set_.insert(name);
		name = "pillar_2";
		pose = arc::assignGeometryPose(0.8, 0.5, 0.2, 0, 0, 0, 1);
		dx = 0.1;
		dy = 0.1;
		dz = 0.4;
		PlanningSceneObject * pill_2 = new PlanningSceneObject(name, "pillar", dx, dy, dz, pose);
		scene_object_list_.push_back(pill_2);
		scene_object_name_set_.insert(name);
		name = "pillar_3";
		pose = arc::assignGeometryPose(-0.5, 0.5, 0.2, 0, 0, 0, 1);
		dx = 0.1;
		dy = 0.1;
		dz = 0.4;
		PlanningSceneObject * pill_3 = new PlanningSceneObject(name, "pillar", dx, dy, dz, pose);
		scene_object_list_.push_back(pill_3);
		scene_object_name_set_.insert(name);
		#endif
	return picking_sequence;
}


std::vector<std::pair<int, std::string>> PlanningScene::createGazeboYoubotTestSceneFromFile
(int numberOfObjects, double length, double width, double dx, double dy, double dz, std::string filename)
{
	std::vector<int> color_sequence = {0, 4, 3, 10, 9, 4, 3, 2, 3, 8,2,1,3, 5, 10, 4, 6, 3, 1, 2, 9, 8, 4, 4, 1};
	int color_index = 0;
	int m_numRobots = 0;
	// Clean up
	std::vector<arc::polygon_2> polygon_list;

	const char* c_filename = filename.c_str();
	// Reading in the environment, first the raidus of the (disc) robot
	std::ifstream ifs(c_filename);
	double radius;
	ifs >> radius;

	// Then the number of obstacle polygons
	int numberOfPolygons = 0;
	ifs >> numberOfPolygons;
	m_numRobots = numberOfPolygons;
	polygon_list.clear();
	ROS_WARN_STREAM("polygon_list size:"<<numberOfPolygons);
	// Then read in all obstacle polygons and compute the configuration space for a single disc
	for (int i = 0; i < numberOfPolygons; i++) {
		// Load polygon
		arc::polygon_2 tp;
		int numberOfVertex;
		ifs >> numberOfVertex;
		//Point_2 firstOne_1;
		for (int j = 0; j < numberOfVertex; j++) {
			arc::point_2 p;
			double p_x, p_y;
			ifs >> p_x >> p_y;
			p_x = p_x / 850.0;
			p_y = p_y / 850.0;
			p_x = (p_x - 1.175) ;
			p_y = (p_y - 1.175) ;
			// p_x = p_x / 500.0;
			// p_y = p_y / 500.0;
			// p_x = (p_x - 2) ;
			// p_y = (p_y - 2) ;
			p.set<0>(p_x);
			p.set<1>(p_y);
			bg::append(tp.outer(), p);
		}

		// bg::append(tp.outer(), firstOne_1);
		bg::correct(tp);
		polygon_list.push_back(tp);


		// // Add raw obstacle to scene and set fill to be true with fill transparency
		// AdvancedGraphicsItem<Polygon_2>* pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envPolyList.back()));

		// // m_scene.addItem(pAGI);
		// pAGI->m_bShowEdge = true;
		// pAGI->m_bShowVertices = true;
		// pAGI->m_bFill = false;
		// pAGI->m_fillBrush = QColor(16, 16, 16, 192);
		// m_PolyAGIList.push_back(pAGI);

		// Computing the Minkowski sum
		// int split_num = 8;
		// Polygon_2 ep = growPolygonByRadius(tp, radius, split_num);
		// bg::correct(ep);
		// //split_num = 4;
		// //Polygon_2 vp = growPolygonByRadius(tp, radius, split_num);
		// //bg::correct(vp);

		// m_envPolyVoronoiList.push_back(ep);
		// m_envObsPolyList.push_back(ep);

		// // Add the configuration space obstacle to the scene
		// pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envObsPolyList.back()));

		// // m_scene.addItem(pAGI);
		// pAGI->m_bFill = true;
		// pAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		// pAGI->m_vertexPen = QPen(Qt::black, 0.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		// m_PolyAGIList.push_back(pAGI);
	}


	// Then read the bounding rectangle (configuration space)
	int numberOfBoundingVertex;
	arc::polygon_2 m_boundingPoly;
	ifs >> numberOfBoundingVertex;
	for (int j = 0; j < numberOfBoundingVertex; j++) {
		arc::point_2 p;
		double p_x, p_y;
		ifs >> p_x >> p_y;
		
		ROS_WARN_STREAM("point");
		p.set<0>(p_x);
		p.set<1>(p_y);
		bg::append(m_boundingPoly.outer(), p);
	}
	bg::correct(m_boundingPoly);

	// Add to scence

	// m_pBoundingPolyAGI = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly);
	// m_pBoundingPolyAGI->m_bFill = false;
	// m_pBoundingPolyAGI->m_bShowVertices = false;
	// m_pBoundingPolyAGI->m_bShowEdge = true;
	// m_pBoundingPolyAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	// m_PolyAGIList.push_back(m_pBoundingPolyAGI);
	// m_scene.addItem(m_pBoundingPolyAGI);

	// Then compute the outside bounding rectangle
	// double x1, y1, x2, y2;
	// std::vector<Point_2> const& points = m_boundingPoly.outer();

	// x1 = points[0].get<0>(); y1 = points[0].get<1>();
	// x2 = points[2].get<0>(); y2 = points[2].get<1>();
	// bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	// bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y2 + radius));
	// bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y2 + radius));
	// bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y1 - radius));
	// //	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	// bg::correct(m_boundingPoly2);
	// // Add to scene
	// m_pBoundingPolyAGI2 = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly2);
	// m_pBoundingPolyAGI2->m_bFill = false;
	// m_pBoundingPolyAGI2->m_bShowVertices = false;
	// m_pBoundingPolyAGI2->m_bShowEdge = true;;
	// m_pBoundingPolyAGI2->m_edgePen = QPen(Qt::black, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	// m_PolyAGIList.push_back(m_pBoundingPolyAGI2);
	// // m_scene.addItem(m_pBoundingPolyAGI2);

	// m_radius = radius;

	// drawBasicEnvironment();

	// // Do roadmap building setup
	// m_roadmap.buildRoadmap(&m_envObsPolyList, &m_envPolyList, &m_boundingPoly, &m_envPolyVoronoiList, radius, m_scene);
	// // m_roadmap.addToScene(m_scene);

	// m_boundingRect = QRectF(x1 - radius * 4, y1 - radius * 4, x2 + radius * 4, y2 + radius * 4);
	std::vector<std::pair<int, std::string>> picking_sequence;
	for (int i = 0; i < numberOfPolygons; i++) {
		std::pair<int, std::string> p;
		ifs >> p.first;
		picking_sequence.push_back(p);
	}


	std::vector<arc::polygon_2> generated_polys;
	// Spawn some cuboids and create the scence
	for(int i = 0; i < numberOfPolygons; i ++){
		std::stringstream ss; 
		ss << "cuboid_" << (i+1);
		std::string name = ss.str();
		arc::polygon_2 current_poly = polygon_list[i];
		std::string object_type;
			
		for(int j = 0; j < picking_sequence.size(); j++){
			if(picking_sequence[j].first == (i+1)){
				if(current_poly.outer().size() == 9){
					picking_sequence[j].second = "T_block";
				}else if(current_poly.outer().size() == 7){
					picking_sequence[j].second = "L_block";
				}else if(current_poly.outer().size() == 5){
					picking_sequence[j].second = "cube";
				}
			}
		}		
		geometry_msgs::Pose pose;
		if(current_poly.outer().size() == 9){
		// T shape
			geometry_msgs::Pose T_pose;
			T_pose.position.x = ((current_poly.outer()[0].get<0>() + current_poly.outer()[7].get<0>())/2.0 + (current_poly.outer()[5].get<0>() + current_poly.outer()[6].get<0>())/2.0) / 2.0;
			T_pose.position.y = ((current_poly.outer()[0].get<1>() + current_poly.outer()[7].get<1>())/2.0 + (current_poly.outer()[5].get<1>() + current_poly.outer()[6].get<1>())/2.0) / 2.0;
			T_pose.position.z = 0.0324/2.0;

    		tf::Quaternion q = arc::getPoseFromPolygon(current_poly);
	    	double q_x = q.x();
    		double q_y = q.y();
    		double q_z = q.z();
    		double q_w = q.getW();
			T_pose.orientation.x = q_x;
			T_pose.orientation.y = q_y;
			T_pose.orientation.z = q_z;
			T_pose.orientation.w = q_w;
			if(color_sequence[color_index] == 10)
				arc::gazeboUtility::spawnURDFModel(node_handle_, "/home/wei/catkin_gazebo_ws/src/luh_youbot_description/urdf/model_T_orange.urdf", T_pose, name);
			else if(color_sequence[color_index] == 2)
				arc::gazeboUtility::spawnURDFModel(node_handle_, "/home/wei/catkin_gazebo_ws/src/luh_youbot_description/urdf/model_T_red.urdf", T_pose, name);

			//arc::gazeboUtility::spawnSDFModel(node_handle_, "/home/wei/.gazebo/models/006_mustard_bottle/model_T.sdf", T_pose, name);
			object_type = "T_block";
			pose = T_pose;
			ROS_WARN_STREAM("object:"<<name<<", quatenion:"<<q_x<<","<<q_y<<","<<q_z<<","<<q_w);

		}else if(current_poly.outer().size() == 7){
		//L shape
			geometry_msgs::Pose L_pose;
			L_pose.position.x = (current_poly.outer()[1].get<0>() + current_poly.outer()[3].get<0>())/2.0;
			L_pose.position.y = (current_poly.outer()[1].get<1>() + current_poly.outer()[3].get<1>())/2.0;
			L_pose.position.z = 0.0324/2.0;


    		tf::Quaternion q = arc::getPoseFromPolygon(current_poly);
    		double q_x = q.x();
    		double q_y = q.y();
    		double q_z = q.z();
    		double q_w = q.getW();

			L_pose.orientation.x = q_x;
			L_pose.orientation.y = q_y;
			L_pose.orientation.z = q_z;
			L_pose.orientation.w = q_w;
			if(color_sequence[color_index] == 3)
				arc::gazeboUtility::spawnURDFModel(node_handle_, "/home/wei/catkin_gazebo_ws/src/luh_youbot_description/urdf/model_L_green.urdf", L_pose, name);
			else if(color_sequence[color_index] == 4)
				arc::gazeboUtility::spawnURDFModel(node_handle_, "/home/wei/catkin_gazebo_ws/src/luh_youbot_description/urdf/model_L_yellow.urdf", L_pose, name);

			object_type = "L_block";
			pose = L_pose;
			ROS_WARN_STREAM("object:"<<name<<", quatenion:"<<q_x<<","<<q_y<<","<<q_z<<","<<q_w);
		}else if(current_poly.outer().size() == 5){
		//rect
			bool higher = false;
			for(int k = 0; k < generated_polys.size(); k ++){
				if(bg::intersects(current_poly, generated_polys[k])){
					higher = true;
					break;
				}

			}
			double long_x, long_y, short_x, short_y;
	        double long_seg, short_seg;
	        double temp_seg = 0;
	        double temp_x, temp_y;
	        double yaw = 0;
	        long_x = current_poly.outer()[0].get<0>() - current_poly.outer()[3].get<0>();
	        long_y = current_poly.outer()[0].get<1>() - current_poly.outer()[3].get<1>();
	        long_seg = std::sqrt(long_x*long_x + long_y*long_y);

	        short_x = current_poly.outer()[0].get<0>() - current_poly.outer()[1].get<0>();
	        short_y = current_poly.outer()[0].get<1>() - current_poly.outer()[1].get<1>();
	        short_seg = std::sqrt(short_x*short_x + short_y*short_y);

	        if(short_seg > long_seg){
	            temp_seg = short_seg;
	            temp_x = short_x;
	            temp_y = short_y;
	            short_seg = long_seg;
	            short_x = long_x;
	            short_y = long_y;
	            long_seg = temp_seg;
	            long_x = temp_x;
	            long_y = temp_y;

	        }

			if(!higher){ 
				arc::gazeboUtility::spawnCuboid(node_handle_, 0.3, short_seg, short_seg, long_seg, 
	            current_poly,
	            arc::gazeboUtility::GAZEBO_COLORS[color_sequence[color_index]],     
	            name);	
	            ROS_WARN_STREAM("object:"<<name<<" is on the ground");
			}else{
				arc::gazeboUtility::spawnCuboid(node_handle_, 0.3, short_seg, short_seg, long_seg, 
	            current_poly,
	            arc::gazeboUtility::GAZEBO_COLORS[rand()%10],     
	            name, 0.05);
	            ROS_WARN_STREAM("object:"<<name<<" is higher");

			}
			generated_polys.push_back(current_poly);
			object_type = "cube";
			arc::gazeboUtility::getObjectPose(pose, name);

			dx = short_seg;
			dy = short_seg;
			dz = long_seg;
			ROS_WARN_STREAM("object:"<<name<<" size:"<<dx<<","<<dy<<","<<dz);
		}
		
		color_index ++;
		scene_object_list_.push_back(new PlanningSceneObject(name, object_type, dx, dy, dz, pose));
		scene_object_name_set_.insert(name);
	}

	

		


		std::string name = "wall_0";
		geometry_msgs::Pose pose;
		pose = arc::assignGeometryPose(-0.85, -1.1, 0.075, 0, 0, 0, 1);
		dx = 0.8;
		dy = 0.144018;
		dz = 0.3;
		PlanningSceneObject * wall_0 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
		scene_object_list_.push_back(wall_0);
		scene_object_name_set_.insert(name);
		name = "wall_1";
		pose = arc::assignGeometryPose(0.85, -1.1, 0.075, 0, 0, 0, 1);
		dx = 0.8;
		dy = 0.144018;
		dz = 0.3;
		PlanningSceneObject * wall_1 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
		scene_object_list_.push_back(wall_1);
		scene_object_name_set_.insert(name);
		name = "wall_2";     
		pose = arc::assignGeometryPose(1.3, 0, 0.1500001, 0, 0, 0.7071068, 0.7071068);
		dx = 2.2;
		dy = 0.144018;
		dz = 0.3;
		PlanningSceneObject * wall_2 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
		scene_object_list_.push_back(wall_2);
		scene_object_name_set_.insert(name);
		name = "wall_3";       
		pose = arc::assignGeometryPose(-1.3, 0, 0.15, 0, 0, 0.7071068, 0.7071068);
		dx = 2.2;
		dy = 0.144018;
		dz = 0.3;
		PlanningSceneObject * wall_3 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
		scene_object_list_.push_back(wall_3);
		scene_object_name_set_.insert(name);
		name = "wall_4";    
		pose = arc::assignGeometryPose(0, 1.15 ,0.075, 0,0,0,1);
		dx = 2.3;       
		dy = 0.14227;
		dz = 0.3;
		PlanningSceneObject * wall_4 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
		scene_object_list_.push_back(wall_4);
		scene_object_name_set_.insert(name);
		
		#ifdef WITH_OBS
		name = "pillar_0";
		pose = arc::assignGeometryPose(-0.7, -0.7, 0.2, 0, 0, 0, 1);
		dx = 0.1;
		dy = 0.1;
		dz = 0.4;
		PlanningSceneObject * pill_0 = new PlanningSceneObject(name, "pillar", dx, dy, dz, pose);
		scene_object_list_.push_back(pill_0);
		scene_object_name_set_.insert(name);
		name = "pillar_1";
		pose = arc::assignGeometryPose(-0.08, 0.2, 0.2, 0, 0, 0, 1);
		dx = 0.1;
		dy = 0.1;
		dz = 0.4;
		PlanningSceneObject * pill_1 = new PlanningSceneObject(name, "pillar", dx, dy, dz, pose);
		scene_object_list_.push_back(pill_1);
		scene_object_name_set_.insert(name);
		name = "pillar_2";
		pose = arc::assignGeometryPose(0.8, 0.5, 0.2, 0, 0, 0, 1);
		dx = 0.1;
		dy = 0.1;
		dz = 0.4;
		PlanningSceneObject * pill_2 = new PlanningSceneObject(name, "pillar", dx, dy, dz, pose);
		scene_object_list_.push_back(pill_2);
		scene_object_name_set_.insert(name);
		name = "pillar_3";
		pose = arc::assignGeometryPose(-0.5, 0.5, 0.2, 0, 0, 0, 1);
		dx = 0.1;
		dy = 0.1;
		dz = 0.4;
		PlanningSceneObject * pill_3 = new PlanningSceneObject(name, "pillar", dx, dy, dz, pose);
		scene_object_list_.push_back(pill_3);
		scene_object_name_set_.insert(name);
		#endif
	return picking_sequence;
}

std::vector<std::tuple<int, int, std::string>> PlanningScene::createGazeboYoubotTestSceneFromFileMulti
(int numberOfObjects, double length, double width, double dx, double dy, double dz, std::string filename)
{
	int m_numRobots = 0;
	// Clean up
	std::vector<arc::polygon_2> polygon_list;

	const char* c_filename = filename.c_str();
	// Reading in the environment, first the raidus of the (disc) robot
	std::ifstream ifs(c_filename);
	double radius;
	ifs >> radius;

	// Then the number of obstacle polygons
	int numberOfPolygons = 0;
	ifs >> numberOfPolygons;
	m_numRobots = numberOfPolygons;
	polygon_list.clear();
	// Then read in all obstacle polygons and compute the configuration space for a single disc
	for (int i = 0; i < numberOfPolygons; i++) {
		// Load polygon
		arc::polygon_2 tp;
		int numberOfVertex;
		ifs >> numberOfVertex;
		//Point_2 firstOne_1;
		for (int j = 0; j < numberOfVertex; j++) {
			arc::point_2 p;
			double p_x, p_y;
			ifs >> p_x >> p_y;
		// 	p_x = p_x / 1000.0;
		// p_y = p_y / 1000.0;
		// p_x = (p_x - 2.5)*2 ;
		// p_y = (p_y - 2.5)*2 ;
			p_x = p_x / 850.0;
			p_y = p_y / 850.0;
			p_x = (p_x - 1.175) ;
			p_y = (p_y - 1.175) ;
			// if(j == 0){
			//      		firstOne_1.set<0>(p_x);
			//      		firstOne_1.set<1>(p_y);
			//      	}
			p.set<0>(p_x);
			p.set<1>(p_y);
			bg::append(tp.outer(), p);
		}

		// bg::append(tp.outer(), firstOne_1);
		bg::correct(tp);
		polygon_list.push_back(tp);


		// // Add raw obstacle to scene and set fill to be true with fill transparency
		// AdvancedGraphicsItem<Polygon_2>* pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envPolyList.back()));

		// // m_scene.addItem(pAGI);
		// pAGI->m_bShowEdge = true;
		// pAGI->m_bShowVertices = true;
		// pAGI->m_bFill = false;
		// pAGI->m_fillBrush = QColor(16, 16, 16, 192);
		// m_PolyAGIList.push_back(pAGI);

		// Computing the Minkowski sum
		// int split_num = 8;
		// Polygon_2 ep = growPolygonByRadius(tp, radius, split_num);
		// bg::correct(ep);
		// //split_num = 4;
		// //Polygon_2 vp = growPolygonByRadius(tp, radius, split_num);
		// //bg::correct(vp);

		// m_envPolyVoronoiList.push_back(ep);
		// m_envObsPolyList.push_back(ep);

		// // Add the configuration space obstacle to the scene
		// pAGI = new AdvancedGraphicsItem<Polygon_2>(&(m_envObsPolyList.back()));

		// // m_scene.addItem(pAGI);
		// pAGI->m_bFill = true;
		// pAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		// pAGI->m_vertexPen = QPen(Qt::black, 0.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		// m_PolyAGIList.push_back(pAGI);
	}

	// Then read the bounding rectangle (configuration space)
	int numberOfBoundingVertex;
	arc::polygon_2 m_boundingPoly;
	ifs >> numberOfBoundingVertex;
	for (int j = 0; j < numberOfBoundingVertex; j++) {
		arc::point_2 p;
		double p_x, p_y;
		ifs >> p_x >> p_y;
		
		ROS_WARN_STREAM("point");
		p.set<0>(p_x);
		p.set<1>(p_y);
		bg::append(m_boundingPoly.outer(), p);
	}
	bg::correct(m_boundingPoly);

	// Add to scence

	// m_pBoundingPolyAGI = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly);
	// m_pBoundingPolyAGI->m_bFill = false;
	// m_pBoundingPolyAGI->m_bShowVertices = false;
	// m_pBoundingPolyAGI->m_bShowEdge = true;
	// m_pBoundingPolyAGI->m_edgePen = QPen(Qt::gray, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	// m_PolyAGIList.push_back(m_pBoundingPolyAGI);
	// m_scene.addItem(m_pBoundingPolyAGI);

	// Then compute the outside bounding rectangle
	// double x1, y1, x2, y2;
	// std::vector<Point_2> const& points = m_boundingPoly.outer();

	// x1 = points[0].get<0>(); y1 = points[0].get<1>();
	// x2 = points[2].get<0>(); y2 = points[2].get<1>();
	// bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	// bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y2 + radius));
	// bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y2 + radius));
	// bg::append(m_boundingPoly2.outer(), Point_2(x2 + radius, y1 - radius));
	// //	bg::append(m_boundingPoly2.outer(), Point_2(x1 - radius, y1 - radius));
	// bg::correct(m_boundingPoly2);
	// // Add to scene
	// m_pBoundingPolyAGI2 = new AdvancedGraphicsItem<Polygon_2>(&m_boundingPoly2);
	// m_pBoundingPolyAGI2->m_bFill = false;
	// m_pBoundingPolyAGI2->m_bShowVertices = false;
	// m_pBoundingPolyAGI2->m_bShowEdge = true;
	// m_pBoundingPolyAGI2->m_edgePen = QPen(Qt::black, 0.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	// m_PolyAGIList.push_back(m_pBoundingPolyAGI2);
	// // m_scene.addItem(m_pBoundingPolyAGI2);

	// m_radius = radius;

	// drawBasicEnvironment();

	// // Do roadmap building setup
	// m_roadmap.buildRoadmap(&m_envObsPolyList, &m_envPolyList, &m_boundingPoly, &m_envPolyVoronoiList, radius, m_scene);
	// // m_roadmap.addToScene(m_scene);

	// m_boundingRect = QRectF(x1 - radius * 4, y1 - radius * 4, x2 + radius * 4, y2 + radius * 4);
	std::vector<std::tuple<int, int, std::string>> picking_sequence;
	for (int i = 0; i < numberOfPolygons; i++) {
		int p = 0;
		int next_stop = 0;
		ifs >> p;
		ifs >> next_stop;
		picking_sequence.push_back(std::make_tuple(p, next_stop, " "));
	}



	// Spawn some cuboids and create the scence
	// for(int i = 0; i < numberOfPolygons; i ++){
	// 	std::stringstream ss; 
	// 	ss << "cuboid_" << (i+1);
	// 	std::string name = ss.str();
	// 	arc::polygon_2 current_poly = polygon_list[i];
		

	// 	arc::gazeboUtility::spawnCuboid(node_handle_, 0.1, dx, dy, dz, 
 //            current_poly,
 //            arc::gazeboUtility::GAZEBO_COLORS[i%10],
 //            name);		
	// 	geometry_msgs::Pose pose;
	// 	arc::gazeboUtility::getObjectPose(pose, name);
	// 	scene_object_list_.push_back(new PlanningSceneObject(name, "cube",dx, dy, dz, pose));
	// 	scene_object_name_set_.insert(name);
	// }

    ROS_WARN_STREAM("number of objects:"<<numberOfPolygons);
	std::vector<arc::polygon_2> generated_polys;
	// Spawn some cuboids and create the scence
	for(int i = 0; i < numberOfPolygons; i ++){
		std::stringstream ss; 
		ss << "cuboid_" << (i+1);
		std::string name = ss.str();
		arc::polygon_2 current_poly = polygon_list[i];
		std::string object_type;
			
		for(int j = 0; j < picking_sequence.size(); j++){
			if(std::get<0>(picking_sequence[j]) == (i+1)){
				if(current_poly.outer().size() == 9){
					std::get<2>(picking_sequence[j]) = "T_block";
				}else if(current_poly.outer().size() == 7){
					std::get<2>(picking_sequence[j]) = "L_block";
				}else if(current_poly.outer().size() == 5){
					std::get<2>(picking_sequence[j]) = "cube";
				}
			}
		}		
		geometry_msgs::Pose pose;
		if(current_poly.outer().size() == 9){
		// T shape
			geometry_msgs::Pose T_pose;
			T_pose.position.x = ((current_poly.outer()[0].get<0>() + current_poly.outer()[7].get<0>())/2.0 + (current_poly.outer()[5].get<0>() + current_poly.outer()[6].get<0>())/2.0) / 2.0;
			T_pose.position.y = ((current_poly.outer()[0].get<1>() + current_poly.outer()[7].get<1>())/2.0 + (current_poly.outer()[5].get<1>() + current_poly.outer()[6].get<1>())/2.0) / 2.0;
			T_pose.position.z = 0.0324/2.0;

    		tf::Quaternion q = arc::getPoseFromPolygon(current_poly);
	    	double q_x = q.x();
    		double q_y = q.y();
    		double q_z = q.z();
    		double q_w = q.getW();
			T_pose.orientation.x = q_x;
			T_pose.orientation.y = q_y;
			T_pose.orientation.z = q_z;
			T_pose.orientation.w = q_w;
			if(rand()%10 < 5)
			arc::gazeboUtility::spawnURDFModel(node_handle_, "/home/wei/catkin_gazebo_ws/src/luh_youbot_description/urdf/model_T_orange.urdf", T_pose, name);
			else
			arc::gazeboUtility::spawnURDFModel(node_handle_, "/home/wei/catkin_gazebo_ws/src/luh_youbot_description/urdf/model_T_red.urdf", T_pose, name);

			//arc::gazeboUtility::spawnSDFModel(node_handle_, "/home/wei/.gazebo/models/006_mustard_bottle/model_T.sdf", T_pose, name);
			object_type = "T_block";
			pose = T_pose;
			ROS_WARN_STREAM("object:"<<name<<", quatenion:"<<q_x<<","<<q_y<<","<<q_z<<","<<q_w);

		}else if(current_poly.outer().size() == 7){
		//L shape
			geometry_msgs::Pose L_pose;
			L_pose.position.x = (current_poly.outer()[1].get<0>() + current_poly.outer()[3].get<0>())/2.0;
			L_pose.position.y = (current_poly.outer()[1].get<1>() + current_poly.outer()[3].get<1>())/2.0;
			L_pose.position.z = 0.0324/2.0;


    		tf::Quaternion q = arc::getPoseFromPolygon(current_poly);
    		double q_x = q.x();
    		double q_y = q.y();
    		double q_z = q.z();
    		double q_w = q.getW();

			L_pose.orientation.x = q_x;
			L_pose.orientation.y = q_y;
			L_pose.orientation.z = q_z;
			L_pose.orientation.w = q_w;
			if(rand()%10 < 5)
			arc::gazeboUtility::spawnURDFModel(node_handle_, "/home/wei/catkin_gazebo_ws/src/luh_youbot_description/urdf/model_L_green.urdf", L_pose, name);
			else
			arc::gazeboUtility::spawnURDFModel(node_handle_, "/home/wei/catkin_gazebo_ws/src/luh_youbot_description/urdf/model_L_yellow.urdf", L_pose, name);
			object_type = "L_block";
			pose = L_pose;
			ROS_WARN_STREAM("object:"<<name<<", quatenion:"<<q_x<<","<<q_y<<","<<q_z<<","<<q_w);
		}else if(current_poly.outer().size() == 5){
		//rect
			bool higher = false;
			for(int k = 0; k < generated_polys.size(); k ++){
				if(bg::intersects(current_poly, generated_polys[k])){
					higher = true;
					break;
				}

			}
			if(!higher){ 
				arc::gazeboUtility::spawnCuboid(node_handle_, 0.1, dx, dy, dz, 
	            current_poly,
	            arc::gazeboUtility::GAZEBO_COLORS[rand()%10],     
	            name);	
	            ROS_WARN_STREAM("object:"<<name<<" is on the ground");
			}else{
				arc::gazeboUtility::spawnCuboid(node_handle_, 0.1, dx, dy, dz, 
	            current_poly,
	            arc::gazeboUtility::GAZEBO_COLORS[rand()%10],     
	            name, 0.05);
	            ROS_WARN_STREAM("object:"<<name<<" is higher");

			}
			generated_polys.push_back(current_poly);
			object_type = "cube";
			arc::gazeboUtility::getObjectPose(pose, name);
		}
		
		
		scene_object_list_.push_back(new PlanningSceneObject(name, object_type, dx, dy, dz, pose));
		scene_object_name_set_.insert(name);
	}

	
	std::string name = "wall_0";
	geometry_msgs::Pose pose;
	pose = arc::assignGeometryPose(1.2, 0.5, 0.15, 0, 0, 0.7071068, 0.7071068);
	dx = 0.7;
	dy = 0.144018;
	dz = 0.3;
	PlanningSceneObject * wall_0 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
	scene_object_list_.push_back(wall_0);
	scene_object_name_set_.insert(name);
	name = "wall_1";
	pose = arc::assignGeometryPose(-1.2, 0.5, 0.15, 0, 0, 0.7071068, 0.7071068);
	dx = 0.7;
	dy = 0.144018;
	dz = 0.3;
	PlanningSceneObject * wall_1 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
	scene_object_list_.push_back(wall_1);
	scene_object_name_set_.insert(name);
	name = "wall_2";     
	pose = arc::assignGeometryPose(1.2, -0.85, 0.15, 0, 0, 0.7071068, 0.7071068);
	dx = 0.7;
	dy = 0.144018;
	dz = 0.3;
	PlanningSceneObject * wall_2 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
	scene_object_list_.push_back(wall_2);
	scene_object_name_set_.insert(name);
	name = "wall_3";       
	pose = arc::assignGeometryPose(-1.2, -0.85, 0.15, 0, 0, 0.7071068, 0.7071068);
	dx = 0.7;
	dy = 0.144018;
	dz = 0.3;
	PlanningSceneObject * wall_3 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
	scene_object_list_.push_back(wall_3);
	scene_object_name_set_.insert(name);
	name = "wall_4";     
	pose = arc::assignGeometryPose(-0.825, 0.9, 0.15, 0,0,0,1);
	dx = 0.75;       
	dy = 0.14227;
	dz = 0.3;
	PlanningSceneObject * wall_4 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
	scene_object_list_.push_back(wall_4);
	scene_object_name_set_.insert(name);
	name = "wall_5";     
	pose = arc::assignGeometryPose(0.825, 0.9, 0.15, 0,0,0,1);
	dx = 0.75;       
	dy = 0.14227;
	dz = 0.3;
	PlanningSceneObject * wall_5 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
	scene_object_list_.push_back(wall_5);
	scene_object_name_set_.insert(name);
	name = "wall_6";     
	pose = arc::assignGeometryPose(-0.8, -1.3, 0.15, 0,0,0,1);
	dx = 0.8;       
	dy = 0.14227;
	dz = 0.3;
	PlanningSceneObject * wall_6 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
	scene_object_list_.push_back(wall_6);
	scene_object_name_set_.insert(name);
	name = "wall_7";     
	pose = arc::assignGeometryPose(0.8, -1.3, 0.15, 0,0,0,1);
	dx = 0.8;       
	dy = 0.14227;
	dz = 0.3;
	PlanningSceneObject * wall_7 = new PlanningSceneObject(name, "wall", dx, dy, dz, pose);
	scene_object_list_.push_back(wall_7);
	scene_object_name_set_.insert(name);


	return picking_sequence;
}


void PlanningScene::cleanGazeboYoubotTestScene()
{
	std::map< std::string, geometry_msgs::Pose > objectNamePoseMap;
	arc::gazeboUtility::getObjectPoses(NULL, objectNamePoseMap);
	for(std::map< std::string, geometry_msgs::Pose >::iterator it = objectNamePoseMap.begin();
		it != objectNamePoseMap.end(); it ++ )
	{
		std::string objectname = (*it).first;
		std::string substr = objectname.substr(0, 7);
		if(substr == "cuboid_")
		{
			arc::gazeboUtility::deleteModel(node_handle_,objectname);
		}
	}
}

/**
 * retrieve and update scene objects and robot’s newest pose
 */
void PlanningScene::updateScene()
{
	// Retrieve poses for all affected objects
	std::map<std::string, geometry_msgs::Pose> namePoseMap;
	arc::gazeboUtility::getObjectPoses(&scene_object_name_set_, namePoseMap);

	// Update the poses 
	geometry_msgs::Pose pose;


	for(std::vector<PlanningSceneObject*>::iterator it = scene_object_list_.begin();
		it != scene_object_list_.end(); it++)
	{	
		if((*it)->object_name_ != "floor"){ 
		if(namePoseMap.find((*it)->object_name_) != namePoseMap.end()){
			auto mit = namePoseMap.find((*it)->object_name_);
			(*it)->updateObjectPose((*mit).second);
		}else{
			delete *it;
			scene_object_list_.erase(it);
		}
		}
		// std::cout << (*it)->object_name_ << boost::geometry::dsv((*it)->object_footprint_) << std::endl;
	}

}

void PlanningScene::getCollisionArrayIndices(int& minXI, int& maxXI, int& minYI, int& maxYI,
	double minX, double maxX, double minY, double maxY)
{
	// Filter out the trivial cases
	if(maxX < x_min_ || minX > x_max_ || maxY < y_min_ || minY > y_max_)
	{
		minXI = maxXI = minYI = maxYI = -1;
		return;
	}

	// There are COLLISION_CELLS_X cells in the X direction that going from x_min_
	// to x_max_. Firt figure out minXI
	minXI = (int)((minX < x_min_ ? 0 : minX - x_min_)/((x_max_-x_min_)/COLLISION_CELLS_X));
	maxXI = (int)(ceil((maxX > x_max_ ? x_max_ - x_min_ : maxX - x_min_)/((x_max_-x_min_)/COLLISION_CELLS_X)));

	minYI = (int)((minY < y_min_ ? 0 : minY - y_min_)/((y_max_-y_min_)/COLLISION_CELLS_Y));
	maxYI = (int)(ceil((maxY > y_max_ ? y_max_ - y_min_ : maxY - y_min_)/((y_max_-y_min_)/COLLISION_CELLS_Y)));
}

/**
 * Update the partitionined collision checking structure 
 */
void PlanningScene::updateCollisionHash()
{
	// Clean up
	for(int ix = 0; ix < COLLISION_CELLS_X; ix ++)
	{
		for(int iy = 0; iy < COLLISION_CELLS_Y; iy ++)
		{
			scene_object_pointers_[ix][iy].clear();
		}
	}
	

	// Check each obstacle individually 
	int minXI, maxXI, minYI, maxYI;
	for(std::vector<PlanningSceneObject*>::iterator it = scene_object_list_.begin();
		it != scene_object_list_.end(); it++){

		// Retrieve the bounding box
		box_2& obox = (*it)->object_footprint_bounding_box_;
		point_2 min_corner = obox.min_corner();
		point_2 max_corner = obox.max_corner();

		// Figure out the indices
		getCollisionArrayIndices(minXI, maxXI, minYI, maxYI, min_corner.get<0>(), 
			max_corner.get<0>(), min_corner.get<1>(), max_corner.get<1>());

		// std::cout << minXI << " " << maxXI << " " << minYI << " " << maxYI << std::endl;

		// For each affected cell, check whether collsion actually happens
		for(int ix = minXI; ix < maxXI; ix ++)
		{
			for(int iy = minYI; iy < maxYI; iy ++)
			{
				if(bg::intersects(cell_bounding_polygon_[ix][iy], (*it)->object_footprint_))
				{
					scene_object_pointers_[ix][iy].push_back(*it);
					//std::cout << bg::dsv(cell_bounding_polygon_[ix][iy]) << std::endl;
					//std::cout << ix << " " << iy << " " << (*it)->object_name_ << std::endl;
				}
			}
		}
	}

	// Plot the footprint for debugging 
	
	// for(auto it = scene_object_list_.begin(); it != scene_object_list_.end(); it++)
	// {
		
	// 	PlanningSceneObject& obj = *(*it);
	// 	point_2 min_corner = obj.object_footprint_bounding_box_.min_corner();
	// 	point_2 max_corner = obj.object_footprint_bounding_box_.max_corner();
	
	// 	std::stringstream ss; 
	// 	ss << obj.object_name_ << "_shadow";
	// 	std::string name = ss.str();
	// 	gazeboUtility::deleteModel(node_handle_,name);
	// 	gazeboUtility::spawnPassThroughCuboid(node_handle_, 
	// 		max_corner.get<0>() - min_corner.get<0>(), 
	// 		max_corner.get<1>() - min_corner.get<1>(), 
	// 		0.02, 
	// 		(max_corner.get<0>() + min_corner.get<0>())/2,
	// 		(max_corner.get<1>() + min_corner.get<1>())/2,
	// 		0, 
	// 		0, 0, 0, 1,
	// 		name,
	// 		true);

	// }
}

/**
 * whether there is collision between objects and robots given current poses
 */
bool PlanningScene::isCollisionFreeWithExceptions(const polygon_2& robot, std::vector<std::string> exception_names, bool debugInfo)
{
	// Get bounding box for the robot and compute the affected indices 
	box_2 robotBoundingBox = 
		bg::return_envelope<box_2, polygon_2>(robot);

	// std::cout << "Polygon to be checked " << bg::dsv(robot) << std::endl;
	// std::cout << "Its bounding box " << bg::dsv(robotBoundingBox) << std::endl;

	int minXI, maxXI, minYI, maxYI;
	point_2 min_corner = robotBoundingBox.min_corner();
	point_2 max_corner = robotBoundingBox.max_corner();
	getCollisionArrayIndices(minXI, maxXI, minYI, maxYI, min_corner.get<0>(), 
		max_corner.get<0>(), min_corner.get<1>(), max_corner.get<1>());

	// std::cout << minXI << " " << maxXI << " " << minYI << " " << maxYI << std::endl;		

	// Compute possible collisions. Logic: 
	// 1. for each possibly affected cell, check whether the robot is actually in the cell 
	// 2. If the above is true, check for possible collision with obstacles in the cell,
	//    remembering the obstacles that we have already checked. 
	std::set<int> checkedObstacleSet;
	for(int ix = minXI; ix < maxXI; ix ++)
	{
		for(int iy = minYI; iy < maxYI; iy ++)
		{
			// Check whether the robot is in the cell 
			//if(bg::intersects(cell_bounding_polygon_[ix][iy], robot))
			//{
				// Check collision with objects
				for(std::vector<PlanningSceneObject*>::iterator it = scene_object_pointers_[ix][iy].begin();
					it != scene_object_pointers_[ix][iy].end(); it++)
				{	
					bool is_exception = false;
					for(int k = 0; k < exception_names.size();k++){
						if((*it)->object_name_ == exception_names[k]){
							is_exception = true;
							break;
						}
					}
					if(is_exception){
						continue;
					}
					//fran fran_;
					// Some logic to avoid checking the same obstacle mutliple times
					if(checkedObstacleSet.find((*it)->object_id_) == checkedObstacleSet.end())
					{
						point_2 object_center, robot_center;
						bg::centroid((*it)->object_footprint_, object_center);
						if(bg::within(object_center, robot)){
							return false;
						}
						 
						// Check collision of the bounding boxes first 
						if(bg::intersects((*it)->object_footprint_bounding_box_, robotBoundingBox) /*|| bg::within((*it)->object_footprint_bounding_box_, robotBoundingBox,  fran_)*/)
						{
							if(bg::intersects((*it)->object_footprint_, robot))
							{
								//std::cout << "Collsion detected! with " <<(*it)->object_name_  <<std::endl;
								return false;
							}
						}
						checkedObstacleSet.insert((*it)->object_id_);
					}
				}
			//}
		}
	}

	return true;
}



/**
 * whether there is collision between objects and robots given current poses
 */
bool PlanningScene::isCollisionFree(const polygon_2& robot, bool debugInfo)
{
	// Get bounding box for the robot and compute the affected indices 
	box_2 robotBoundingBox = 
		bg::return_envelope<box_2, polygon_2>(robot);

	//std::cout << "Polygon to be checked " << bg::dsv(robot) << std::endl;
	//std::cout << "Its bounding box " << bg::dsv(robotBoundingBox) << std::endl;

	int minXI, maxXI, minYI, maxYI;
	point_2 min_corner = robotBoundingBox.min_corner();
	point_2 max_corner = robotBoundingBox.max_corner();
	getCollisionArrayIndices(minXI, maxXI, minYI, maxYI, min_corner.get<0>(), 
		max_corner.get<0>(), min_corner.get<1>(), max_corner.get<1>());

	// std::cout << minXI << " " << maxXI << " " << minYI << " " << maxYI << std::endl;		

	// Compute possible collisions. Logic: 
	// 1. for each possibly affected cell, check whether the robot is actually in the cell 
	// 2. If the above is true, check for possible collision with obstacles in the cell,
	//    remembering the obstacles that we have already checked. 
	std::set<int> checkedObstacleSet;
	for(int ix = minXI; ix < maxXI; ix ++)
	{
		for(int iy = minYI; iy < maxYI; iy ++)
		{
			// Check whether the robot is in the cell 
			//if(bg::intersects(cell_bounding_polygon_[ix][iy], robot))
			//{
				// Check collision with objects
				for(std::vector<PlanningSceneObject*>::iterator it = scene_object_pointers_[ix][iy].begin();
					it != scene_object_pointers_[ix][iy].end(); it++)
				{
					//ROS_WARN_STREAM("checking object:"<<(*it)->object_name_);
				//	fran fran_;
					// Some logic to avoid checking the same obstacle mutliple times
					if(checkedObstacleSet.find((*it)->object_id_) == checkedObstacleSet.end())
					{
						point_2 object_center, robot_center;
						bg::centroid((*it)->object_footprint_, object_center);
						if(bg::within(object_center, robot)){
							//ROS_WARN_STREAM((*it)->object_name_ <<"object center is within robot ");
							return false;
						}
						// Check collision of the bounding boxes first 
						if(bg::intersects((*it)->object_footprint_bounding_box_, robotBoundingBox) /*|| bg::within((*it)->object_footprint_bounding_box_, robotBoundingBox,fran_)*/)
						{
							if(bg::intersects((*it)->object_footprint_, robot) /*|| bg::within((*it)->object_footprint_, robot, fran_)*/)
							{
								// std::cout << "Collsion detected! " << std::endl;
								return false;
							}
						}
						checkedObstacleSet.insert((*it)->object_id_);
					}
				}
			//}
		}
	}

	return true;
}

/**
 * whether there is collision between objects and robots given current poses
 */
bool PlanningScene::isCollisionFreeSlow(const polygon_2& robot)
{
	for(auto it = scene_object_list_.begin(); it != scene_object_list_.end(); it++)
	{
		if(bg::intersects((*it)->object_footprint_, robot))
			return false;
	}
	return true;
}

/**
 * Locate a collision free pose within dx x dy region around the origin
 */
void PlanningScene::findCollsionFreePose(const polygon_2& robot, 
	double &x, double &y, double &t, double dx, double dy)
{
	while(1)
	{
		x = (rand() - RAND_MAX/2.)*dx/RAND_MAX;
		y = (rand() - RAND_MAX/2.)*dy/RAND_MAX;
		t = (rand() - RAND_MAX/2.)*3.14/RAND_MAX;
		arc::polygon_2 newBasePoly;
		arc::movePolygon(robot, newBasePoly, x, y, t);
		if(isCollisionFree(newBasePoly)) return;
	}
}


/**
 * Test of youbot base collision checking capability 
 */
void PlanningScene::collisionCheckTest(const polygon_2& robot, int numberOfChecks)
{
	// Spawn some cuboids
	ROS_INFO("Spawning 50 cuboids in an 8 x 8 region...");
	arc::PlanningScene planningScene(node_handle_,-10, 10, -10, 10);
	planningScene.createGazeboYoubotTestScene(50, 8, 8);

	// Sleep a bit so that the spawned cuboids will settle
	ROS_INFO("Wait 5 seconds for spawn to complete...");
	ros::Duration(5).sleep();

	// Do some collision checking test
	planningScene.updateScene();
	planningScene.updateCollisionHash();


	arc::polygon_2 newBasePoly;
	double x, y, theta;

	ROS_INFO("Performing %d collision checks within the 8 x 8 region.", numberOfChecks);
	ros::Time tb = ros::Time::now();
	for(int i = 0; i < numberOfChecks; i ++)
	{
		// Generate some random poses
		x = (rand() - RAND_MAX/2.)*6/RAND_MAX;
		y = (rand() - RAND_MAX/2.)*6/RAND_MAX;
		theta = (rand() - RAND_MAX/2.)*3.14/RAND_MAX;

		// Display the random youbot pose
		// youbot.addBaseWayPoint(x, y, theta);
		// ros::Duration(0.5).sleep();

		// Transform the youbot footprint 
		arc::movePolygon(robot, newBasePoly, x, y, theta);
		// std::cout << "Youbot base: " << bg::dsv(newBasePoly) << std::endl;

		// Check collision
		bool collsion = planningScene.isCollisionFree(newBasePoly);
		
		// std::cout << collsion << std::endl;
		newBasePoly.clear();
	}
	ROS_INFO("Used time: %f seconds.", (ros::Time::now() - tb).toSec());
	planningScene.cleanGazeboYoubotTestScene();	
}

void PlanningScene::getSceneObjectList(std::vector<PlanningSceneObject*> &scene_object_list)
{
	for(int i = 0;i < scene_object_list_.size();i ++){
		scene_object_list.push_back(scene_object_list_[i]);

	}
	
}


}

