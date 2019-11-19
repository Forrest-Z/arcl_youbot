#include "arcl_youbot_application/ManipulationServer.hpp"

ManipulationServer::ManipulationServer(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&ManipulationServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ManipulationServer::preemptCB, this));


    as_.start();
    ROS_INFO_STREAM("start manipulation planning server!");
    grasp_client_ = nh_.serviceClient<youbot_grasp::GraspPlanning>("GraspPlanning");
    GraspGenerator::loadFile(); 

    foot_print_.outer().push_back(arc::point_2(0.5*BASE_LENGTH,0.5*BASE_WIDTH));
	foot_print_.outer().push_back(arc::point_2(0.5*BASE_LENGTH,-0.5*BASE_WIDTH));
	foot_print_.outer().push_back(arc::point_2(-0.5*BASE_LENGTH,-0.5*BASE_WIDTH));
	foot_print_.outer().push_back(arc::point_2(-0.5*BASE_LENGTH,0.5*BASE_WIDTH));
	bg::correct(foot_print_);
  }

ManipulationServer::~ManipulationServer(void)
{
}

void ManipulationServer::ManipulationSceneToGraspScene(arcl_youbot_application::PlanningSceneMsg m_scene, youbot_grasp::PlanningScene& g_scene)
{
    ROS_WARN("Init grasp scene from manipulation scene");
    g_scene.header = m_scene.header;
    for(auto scene_object: m_scene.scene_object_list)
    {   ROS_WARN_STREAM(scene_object.object_pose.position.x<<","<<scene_object.object_pose.position.y<<","<<scene_object.object_pose.position.z);
        g_scene.scene_object_name.push_back(scene_object.object_name);
        g_scene.scene_object_pose.push_back(scene_object.object_pose);
        g_scene.dx.push_back(scene_object.dx);
        g_scene.dy.push_back(scene_object.dy);
        g_scene.dz.push_back(scene_object.dz);
    }
}

void ManipulationServer::FromGraspPoseToBasePose(geometry_msgs::Pose gripper_pose, std::vector<double> gripper_dir, double cylin_r, double q1, double q5, double theta, geometry_msgs::Pose &base_pose)
{
    tf::Quaternion gripper_q(gripper_pose.orientation.x, gripper_pose.orientation.y, gripper_pose.orientation.z, gripper_pose.orientation.w); 
    tf::Matrix3x3 gripper_m(gripper_q);
    tf::Vector3 y_axis = gripper_m.getColumn(1);
    tf::Vector3 x_axis = gripper_m.getColumn(0);
    tf::Matrix3x3 base_m;
    tf::Quaternion base_q;
    double joint_1_x, joint_1_y;
    double base_x, base_y, base_z;
    double normal_x = gripper_dir[0];
    double normal_y = gripper_dir[1];
    double normal_z = gripper_dir[2];    
    double normal_x_prime = normal_x / sqrt(pow(normal_x,2) + pow(normal_y,2));
    double normal_y_prime = normal_y / sqrt(pow(normal_x,2) + pow(normal_y,2));
    if(y_axis.z() - 0.0 > 0.2 || y_axis.z() - 0.0 < -0.2){
        #ifdef DEBUG_
        ROS_WARN("the gripper is not upright");
        #endif
        joint_1_x = gripper_pose.position.x + (cylin_r-sin(theta)* 0.035) *normal_x_prime;
        joint_1_y = gripper_pose.position.y + (cylin_r-sin(theta)* 0.035)*normal_y_prime;        
        double base_joint_1_x = cos(q1)*normal_x_prime - sin(q1)*normal_y_prime;
        double base_joint_1_y = sin(q1)*normal_x_prime + cos(q1)*normal_y_prime;
        base_x = joint_1_x + base_joint_1_x * ykin::JOINT_1_TO_BASE;
        base_y = joint_1_y + base_joint_1_y * ykin::JOINT_1_TO_BASE;
        base_z = 0;
        tf::Matrix3x3 base_m;
        double a = -base_joint_1_x;
        double b = -base_joint_1_y;
        base_m.setValue(a, -b, 0, b, a, 0, 0, 0, 1);
        tf::Quaternion base_q;
        base_m.getRotation(base_q);
        
        base_pose.orientation.x = base_q.x();
        ROS_WARN("the gripper is not upright");
        base_pose.orientation.y = base_q.y();
        base_pose.orientation.z = base_q.z();
        base_pose.orientation.w = base_q.w();
        base_pose.position.x = base_x;
        base_pose.position.y = base_y;
        base_pose.position.z = base_z;

    }else{ 
        double y_axis_x = y_axis.x();
        double y_axis_y = y_axis.y();
        double y_axis_x_prime = y_axis_x*cos(q5) - y_axis_y*sin(q5);
        double y_axis_y_prime = y_axis_x*sin(q5) + y_axis_y*cos(q5);
        y_axis_y_prime = -y_axis_y_prime;
        y_axis_x_prime = -y_axis_x_prime;

        joint_1_x = gripper_pose.position.x +/* x_axis.x()*0.005*/ + cylin_r*y_axis_x_prime;
        joint_1_y = gripper_pose.position.y +/* x_axis.y()*0.005*/ + cylin_r*y_axis_y_prime;        
        double base_joint_1_x = cos(q1)*y_axis_x_prime - sin(q1)*y_axis_y_prime;
        double base_joint_1_y = sin(q1)*y_axis_x_prime + cos(q1)*y_axis_y_prime;
        base_x = joint_1_x + base_joint_1_x * ykin::JOINT_1_TO_BASE;
        base_y = joint_1_y + base_joint_1_y * ykin::JOINT_1_TO_BASE;
        base_z = 0;
        double a = -base_joint_1_x;
        double b = -base_joint_1_y;
        base_m.setValue(a, -b, 0, b, a, 0, 0, 0, 1);
        base_m.getRotation(base_q);
        
        base_pose.orientation.x = base_q.x();
        base_pose.orientation.y = base_q.y();
        base_pose.orientation.z = base_q.z();
        base_pose.orientation.w = base_q.w();
        base_pose.position.x = base_x;
        base_pose.position.y = base_y;
        base_pose.position.z = base_z;
    }
}

void ManipulationServer::GenerateSamplesForArmConf(geometry_msgs::Pose gripper_pose, std::vector<double> gripper_dir,
    double &min_cylin_r, double &max_cylin_r, double &cylin_z, std::vector<double>& q1_list, std::vector<double>& q5_list, double& theta)
{
    double normal_x = gripper_dir[0];
    double normal_y = gripper_dir[1];
    double normal_z = gripper_dir[2];
    double normal_x_prime = normal_x / sqrt(pow(normal_x,2) + pow(normal_y,2));
    double normal_y_prime = normal_y / sqrt(pow(normal_x,2) + pow(normal_y,2));
    ROS_WARN_STREAM("normal_x:"<<normal_x<<", normal_y:"<<normal_y<<", normal_z:"<<normal_z);
    // get the theta
    theta = M_PI - atan2(sqrt(pow(normal_x,2)+pow(normal_y,2)), normal_z);

    //get cylin_z
    double tip_z;
   // 0.067 is the length of the gripper finger
    
//    tip_z = gripper_pose.position.z - normal_z*0.02;
    tip_z = gripper_pose.position.z;

    cylin_z = tip_z - ykin::H2 + ykin::L0; // relative to arm_link_0
    
    std::cout<<"cylin_z:"<<cylin_z<<std::endl;
    int space;
    //generate the range for cylin_r(min_cylin_r and max_cylin_r)  
    double axis4_to_axis2_z = tip_z - ykin::H2 - ykin::L4 * cos(theta);
    double axis4_axis2_max_dist = ykin::L2 + ykin::L3;
    max_cylin_r = sqrt(axis4_axis2_max_dist*axis4_axis2_max_dist - axis4_to_axis2_z*axis4_to_axis2_z) + ykin::L4*sin(theta) + ykin::L1 + 0.02;
    
    
    if(theta - M_PI < 0.1 && theta - M_PI > -0.1){
        min_cylin_r = 0.24*sin(theta) + 0.033 + 0.2;
    }else{
        min_cylin_r = 0.24*sin(theta) + 0.033 + 0.14; //+ sqrt(pow(normal_x,2)+pow(normal_y,2));
    }
    ROS_WARN_STREAM("theta:"<<theta);
    ROS_WARN_STREAM("min_cylin_r"<<min_cylin_r<<", max_cylin_r:"<<max_cylin_r<<"cylin_z:"<<cylin_z);
    //generate sample sets for q1, q5
    double q1;
    double q5;
    q1_list.clear();
    q5_list.clear();
    for(q1 = 0.0; q1 < ykin::MAX_JNT_POSITIONS[0];q1 += 0.05){
        q1_list.push_back(q1 - ykin::JOINT_OFFSETS[0]);
        q1_list.push_back(-q1 - ykin::JOINT_OFFSETS[0]);
    }
    q5_list.push_back(4.57194);
    q5_list.push_back(1.43194);
    for(q5 = 3.00194; q5 <= 5.641592441; q5 += 0.15){
        q5_list.push_back(q5);
        q5_list.push_back(3.00194 - q5 + 3.00194);
    }
}

void ManipulationServer::goalCB()
{
    ykin::CylindricPosition current_cylin_pose;
    ykin::CylindricPosition cylin_test;
    ykin::CylindricPosition pre_cylin_test;
    ykin::CylindricPosition final_cylin_test;
    ykin::CylindricPosition rest_cylin_test;
    ykin::CylindricPosition drop_cylin_pose;
    ykin::JointPosition final_joint_test;
    ykin::JointPosition joint_test;
    ykin::JointPosition pre_joint_test;
    
    std::vector<geometry_msgs::Pose> final_base_pose_list;
    std::vector<ykin::CylindricPosition>  current_cylin_pose_list;
    bool is_grasp_pose_ok = false;
    bool is_base_r_ok = true;
    bool is_base_r_no_ok = true;
    bool is_base_q1_ok = true;
    bool is_base_q1_no_ok = true;
    bool is_base_q5_ok = false;
    bool is_base_q5_no_ok = true;
    bool found_ok_config = false;
    bool arm_upright = false;  

    std::vector<double> joint_pre_value{169.0/180.0*M_PI, 50.0/180.0*M_PI, -130.0/180.0*M_PI, 165.0/180.0*M_PI, 176.0/180.0*M_PI};
    std::vector<double> joint_post_value{1.0/180.0*M_PI, 70.0/180.0*M_PI, -110.0/180.0*M_PI, 180.0/180.0*M_PI, 90.0/180.0*M_PI};
    std::vector<double> joint_drop_value{169.0/180.0*M_PI, 60.0/180.0*M_PI, -60.0/180.0*M_PI, 138.0/180.0*M_PI, 170.0/180.0*M_PI};
    #ifdef DEBUG_
    ROS_WARN("before move_group being called");
    #endif
    
    drop_cylin_pose.setQ1(0);
    drop_cylin_pose.setR(0.263);
    drop_cylin_pose.setZ(-0.13195);
    drop_cylin_pose.setTheta(3.14159);
    drop_cylin_pose.setQ5(1.466092);
    #ifdef DEBUG_
    ROS_WARN("before action being called");
    #endif
    // accept the new goal
    auto new_goal_ = as_.acceptNewGoal();
    manipulation_scene_ = new_goal_->planning_scene;
    target_object_name_ = new_goal_->target_object_name;
    target_object_pose_ = new_goal_->target_object_pose;
    target_object_type_ = new_goal_->target_object_type;
    last_target_object_name_ = new_goal_->last_target_object_name;
    is_synchronize_ = new_goal_->is_synchronize.data;
    rest_base_pose_ = new_goal_->rest_base_pose;
    youbot_grasp::PlanningScene grasp_scene;

    #ifdef DEBUG_
    ROS_WARN("action being called");
    #endif

    ManipulationSceneToGraspScene(manipulation_scene_, grasp_scene);

    double target_dx, target_dy, target_dz;
    for(int i = 0; i < manipulation_scene_.scene_object_list.size(); i++){
        if(target_object_name_.data == manipulation_scene_.scene_object_list[i].object_name.data){
            target_dx = manipulation_scene_.scene_object_list[i].dx;
            target_dy = manipulation_scene_.scene_object_list[i].dy;
            target_dz = manipulation_scene_.scene_object_list[i].dz;
        }
    }


    arc::PlanningScene base_planning_scene(nh_, -5, 5, -5, 5);
    base_planning_scene.InitFromMsg(manipulation_scene_);
    base_planning_scene.updateCollisionHash();
    ROS_WARN_STREAM("CURRENT DEBUG PRINT_____________________________");
    base_planning_scene.printDebugInfo();
    ROS_WARN_STREAM("END DEBUG PRINT_______________________");

    if(is_synchronize_)
    {
        //if is_synchronize is set to true, the grasp_scene and manipulation_scene_ are both update to date 
    }
    else
    {
        //if is_synchronize is set to false, it is planning ahead, get rid of last target object from the grasp_scene and manipulation_scene_, which may save room for new plan
        
        for(int i = 0;i < grasp_scene.scene_object_name.size(); i++){
            if(last_target_object_name_.data == grasp_scene.scene_object_name[i].data){
                last_target_object_index_ = i;
                grasp_scene.scene_object_pose.erase(grasp_scene.scene_object_pose.begin()+i);
                grasp_scene.scene_object_name.erase(grasp_scene.scene_object_name.begin()+i);
                break;
            }
        }
        // process the manipulation_scene_
    }

    youbot_grasp::GraspPlanning srv;
    std_msgs::String gripper_type;
    gripper_type.data = "luh_standard";

    srv.request.gripper_type = gripper_type;
    srv.request.object_file_name = target_object_type_;  // this should be the real mesh filename
    srv.request.object_pose = target_object_pose_;
    srv.request.planning_scene = grasp_scene;
    srv.request.dx = target_dx;
    srv.request.dy = target_dy;
    srv.request.dz  =target_dz;
    ROS_WARN_STREAM("target_object_pose:"<<target_object_pose_.position.x<<","<<target_object_pose_.position.y);
    //ros::Publisher fly_gripper_pub = nh_.advertise<geometry_msgs::Pose>("/vrep/fly_gripper", 1000);
    youbot_grasp::PlannedGrasp_vector planned_grasp_vector;
    double normal_x, normal_y, normal_z;
    if (grasp_client_.call(srv))
    {     
        geometry_msgs::Pose base_pose;
        planned_grasp_vector = srv.response.planned_grasp_vector;        
        #ifdef DEBUG_
            ROS_INFO_STREAM("grasp planning service returned grasp num:"<<planned_grasp_vector.vector.size());
        #endif
        std::string last_name;
        for(int i = 0; i < planned_grasp_vector.vector.size();i++){
            geometry_msgs::Pose gripper_pose = planned_grasp_vector.vector[i].gripper_pose;
            std::stringstream ss; 
		    ss << "fly_gripper_" << (i);
		    std::string name = ss.str();
            last_name = name;
            
            std::vector<double> gripper_dir{planned_grasp_vector.vector[i].grasp_dir.x,planned_grasp_vector.vector[i].grasp_dir.y,planned_grasp_vector.vector[i].grasp_dir.z};
            double min_cylin_r, max_cylin_r, cylin_z, theta, cylin_r;
            std::vector<double> q1_list;
            std::vector<double> q5_list;

            //based on the gripper_pose, generate the sample sets for the arm configuration
            GenerateSamplesForArmConf(gripper_pose, gripper_dir, min_cylin_r, max_cylin_r, cylin_z, q1_list, q5_list, theta);

            tf::Quaternion gripper_q(planned_grasp_vector.vector[i].gripper_pose.orientation.x,planned_grasp_vector.vector[i].gripper_pose.orientation.y,planned_grasp_vector.vector[i].gripper_pose.orientation.z,planned_grasp_vector.vector[i].gripper_pose.orientation.w); 
            tf::Matrix3x3 gripper_m(gripper_q);
            tf::Vector3 x_axis = gripper_m.getColumn(0);
            normal_x = gripper_dir[0];
            normal_y = gripper_dir[1];
            normal_z = gripper_dir[2];

            

            for(cylin_r = min_cylin_r; cylin_r < max_cylin_r; cylin_r += 0.005){
                #ifdef DEBUG_
                    ROS_INFO_STREAM("testing with cylin_r = "<<cylin_r<<" theta="<<theta);
                #endif
                // === JOINT SPACE CHECK ===
                //joint limit can be checked with any q1, q5, thus set them both as 0
                cylin_test.setQ1(-1);
                cylin_test.setQ5(-1);
                cylin_test.setR(cylin_r);
                cylin_test.setZ(cylin_z);
                cylin_test.setTheta(theta);
                cylin_test.printValues("sanity check:(cylin)");
                pre_cylin_test = cylin_test;
                pre_cylin_test.setZ(pre_cylin_test.z()+0.075*normal_z);
                pre_cylin_test.setR(pre_cylin_test.r()-0.075*sqrt(pow(normal_x,2)+pow(normal_y, 2))/*/normal_z*/);
                joint_test = cylin_test.toJointspace();
                //joint_test.subtractOffset();
                pre_joint_test = pre_cylin_test.toJointspace();
                //pre_joint_test.subtractOffset();
                if(!joint_test.isReachable() || !pre_joint_test.isReachable()){
                    #ifdef DEBUG_
                    joint_test.printValues("JOINT_TEST: out of limit");
                    pre_joint_test.printValues("PRE_JOINT_TEST: out of limit");
                    ROS_ERROR("this grasping config exceeds joint limitations.");
                    #endif
                    //check next cylin_r
                    continue;
                }
                // === JOINT SPACE CHECK ===
                joint_test.printValues("sannity check");
                for(int j = 0; j < q1_list.size(); j++){
                    double q1 = q1_list[j];
                    q1 += ykin::JOINT_OFFSETS[0]; 
                    is_base_q1_ok = true;
                    #ifdef DEBUG_
                        ROS_INFO_STREAM("testing with q1 = "<<q1);
                    #endif
                    //the gripper pose is vertical down, the base could be on any direction depending on q5
                    if(theta - M_PI < 0.1 && theta - M_PI > -0.1){
                        #ifdef DEBUG_
                            //ROS_INFO_STREAM("gripper is arm_upright");
                        #endif
                        arm_upright = true;

                        for(int k = 0; k < q5_list.size(); k ++){
                            double q5 = q5_list[k];
                            is_base_q5_ok = true;
                            
                            q5 += ykin::JOINT_OFFSETS[4];
                            #ifdef DEBUG_
                            ROS_INFO_STREAM("testing with q5 = "<<q5);
                            #endif
                            //q5 = 0 - q5;
                            cylin_test.setQ1(q1);
                            cylin_test.setQ5(q5);
                            cylin_test.setR(cylin_r);
                            cylin_test.setZ(cylin_z);
                            cylin_test.setTheta(theta);

                            joint_test = cylin_test.toJointspace();
                            //joint_test.subtractOffset();
                            current_cylin_pose = cylin_test;
                            final_cylin_test = cylin_test;
                            
                            // === Robot Base collision check with the environment ===
                            FromGraspPoseToBasePose(gripper_pose, gripper_dir, cylin_r, q1, q5, theta, base_pose);
                            #ifdef DEBUG_
                                // ROS_INFO_STREAM("check base_pose:"<<base_pose.position.x<<","<<base_pose.position.y<<","<<base_pose.position.z);
                            #endif
                            
                            arc::polygon_2 base_poly;
                            arc::SE2Pose base_se2 = arc::GPoseToSE2(base_pose);
	                        arc::movePolygon(foot_print_, base_poly, base_se2.x, base_se2.y, base_se2.yaw );	
                            std::stringstream ss; 
    	                    ss << "CHECK_BASE_" <<std::rand();
    	                    std::string name = ss.str();
                            int space;
                            // === Robot Base collision check with the environment ===
                            // arc::gazeboUtility::spawnPassThroughCuboid(nh_, YOUBOT_BASE_LENGTH, 
		                    // YOUBOT_BASE_WIDTH, 0.002, base_pose, name, false);
                        //std::cin>>space;
                            if(!base_planning_scene.isCollisionFreeSlow(base_poly))
                            {
                                is_base_q5_ok = false;
                            }


                            if(is_base_q5_ok){

                                final_base_pose_list.push_back(base_pose);
                                current_cylin_pose_list.push_back(cylin_test);
                
                                
                            }

                        }
                        if(!is_base_q5_ok){
                            is_base_q1_ok = false;
                        }

                    }else{
                        // the gripper leans to one side, the base should be on that side
                        #ifdef DEBUG_
                            ROS_INFO_STREAM("gripper leans to one side");
                        #endif
                        arm_upright = false;
                        double water_x_axis_y = sqrt(pow(normal_x,2)/( pow(normal_x,2) + pow(normal_y,2) ));
                        double water_x_axis_x = -normal_y / normal_x * water_x_axis_y;
                        tf::Vector3 water_x_axis(water_x_axis_x, water_x_axis_y,0);
                        double q5 = -acos(x_axis.dot(water_x_axis) / (x_axis.length() * water_x_axis.length()));
                        //q5 -= ykin::JOINT_OFFSETS[4];    
                        ROS_WARN_STREAM("leans to one side:"<<q5);
                        cylin_test.setQ1(q1);
                        cylin_test.setQ5(q5);
                        cylin_test.setR(cylin_r);
                        cylin_test.setZ(cylin_z);
                        cylin_test.setTheta(theta);
                        joint_test = cylin_test.toJointspace();
                        joint_test.subtractOffset();
                        current_cylin_pose = cylin_test;
                        
                        // Robot base collision check with the environment
                        FromGraspPoseToBasePose(gripper_pose, gripper_dir, cylin_r, q1, q5, theta, base_pose);
                        arc::polygon_2 base_poly;
                        arc::SE2Pose base_se2 = arc::GPoseToSE2(base_pose);
                        arc::movePolygon(foot_print_, base_poly, base_se2.x, base_se2.y, base_se2.yaw );	
                        if(!base_planning_scene.isCollisionFree(base_poly))
                        {
                            is_base_q1_ok = false;
                        }
                        // === Robot Base collision check with the environment ===  

                        if(is_base_q1_ok){
                            
                            std::vector<double> test_joint_values{joint_test[0], joint_test[1], joint_test[2], joint_test[3], joint_test[4]};
                            joint_test.printValues("leans to one side, plan joint test");
                            if(planInAdvance(base_pose, test_joint_values, nh_)){ 
                                cylin_test.setZ(cylin_test.z()+0.075*normal_z);
                                cylin_test.setR(cylin_test.r()-0.075*sqrt(pow(normal_x,2)+pow(normal_y, 2))/*/normal_z*/);
                                joint_test = cylin_test.toJointspace();
                                joint_test.subtractOffset();
                                test_joint_values = {joint_test[0], joint_test[1], joint_test[2], joint_test[3], joint_test[4]};
                                
                                if(planInAdvance(base_pose, test_joint_values, nh_)){ 
                                    cylin_test.setZ(cylin_test.z()-0.075*normal_z);
                                    cylin_test.setR(cylin_test.r()+0.075*sqrt(pow(normal_x,2)+pow(normal_y, 2))/*/normal_z*/);
                                    is_base_r_ok = true;
                                    break;    
                                        
                                }else{
                                    is_base_q1_ok = false;
                                }
                            }else{
                                is_base_q1_ok = false;
                            }      

                        }


                    }
                    

                    if(is_base_q1_ok){
                        is_base_r_ok = true;
                        //break;
                    } 
                }
                
                double shortest_dist_to_rest = 10000000;
                double temp_dist = 0;
                while(final_base_pose_list.size() > 0){ 
                    shortest_dist_to_rest = 10000000;
                    auto base_pose_deletor = final_base_pose_list.begin();
                    auto cylin_pose_deletor = current_cylin_pose_list.begin();
                    is_base_q5_ok = true;

                    for(int p = 0; p < final_base_pose_list.size(); p++){
                        temp_dist = sqrt((final_base_pose_list[p].position.x - rest_base_pose_.position.x)*(final_base_pose_list[p].position.x - rest_base_pose_.position.x) + (final_base_pose_list[p].position.y - rest_base_pose_.position.y)*(final_base_pose_list[p].position.y - rest_base_pose_.position.y));
                        // ROS_WARN_STREAM("base_pose:"<<final_base_pose_list[p].position.x<<","<<final_base_pose_list[p].position.y<<", temp_dist:"<<temp_dist);
                        if(temp_dist < shortest_dist_to_rest){
                            shortest_dist_to_rest = temp_dist;
                            base_pose = final_base_pose_list[p];
                            cylin_test = current_cylin_pose_list[p];
                            cylin_pose_deletor = current_cylin_pose_list.begin() + p;
                            base_pose_deletor = final_base_pose_list.begin() + p;
                            is_base_q1_ok = true;
                        }
                    }
                    joint_test = cylin_test.toJointspace();
                    //joint_test.subtractOffset();
                    std::vector<double> test_joint_values{joint_test[0], joint_test[1], joint_test[2], joint_test[3], joint_test[4]};
                                
                    if(planInAdvance(base_pose, test_joint_values, nh_)){ 
                        cylin_test.setZ(cylin_test.z()+0.075*normal_z);
                        cylin_test.setR(cylin_test.r()-0.075*sqrt(pow(normal_x,2)+pow(normal_y, 2)));
                        pre_joint_test = cylin_test.toJointspace();
                        //pre_joint_test.subtractOffset();
                        test_joint_values = {joint_test[0], joint_test[1], joint_test[2], joint_test[3], joint_test[4]};
                        if(!joint_test.isReachable() || !pre_joint_test.isReachable()){
                            is_base_q5_ok = false;
                        }else{ 
                            if(planInAdvance(base_pose, test_joint_values, nh_)){ 
                                cylin_test.setZ(cylin_test.z()-0.075*normal_z);
                                cylin_test.setR(cylin_test.r()+0.075*sqrt(pow(normal_x,2)+pow(normal_y, 2)));

                                is_base_q1_ok = true;
                                is_base_r_ok = true;
                                break;    
                                

                            }else{
                                is_base_q5_ok = false;
                            }
                        }
                    }else{
                        is_base_q5_ok = false;
                    }
                    if(!is_base_q5_ok){
                        final_base_pose_list.erase(base_pose_deletor);
                        current_cylin_pose_list.erase(cylin_pose_deletor);
                    }
                }
                
                if(!is_base_q1_ok){
                    is_base_r_ok = false;
                }
                if(is_base_r_ok){
                    is_grasp_pose_ok = true;
                    break;
                }
               
            }
            if(is_grasp_pose_ok){
                found_ok_config = true;
                break;
            }
        }
        if(found_ok_config){

        	result_.final_base_pose = base_pose;
        	result_.r = cylin_test.r();
        	result_.z = cylin_test.z();
        	result_.q1 = cylin_test.q1();
        	result_.q5 = cylin_test.q5();
        	result_.theta = cylin_test.theta();
            result_.q2 = joint_test[1];
            result_.q3 = joint_test[2];
            result_.q4 = joint_test[3];
            result_.q1_pre = pre_joint_test[0];
            result_.q2_pre = pre_joint_test[1];
            result_.q3_pre = pre_joint_test[2];
            result_.q4_pre = pre_joint_test[3];
            result_.q5_pre = pre_joint_test[4];
        	result_.normal_x = normal_x;
        	result_.normal_y = normal_y;
        	result_.normal_z = normal_z;
        	as_.setSucceeded(result_);
        }
    }
}


bool ManipulationServer::planInAdvance(geometry_msgs::Pose final_base_pose, std::vector<double> test_group_variable_values, ros::NodeHandle& node_handle){
    return true;
}


void ManipulationServer::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

  


int main(int argc, char** argv)
{
    ros::init(argc, argv, "manipulation_action");

    ManipulationServer manipulation_planner_0("manipulation_action");
    //ManipulationServer manipulation_planner_1("manipulation_action_1");    
    ros::spin();

    return 0;
}
