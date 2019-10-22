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
    #ifdef USE_VREP_
    if(theta - M_PI < 0.1 && theta - M_PI > -0.1){
        tip_z = gripper_pose.position.z - normal_z*0.12;
    }else{
        tip_z = gripper_pose.position.z - normal_z*0.02;
    }
    #else 
        tip_z = gripper_pose.position.z - normal_z*0.02;
    #endif

    cylin_z = tip_z - ykin::H2 + ykin::L0; // relative to arm_link_0
    
    std::cout<<"cylin_z:"<<cylin_z<<std::endl;
    int space;
    //std::cin>>space;
    //generate the range for cylin_r(min_cylin_r and max_cylin_r)  
    double axis4_to_axis2_z = tip_z - ykin::H2 - ykin::L4 * cos(theta);
    double axis4_axis2_max_dist = ykin::L2 + ykin::L3;
    max_cylin_r = sqrt(axis4_axis2_max_dist*axis4_axis2_max_dist - axis4_to_axis2_z*axis4_to_axis2_z) + ykin::L4*sin(theta) + ykin::L1 + 0.02;
    
    
    if(theta - M_PI < 0.1 && theta - M_PI > -0.1){
        min_cylin_r = 0.24*sin(theta) + 0.033 + 0.1;
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
    for(q5 = 3.00194; q5 <= 5.84685299418; q5 += 0.3){
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
    std::string test;
    //std::cin>>test;
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
    //moveit::planning_interface::MoveGroup group("youbot_arm");
    
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
            //arc::gazeboUtility::deleteModel(nh_, last_name);
            //arc::gazeboUtility::spawnSDFModel(nh_, "/home/wei/.gazebo/models/006_mustard_bottle/model.sdf", gripper_pose, name);
            
            //arc::gazeboUtility::spawnPassThroughCuboid(nh_, 0.04, 0.004, 0.04, 
            // gripper_pose.position.x,
            // gripper_pose.position.y,
            // gripper_pose.position.z,
            // gripper_pose.orientation.x,
            // gripper_pose.orientation.y, 
            // gripper_pose.orientation.z, 
            // gripper_pose.orientation.w,             
            // name, true
            // );		
            //fly_gripper_pub.publish(gripper_pose);
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
                pre_cylin_test.setZ(pre_cylin_test.z()+0.07*normal_z);
                pre_cylin_test.setR(pre_cylin_test.r()-0.07*sqrt(pow(normal_x,2)+pow(normal_y, 2))/*/normal_z*/);
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
                            ROS_INFO_STREAM("gripper is arm_upright");
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
                                ROS_INFO_STREAM("base_pose:"<<base_pose.position.x<<","<<base_pose.position.y<<","<<base_pose.position.z);
                            #endif
                            
                            arc::polygon_2 base_poly;
                            arc::SE2Pose base_se2 = arc::GPoseToSE2(base_pose);
	                        arc::movePolygon(foot_print_, base_poly, base_se2.x, base_se2.y, base_se2.yaw );	
                            std::stringstream ss; 
    	                    ss << "CHECK_BASE_" <<std::rand();
    	                    std::string name = ss.str();
                            int space;
                            // === Robot Base collision check with the environment ===
                            //arc::gazeboUtility::spawnPassThroughCuboid(nh_, YOUBOT_BASE_LENGTH, 
		                    //YOUBOT_BASE_WIDTH, 0.002, base_pose, name, false);
                        //std::cin>>space;
                            if(!base_planning_scene.isCollisionFree(base_poly))
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
                            // cylin_test.setZ(cylin_test.z()+0.07*normal_z);
                            // cylin_test.setR(cylin_test.r()-0.07*sqrt(pow(normal_x,2)+pow(normal_y, 2))/*/normal_z*/);
                            // joint_test = cylin_test.toJointspace();
                            // joint_test.subtractOffset();
                            std::vector<double> test_joint_values{joint_test[0], joint_test[1], joint_test[2], joint_test[3], joint_test[4]};
                            joint_test.printValues("leans to one side, plan joint test");
                            if(planInAdvance(base_pose, test_joint_values, nh_)){ 
                                cylin_test.setZ(cylin_test.z()+0.07*normal_z);
                                cylin_test.setR(cylin_test.r()-0.07*sqrt(pow(normal_x,2)+pow(normal_y, 2))/*/normal_z*/);
                                joint_test = cylin_test.toJointspace();
                                joint_test.subtractOffset();
                                test_joint_values = {joint_test[0], joint_test[1], joint_test[2], joint_test[3], joint_test[4]};
                                
                                if(planInAdvance(base_pose, test_joint_values, nh_)){ 
                                    cylin_test.setZ(cylin_test.z()-0.07*normal_z);
                                    cylin_test.setR(cylin_test.r()+0.07*sqrt(pow(normal_x,2)+pow(normal_y, 2))/*/normal_z*/);
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
                        cylin_test.setZ(cylin_test.z()+0.07*normal_z);
                        cylin_test.setR(cylin_test.r()-0.07*sqrt(pow(normal_x,2)+pow(normal_y, 2)));
                        pre_joint_test = cylin_test.toJointspace();
                        //pre_joint_test.subtractOffset();
                        test_joint_values = {joint_test[0], joint_test[1], joint_test[2], joint_test[3], joint_test[4]};
                        
                        if(planInAdvance(base_pose, test_joint_values, nh_)){ 
                            cylin_test.setZ(cylin_test.z()-0.07*normal_z);
                            cylin_test.setR(cylin_test.r()+0.07*sqrt(pow(normal_x,2)+pow(normal_y, 2)));

                            is_base_q1_ok = true;
                            is_base_r_ok = true;
                            break;    
                            

                        }else{
                            is_base_q5_ok = false;
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
    
//     ROS_WARN_STREAM("planInAvance:"<<test_group_variable_values[0]<<","<<test_group_variable_values[1]<<","<<test_group_variable_values[2]<<","<<test_group_variable_values[3]<<","<<test_group_variable_values[4]);
//     std::string robot_description_name, planning_scene_name, joint_group_name, plan_plugin_name;
//     if(action_name_ == "manipulation_action_0"){
//         robot_description_name = "/youbot0/robot_description";
//         planning_scene_name = "/youbot0/planning_scene";
//         joint_group_name = "youbot_arm0";
//         plan_plugin_name = "/youbot0/move_group_0/planning_plugin";
//     }else if(action_name_ == "manipulation_action_1"){
//         robot_description_name = "/youbot1/robot_description";
//         planning_scene_name = "/youbot1/planning_scene";
//         joint_group_name = "youbot_arm1";
//         plan_plugin_name = "/youbot1/move_group_1/planning_plugin";        
//     }
    
//     robot_model_loader::RobotModelLoader robot_model_loader(robot_description_name);
//     robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

//     ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>(planning_scene_name, 1);
//     while(planning_scene_diff_publisher.getNumSubscribers() < 1)
//     {
//     ros::WallDuration sleep_t(0.5);
//     sleep_t.sleep();
//     }

//     planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
//     moveit_msgs::PlanningScene planning_scene_msg;
//     for(int i = 0;i < manipulation_scene_.scene_object_list.size();i ++)
//     {
//         if(!is_synchronize_){ 
//             if(i == last_target_object_index_){
//                 continue;
//             }
//         }
//         moveit_msgs::CollisionObject bin_object;
//         bin_object.header.frame_id = "world";
//         bin_object.id = manipulation_scene_.scene_object_list[i].object_name.data;
//         shapes::Mesh* m;

//         if(i < manipulation_scene_.scene_object_list.size()-1){ 
//             if(manipulation_scene_.scene_object_list[i].object_type.data == "cube"){ 
//                 m = shapes::createMeshFromResource("package://youbot_grasp/models/cube_smallunit.obj");
//             }else if(manipulation_scene_.scene_object_list[i].object_type.data == "T_block"){
//                 m = shapes::createMeshFromResource("package://luh_youbot_description/meshes/T.stl");
//             }else if(manipulation_scene_.scene_object_list[i].object_type.data == "L_block"){
//                 m = shapes::createMeshFromResource("package://luh_youbot_description/meshes/L.stl");
//             }else if(manipulation_scene_.scene_object_list[i].object_type.data == "pillar"){
//                 m = shapes::createMeshFromResource("package://luh_youbot_description/meshes/pillar.stl");
//             }
//         }else{
//             m = shapes::createMeshFromResource("package://youbot_grasp/models/floor_smallunit.obj");            
//         }

//         shape_msgs::Mesh bin_mesh;
//         shapes::ShapeMsg bin_mesh_msg;
//         shapes::constructMsgFromShape(m,bin_mesh_msg);
//         bin_mesh = boost::get<shape_msgs::Mesh>(bin_mesh_msg);


//         bin_object.meshes.push_back(bin_mesh);
//         geometry_msgs::Pose bin_pose;
//         bin_pose = manipulation_scene_.scene_object_list[i].object_pose;
//         bin_object.mesh_poses.push_back(bin_pose);
//         bin_object.operation = bin_object.APPEND; 

//         //planning_scene_msg.world.collision_objects.push_back(bin_object); 
//         planning_scene->processCollisionObjectMsg(bin_object);
//     }
    
    
    
    
//     // Construct a loader to load a planner, by name. 
//     // Note that we are using the ROS pluginlib library here.
//     boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
//     planning_interface::PlannerManagerPtr planner_instance;
//     std::string planner_plugin_name;

//     // We will get the name of planning plugin we want to load
//     // from the ROS param server, and then load the planner
//     // making sure to catch all exceptions.
//     if (!node_handle.getParam(plan_plugin_name, planner_plugin_name))
//     ROS_FATAL_STREAM("Could not find planner plugin name");
//     try
//     {
//     planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
//     }
//     catch(pluginlib::PluginlibException& ex)
//     {
//     ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
//     }
//     try
//     {
//     planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
//     if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
//         ROS_FATAL_STREAM("Could not initialize planner instance");
//         ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
//     }
//     catch(pluginlib::PluginlibException& ex)
//     {
//     const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
//     std::stringstream ss;
//     for (std::size_t i = 0 ; i < classes.size() ; ++i)
//         ss << classes[i] << " ";
//     ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
//                         << "Available plugins: " << ss.str());
//     }



//     // setup planning instance
//     planning_interface::MotionPlanRequest req;
//     planning_interface::MotionPlanResponse res;


//     std::vector<double> tolerance_pose(5, 0.01);
//     std::vector<double> tolerance_angle(5, 0.01);


//     req.group_name = joint_group_name;

//     // Joint Space Goals

//     robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();


//     std::vector<double> joint_values(5, 0.0);
//     joint_values[0] = 3.03;
//     joint_values[1] = 0.01;
//     joint_values[2] = -1.047196667;
//     joint_values[3] = 2.4;
//     joint_values[4] = 2.9845;
//     const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup(joint_group_name);
//     robot_state.setJointGroupPositions(joint_model_group, joint_values);
//     std::vector<double> gripper_joint_l{0.035};
//     std::vector<double> gripper_joint_r{0.035};
    
//     robot_state.setJointPositions("gripper_finger_joint_l", gripper_joint_l);
//     robot_state.setJointPositions("gripper_finger_joint_r", gripper_joint_r);    
//     //robot_state.printStatePositions();
//     if(action_name_ == "manipulation_action_0"){
//         robot_state.setVariablePosition("world_joint0/trans_x", final_base_pose.position.x);
//         robot_state.setVariablePosition("world_joint0/trans_y", final_base_pose.position.y);
//         robot_state.setVariablePosition("world_joint0/trans_z", 0.060431);
//         robot_state.setVariablePosition("world_joint0/rot_x", final_base_pose.orientation.x);
//         robot_state.setVariablePosition("world_joint0/rot_y", final_base_pose.orientation.y);
//         robot_state.setVariablePosition("world_joint0/rot_z", final_base_pose.orientation.z);
//         robot_state.setVariablePosition("world_joint0/rot_w", final_base_pose.orientation.w);
//     }else if(action_name_ == "manipulation_action_1"){
//         robot_state.setVariablePosition("world_joint1/trans_x", final_base_pose.position.x);
//         robot_state.setVariablePosition("world_joint1/trans_y", final_base_pose.position.y);
//         robot_state.setVariablePosition("world_joint1/trans_z", 0.060431);
//         robot_state.setVariablePosition("world_joint1/rot_x", final_base_pose.orientation.x);
//         robot_state.setVariablePosition("world_joint1/rot_y", final_base_pose.orientation.y);
//         robot_state.setVariablePosition("world_joint1/rot_z", final_base_pose.orientation.z);
//         robot_state.setVariablePosition("world_joint1/rot_w", final_base_pose.orientation.w);
//     }
//     //robot_state.setJointPositions("world_joint", );

//     //planning_scene->setCurrentState(robot_state);
//     moveit_msgs::RobotState start_state_msg;
//     moveit::core::robotStateToRobotStateMsg(robot_state, start_state_msg);
//     req.start_state = start_state_msg;

//     const std::vector<std::string>& nm = robot_state.getVariableNames();
//     for (std::size_t i = 0; i < nm.size(); ++i)
//         ROS_INFO_STREAM(nm[i]);


//     // Now, setup a joint space goal
//     robot_state::RobotState goal_state(robot_model);
//     if(action_name_ == "manipulation_action_0"){
//         goal_state.setVariablePosition("world_joint0/trans_x", final_base_pose.position.x);
//         goal_state.setVariablePosition("world_joint0/trans_y", final_base_pose.position.y);
//         goal_state.setVariablePosition("world_joint0/trans_z", 0.060431);
//         goal_state.setVariablePosition("world_joint0/rot_x", final_base_pose.orientation.x);
//         goal_state.setVariablePosition("world_joint0/rot_y", final_base_pose.orientation.y);
//         goal_state.setVariablePosition("world_joint0/rot_z", final_base_pose.orientation.z);
//         goal_state.setVariablePosition("world_joint0/rot_w", final_base_pose.orientation.w);
//     }else if(action_name_ == "manipulation_action_1"){
//         goal_state.setVariablePosition("world_joint1/trans_x", final_base_pose.position.x);
//         goal_state.setVariablePosition("world_joint1/trans_y", final_base_pose.position.y);
//         goal_state.setVariablePosition("world_joint1/trans_z", 0.060431);
//         goal_state.setVariablePosition("world_joint1/rot_x", final_base_pose.orientation.x);
//         goal_state.setVariablePosition("world_joint1/rot_y", final_base_pose.orientation.y);
//         goal_state.setVariablePosition("world_joint1/rot_z", final_base_pose.orientation.z);
//         goal_state.setVariablePosition("world_joint1/rot_w", final_base_pose.orientation.w);
//     }
//     joint_values[0] = test_group_variable_values[0];
//     joint_values[1] = test_group_variable_values[1];
//     joint_values[2] = test_group_variable_values[2];
//     joint_values[3] = test_group_variable_values[3];
//     joint_values[4] = test_group_variable_values[4]; 
//     goal_state.setJointGroupPositions(joint_model_group, joint_values);
//     goal_state.setJointPositions("gripper_finger_joint_l", gripper_joint_l);
//     goal_state.setJointPositions("gripper_finger_joint_r", gripper_joint_r);    
//     // goal_state.printStatePositions();
//     moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
//     req.goal_constraints.clear();
//     req.goal_constraints.push_back(joint_goal);


//     //ROS_INFO("before get context");
//     // Call the planner and visualize the trajectory
//     planning_scene->getPlanningSceneMsg(planning_scene_msg);
//    // planning_scene_diff_publisher.publish(planning_scene_msg);

//     planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

//     //ROS_INFO("finish setup");
//     context->solve(res);

//     if(res.error_code_.val != res.error_code_.SUCCESS)
//     {
//     ROS_ERROR("Could not compute plan successfully");
//     return false;
//     }

//     ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1000, true);
//     moveit_msgs::DisplayTrajectory display_trajectory;
//     moveit_msgs::MotionPlanResponse response;

//     //ROS_INFO("Visualizing the trajectory");
//     res.getMessage(response);
//     ROS_WARN_STREAM("response:"<<response.error_code.val);
//     display_trajectory.trajectory_start = response.trajectory_start;
//     display_trajectory.trajectory.push_back(response.trajectory);

//     ros::spinOnce();
//     display_publisher.publish(display_trajectory);

//     ros::spinOnce();
//     //END_TUTORIAL

//     //sleep_time.sleep();
//     //ROS_INFO("Done");
//     planner_instance.reset();
//     for(int i = 0;i < manipulation_scene_.scene_object_list.size();i ++)
//     {
//         moveit_msgs::CollisionObject bin_object;
//         bin_object.header.frame_id = "world";
//         bin_object.id = manipulation_scene_.scene_object_list[i].object_name.data;
//         shapes::Mesh* m;
//         if(i < manipulation_scene_.scene_object_list.size()-1){ 
//             if(manipulation_scene_.scene_object_list[i].object_type.data == "cube"){ 
//                 m = shapes::createMeshFromResource("package://youbot_grasp/models/cube_smallunit.obj");
//             }else if(manipulation_scene_.scene_object_list[i].object_type.data == "T_block"){
//                 m = shapes::createMeshFromResource("package://youbot_grasp/models/T_block.obj");
//             }else if(manipulation_scene_.scene_object_list[i].object_type.data == "L_block"){
//                 m = shapes::createMeshFromResource("package://youbot_grasp/models/L_block.obj");
//             }else if(manipulation_scene_.scene_object_list[i].object_type.data == "pillar"){
//                 m = shapes::createMeshFromResource("package://luh_youbot_description/meshes/pillar.stl");
//             }            
//         }else{
//             m = shapes::createMeshFromResource("package://youbot_grasp/models/floor_smallunit.obj");            
//         }
//         shape_msgs::Mesh bin_mesh;
//         shapes::ShapeMsg bin_mesh_msg;
//         shapes::constructMsgFromShape(m,bin_mesh_msg);
//         bin_mesh = boost::get<shape_msgs::Mesh>(bin_mesh_msg);


//         bin_object.meshes.push_back(bin_mesh);
//         geometry_msgs::Pose bin_pose;
//         bin_pose = manipulation_scene_.scene_object_list[i].object_pose;
//         bin_object.mesh_poses.push_back(bin_pose);
//         bin_object.operation = bin_object.ADD; 

//         //planning_scene_msg.world.collision_objects.push_back(bin_object); 
//         planning_scene->processCollisionObjectMsg(bin_object);
//     }
//     planning_scene->getPlanningSceneMsg(planning_scene_msg);
//     //planning_scene_diff_publisher.publish(planning_scene_msg);
//     return true;
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
