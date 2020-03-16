#include "arcl_youbot_vision/util.hpp"
#include "arcl_youbot_vision/target_selector.hpp"


target_selector::target_selector(ros::NodeHandle& nh){
    nh_ = nh;
    

}

void target_selector::segment_mask_callback(const arcl_youbot_msgs::AllMask& msg){
    for(int i = 0; i < msg.mask_list.size(); i++){
        // for each single mask
        std::vector<std::pair<int, int>> single_mask;
        for(int col = 0; col < msg.mask_list[i].col_list.size(); col++){
            single_mask.push_back(std::make_pair(msg.mask_list[i].col_list[col], msg.mask_list[i].row_list[col]));
        }
        mask_list_.push_back(single_mask);
    }
    is_segment_mask_ready_ = true;
    
}

void target_selector::get_current_scene(){
    
    caminfo_.topic_image = rgb_topic_;
    caminfo_.topic_depth = depth_topic_;
    caminfo_.topic_camerainfo = camera_info_topic_;

    GetInputFromCamera(nh_, curr_image_, curr_depth_, cloud_camera_, caminfo_);
    cloud_camera_.header.frame_id = "/camera_rgb_optical_frame";
    tf::TransformListener listener;
    pcl_ros::transformPointCloud (base_frame_, ros::Time(0), 
                       cloud_camera_, 
                       camera_frame_, 
                       cloud_base_, 
                       listener);
    publish_pointcloud(nh_, "target_cloud", cloud_base_);
}

void target_selector::get_instance_mask(){
    ros::Publisher image_segment_pub = nh_.advertise<sensor_msgs::Image>(image_segment_topic_, 3);
    ros::Subscriber mask_sub = nh_.subscribe(segment_mask_topic_, 1, &target_selector::segment_mask_callback, this);
    is_segment_mask_ready_ = false;
    while(1){
        if(image_segment_pub.getNumSubscribers() > 0){
            image_segment_pub.publish(curr_image_);
            ros::spinOnce();
            std::cout<<"publish the image segment message"<<std::endl;
            if(is_segment_mask_ready_ == true){
            std::cout<<"get the segment mask, continue"<<std::endl;
            break;
            }
        }else{ 
        std::cout<<"no image segment subscriber"<<std::endl;
        }
        
    }
}
    



        
bool target_selector::is_mask_separated(std::vector<std::pair<int, int>> mask_ins, int miss_num_threshold){
    bool is_separated = false;
    int min_x = 10000;
    int max_x = -1;
    int min_y = 10000;
    int max_y = -1;
    std::map<int, bool> x_map, y_map;
    std::cout<<"mask_ins size:"<<mask_ins.size()<<std::endl;
    for(int i = 0; i < mask_ins.size(); i++){
        if(mask_ins[i].first < min_x){
            min_x = mask_ins[i].first;
        }    
        if(mask_ins[i].first > max_x){
            max_x = mask_ins[i].first;
        }
        if(mask_ins[i].second < min_y){
            min_y = mask_ins[i].second;
        }    
        if(mask_ins[i].second > max_y){
            max_y = mask_ins[i].second;
        }
        if(x_map.find(mask_ins[i].first) == x_map.end()){
            x_map[mask_ins[i].first] = true;
        }
        if(y_map.find(mask_ins[i].second) == y_map.end()){
            y_map[mask_ins[i].second] = true;
        }
    }

    int x_miss_num = 0;
    int y_miss_num = 0;

    for(int test_x = min_x; test_x <= max_x; test_x ++){
        if(x_map.find(test_x) == x_map.end()){
            x_miss_num ++;
        }
    }
    if(x_miss_num > miss_num_threshold){
        return true;
    }
    for(int test_y = min_y; test_y <= max_y; test_y ++){
        if(y_map.find(test_y) == y_map.end()){
            y_miss_num ++;
        }
    }
    if(y_miss_num > miss_num_threshold){
        return true;
    }

    return false;


    
}



void target_selector::get_next_target_instance(){

    get_current_scene();
    get_instance_mask();

    
    //calculate closest instance that is also graspable

    //step 1, get the closest one
    // pcl::PointCloud<pcl::PointXYZRGB> curr_cloud;
    cv_bridge::CvImagePtr cv_dep = cv_bridge::toCvCopy(curr_depth_, curr_depth_.encoding);

    std::vector<std::tuple<double, double, double>> instance_center_list;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> instance_cloud_list;
    for(int i = 0; i < mask_list_.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGB> curr_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> curr_cloud_base;
        curr_cloud.header.frame_id = camera_frame_;
        std::tuple<double, double, double> instance_center;
        double center_x = 0;
        double center_y = 0;
        double center_z = 0;
        int invalid_point_num = 0;
        for(int p = 0; p < mask_list_[i].size(); p++){
            pcl::PointXYZRGB pt;
            int r = mask_list_[i][p].second;
            int c = mask_list_[i][p].first;

            float z = (float)(cv_dep->image).at<uint16_t>(r,c) * caminfo_.depth_scale;
            if( z < 0.10 )  
            {
                pt.x = 0; pt.y = 0; pt.z = 0;  
                invalid_point_num ++;          
            }
            float x = (((float)c) - caminfo_.camera_K[2]) / caminfo_.camera_K[0] * z;
            float y = (((float)r) - caminfo_.camera_K[5]) / caminfo_.camera_K[4] * z;


            pt.z = z;
            pt.x = x;
            pt.y = y;
            cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(curr_image_, curr_image_.encoding);    
            pt.r = cv_img->image.at<cv::Vec3b>(r,c)[0];
            pt.g = cv_img->image.at<cv::Vec3b>(r,c)[1];
            pt.b = cv_img->image.at<cv::Vec3b>(r,c)[2];
            curr_cloud.push_back(pt);
            center_x += pt.x;
            center_y += pt.y;
            center_z += pt.z;


        }

        tf::TransformListener listener;
        pcl_ros::transformPointCloud (base_frame_, ros::Time(0), 
                        curr_cloud, 
                        camera_frame_, 
                        curr_cloud_base, 
                        listener);
        center_x = 0;
        center_y = 0;
        center_z = 0;
        for(int s = 0; s < curr_cloud_base.points.size(); s++){
            center_x += curr_cloud_base.points[s].x;
            center_y += curr_cloud_base.points[s].y;
            center_z += curr_cloud_base.points[s].z;
        }
        center_x = center_x / (mask_list_[i].size() - invalid_point_num);
        center_y = center_y / (mask_list_[i].size() - invalid_point_num);
        center_z = center_z / (mask_list_[i].size() - invalid_point_num);
        instance_center = std::make_tuple(center_x, center_y, center_z);
        instance_center_list.push_back(instance_center);
        instance_cloud_list.push_back(curr_cloud_base);
    }

    std::cout<<"after get instance_cloud_list"<<std::endl;
    double closest_dist = 100000000;
    int closest_index = 0;
    bool is_chosen = false;
    std::map<int, bool> discarded_map;
    double highest = -1;
    int highest_index = 0;
    while(!is_chosen){ 
        closest_dist = 100000000;
        for(int i = 0; i < instance_center_list.size(); i ++){
            if(discarded_map.find(i) != discarded_map.end()) continue;
            std::cout<<"x:"<<std::get<0>(instance_center_list[i])<<", y:"<<std::get<1>(instance_center_list[i])<<", z:"<<std::get<2>(instance_center_list[i])<<std::endl;
            if(std::get<2>(instance_center_list[i]) > highest){
                highest = std::get<2>(instance_center_list[i]);
                highest_index = i;
            }
            double curr_dist = std::sqrt(std::get<0>(instance_center_list[i]) * std::get<0>(instance_center_list[i]) + std::get<1>(instance_center_list[i]) * std::get<1>(instance_center_list[i]) + std::get<2>(instance_center_list[i]) * std::get<2>(instance_center_list[i]));
            if(curr_dist < closest_dist){
                closest_dist = curr_dist;
                closest_index = i;
            }
        }

        //step 1.5 check whether current instance is spatially separated, if yes, continue with next instance (current instance is blocked)
        std::cout<<"highest_index:"<<highest_index<<std::endl;
        std::cout<<"mask_list_ size:"<<mask_list_.size()<<std::endl;
        // if(is_mask_separated(mask_list_[highest_index], 10)){
        //     // current instance is potentially separated, discard, go to next one
        //     discarded_map[highest_index] = true;
        //     continue;
        // }

        std::cout<<"starting calculating surface normal"<<std::endl;
        //step 2, get its top surface through normal
        pcl::PointCloud<pcl::PointXYZRGB> target_cloud = instance_cloud_list[highest_index];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_input_cloud( new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        copyPointCloud(target_cloud, *normal_input_cloud);
        // publish_pointcloud(nh_, "target_cloud", *normal_input_cloud);
        ne.setInputCloud (normal_input_cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);

        // Compute the features
        ne.compute (*cloud_normals);
        std::cout<<"target instance point num:"<<cloud_normals->points.size()<<std::endl;
        // pcl::PointCloud<pcl::Normal> target_top_surface;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target_top_surface(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        target_top_surface->header.frame_id = "/base_link";
        for(int i = 0; i < cloud_normals->points.size(); i ++){
            // std::cout<<"pt: x: "<<normal_input_cloud->points[i].x<<" ,y:"<<normal_input_cloud->points[i].y<<" , z:"<<normal_input_cloud->points[i].z<<std::endl;
            if(cloud_normals->points[i].normal[2] > 0.7 || cloud_normals->points[i].normal[2] < -0.7){
                pcl::PointXYZRGBNormal top_pt;
                top_pt.x = normal_input_cloud->points[i].x;
                top_pt.y = normal_input_cloud->points[i].y;
                top_pt.z = normal_input_cloud->points[i].z;
                top_pt.r = normal_input_cloud->points[i].r;
                top_pt.g = normal_input_cloud->points[i].g;
                top_pt.b = normal_input_cloud->points[i].b;

                target_top_surface->points.push_back(top_pt);
            }
        }        
        std::cout<<"target top surface point num:"<<target_top_surface->points.size()<<std::endl;
        // publish_pointcloud(nh_, "target_cloud", target_top_surface);

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered_top_surface(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
        sor.setInputCloud(target_top_surface);
        sor.setMeanK (60);
        sor.setStddevMulThresh (0.9);
        sor.filter (*filtered_top_surface);
        // publish_pointcloud(nh_, "target_cloud", *filtered_top_surface);


        double surface_center_x = 0;
        double surface_center_y = 0;
        double surface_center_z = 0;

        // eigen::MatrixXf top_surface_

        break;




    }



    //step 1.5 check whether current instance is spatially separated, if yes, continue with next instance (current instance is blocked)

    //step 2, get its top surface through normal

    //step 3, check the whether the top-expanded space is in collision with other instance, if yes, continue with next instance

    //step 4, if no from step 3, then return current instance's top surface and its normal direction
}


void target_selector::set_rgb_topic(std::string name){
    rgb_topic_ = name;
}

void target_selector::set_depth_topic(std::string name){
    depth_topic_ = name;
}

void target_selector::set_camera_info_topic(std::string name){
    camera_info_topic_ = name;
}

void target_selector::set_camera_frame(std::string name){
    camera_frame_ = name;
}

void target_selector::set_base_frame(std::string name){
    base_frame_ = name;
}

void target_selector::set_image_segment_topic(std::string name){
    image_segment_topic_ = name;
}

void target_selector::set_segment_mask_topic(std::string name){
    segment_mask_topic_ = name;
}