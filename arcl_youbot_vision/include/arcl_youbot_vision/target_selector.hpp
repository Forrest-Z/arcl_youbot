#ifndef ARCL_YOUBOT_VISION_TARGET_SELECTOR_HPP
#define ARCL_YOUBOT_VISION_TARGET_SELECTOR_HPP



#include <ros/ros.h>
#include <arcl_youbot_msgs/AllMask.h>
#include <pcl/features/normal_3d.h>

class target_selector{
    private:
        ros::NodeHandle nh_;
        std::vector<std::vector<std::pair<int, int>>> mask_list_;
        std::string rgb_topic_;
        std::string depth_topic_;
        std::string camera_info_topic_;
        std::string camera_frame_;
        std::string base_frame_;
        std::string image_segment_topic_;
        std::string segment_mask_topic_;
        sensor_msgs::Image curr_image_;
        sensor_msgs::Image curr_depth_;
        camerainfo_t caminfo_;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_camera_;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_base_;

    public:
        target_selector(ros::NodeHandle& nh);
        void image_segment_callback(const arcl_youbot_msgs::AllMask& msg);
        void set_rgb_topic(std::string);
        void set_depth_topic(std::string);
        void set_camera_info_topic(std::string);
        void set_camera_frame(std::string);
        void set_base_frame(std::string);
        void set_image_segment_topic(std::string);
        void set_segment_mask_topic(std::string);
        bool is_mask_separated(std::vector<std::pair<int, int>>, int);
        void get_next_target_instance();
        void get_current_scene();
        void segment_mask_callback(const arcl_youbot_msgs::AllMask& msg);
        void get_instance_mask();

};


#endif