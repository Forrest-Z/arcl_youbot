#ifndef ARCL_YOUBOT_VISION_TARGET_SELECTOR_HPP
#define ARCL_YOUBOT_VISION_TARGET_SELECTOR_HPP



#include <ros/ros.h>
#include <arcl_youbot_msgs/AllMask.h>

class target_selector{
    private:
        ros::NodeHandle& nh_;
        std::vector<std::vector<std::pair<int, int>>> mask_list_;
        std::string rgb_topic_;
        std::string depth_topic_;
        std::string camera_info_topic_;
        std::string camera_frame_;
        std::string base_frame_;
        std::string image_segment_topic_;
        std::string segment_mask_topic_;


    public:
        target_selector(ros::NodeHandle& nh);
        void image_segment_callback(const AllMask& msg);
        void set_rgb_topic(std::string);
        void set_depth_topic(std::string);
        void set_camera_info_topic(std::string);
        void set_camera_frame(std::string);
        void set_base_frame(std::string);
}


#endif