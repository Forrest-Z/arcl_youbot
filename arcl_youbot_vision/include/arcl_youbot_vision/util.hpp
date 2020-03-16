#ifndef ARCL_YOUBOT_VISION_UTIL_HPP
#define ARCL_YOUBOT_VISION_UTIL_HPP

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <cv_bridge/cv_bridge.h>


using namespace pcl;

typedef struct camerainfo_t
{
    std::string name;
    std::string topic_image;
    std::string topic_depth;
    std::string topic_camerainfo;
    float depth_scale;
    std::vector<float> camera_K;
    std::vector<float> camera_RT;
    Eigen::Matrix4f tf_cam2world;
    std::vector<float> workspace;
    std::string param_segmentation;    
} camerainfo_t;

bool GetInputFromCamera( ros::NodeHandle &nh,
                         sensor_msgs::Image &msg_image,
                         sensor_msgs::Image &msg_depth,
                         PointCloud<PointXYZRGB> &cloud_cam,
                         camerainfo_t &caminfo );   

bool GetInputFromCamera( ros::NodeHandle &nh,
                         PointCloud<PointXYZRGB> &cloud_cam,
                         const camerainfo_t &caminfo);

template<typename PointT, typename TYPE_DEPTH>
void PointCloudfromDepth( pcl::PointCloud<PointT> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &mask,
                          const bool isOrganized=false    );

template<typename PointT, typename TYPE_DEPTH>
void PointCloudfromDepth( pcl::PointCloud<PointT> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,                          
                          const cv::Mat &image,
                          const cv::Mat &mask,
                          const bool isOrganized=false    );

template<typename PointT, typename TYPE_DEPTH>
void PointCloudsfromDepth( std::map<uint8_t,typename pcl::PointCloud<PointT>::Ptr> &clouds,
                           const cv::Mat &seg,
                           const cv::Mat &depth,
                           const float depth_scale,
                           const std::vector<float> &cam_in,
                           const std::vector<float> &cam_ex,                          
                           const cv::Mat &image,
                           const cv::Mat &mask              );


void publish_pointcloud(ros::NodeHandle& nh, std::string topic_name, pcl::PointCloud<pcl::PointXYZRGB>& pointcloud);
void publish_pointcloud(ros::NodeHandle& nh ,std::string topic_name, pcl::PointCloud<pcl::PointXYZRGBNormal>& pointcloud);




#endif