#include "arcl_youbot_vision/util.hpp"

using namespace pcl;
using namespace std;



template<typename PointT>
static void ClearZeroZ( PointCloud<PointT> &cloud )
{
    PointCloud<PointT> cloud_tmp;
    for( int p=0; p<cloud.size(); p++ )
    {
        PointT &pt = cloud[p];
        if( pt.z == 0 ) continue;

        cloud_tmp.push_back(pt);
    }

    cloud.clear();
    copyPointCloud(cloud_tmp, cloud);
}


bool GetInputFromCamera( ros::NodeHandle &nh,
                         sensor_msgs::Image &msg_image,
                         sensor_msgs::Image &msg_depth,
                         PointCloud<PointXYZRGB> &cloud_cam,
                         camerainfo_t &caminfo         )
{
    sensor_msgs::Image::ConstPtr img_ptr, dep_ptr;
    sensor_msgs::CameraInfo::ConstPtr ci_depth;
    try
    {        
        img_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(caminfo.topic_image, nh);
        dep_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(caminfo.topic_depth, nh);
        ci_depth = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(caminfo.topic_camerainfo,nh);
    }
    catch(std::exception &e)
    {
        ROS_ERROR("Exception during waitForMessage from camera: %s", e.what());
        return false;
    }

    msg_image = *img_ptr;
    msg_depth = *dep_ptr;
    std::cout<<"get rgb points num:"<<msg_image.data.size()<<std::endl;
    caminfo.camera_K.resize(9);
    for( int k=0; k<9; k++ ) caminfo.camera_K[k] = ci_depth->K[k];
    caminfo.depth_scale = ci_depth->K[8];
    cv_bridge::CvImagePtr cv_img
     = cv_bridge::toCvCopy(*img_ptr, img_ptr->encoding);    

    cv_bridge::CvImagePtr cv_dep
     = cv_bridge::toCvCopy(*dep_ptr, dep_ptr->encoding);

    int bitdepth = sensor_msgs::image_encodings::bitDepth(dep_ptr->encoding);
    std::cout<<"bitdepth:"<<bitdepth<<std::endl;
    if( bitdepth==16 || bitdepth==64)
    {
        PointCloudfromDepth<PointXYZRGB, uint16_t>(
            cloud_cam, cv_dep->image, caminfo.depth_scale,
            caminfo.camera_K, vector<float>(), cv_img->image, cv::Mat() );
    }    
    else
    {        
        return false;
    }

    return true;
}


bool GetInputFromCamera( ros::NodeHandle &nh,
                         PointCloud<PointXYZRGB> &cloud_cam,
                         const camerainfo_t &caminfo         )
{
    sensor_msgs::Image::ConstPtr img_ptr, dep_ptr;
    sensor_msgs::CameraInfo::ConstPtr ci_depth;
    try
    {        
        img_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(caminfo.topic_image, nh);
        dep_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(caminfo.topic_depth, nh);
        ci_depth
         = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(caminfo.topic_camerainfo,nh);
    }
    catch(std::exception &e)
    {
        ROS_ERROR("Exception during waitForMessage from camera: %s", e.what());
        return false;
    }

    cv_bridge::CvImagePtr cv_img
     = cv_bridge::toCvCopy(*img_ptr, img_ptr->encoding);    

    cv_bridge::CvImagePtr cv_dep
     = cv_bridge::toCvCopy(*dep_ptr, dep_ptr->encoding);

    vector<float> camera_K(9);
    for( int k=0; k<9; k++ ) camera_K[k] = ci_depth->K[k];

    int bitdepth = sensor_msgs::image_encodings::bitDepth(dep_ptr->encoding);
    if( bitdepth==16 )
    {
        PointCloudfromDepth<PointXYZRGB, uint16_t>(
            cloud_cam, cv_dep->image, caminfo.depth_scale,
            camera_K, vector<float>(), cv_img->image, cv::Mat() );
    }    
    else
    {        
        return false;
    }

    return true;
}


template<typename PointT, typename TYPE_DEPTH>
void PointCloudfromDepth( PointCloud<PointT> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &mask,
                          const bool isOrganized            )
{
    int rows = depth.rows;
    int cols = depth.cols;
    cloud.width  = cols;
    cloud.height = rows;
    cloud.points.resize(cols*rows);

    for( int r=0; r<rows; r++ )
    for( int c=0; c<cols; c++ )
    {
        PointT &pt = cloud(c,r);

        // mask out
        if( mask.rows > 0 && mask.cols > 0 && mask.at<uint8_t>(r,c) == 0 )
        {
            pt.x = 0; pt.y = 0; pt.z = 0;
            continue;
        } 

        // 2D -> 3D (camera coordinate)
        float z = (float)depth.at<TYPE_DEPTH>(r,c) * depth_scale;
        if( z < 0.10 )
        {
            pt.x = 0; pt.y = 0; pt.z = 0;            
        }
        float x = (((float)c) - cam_in[2]) / cam_in[0] * z;
        float y = (((float)r) - cam_in[5]) / cam_in[4] * z;

        pt.x = x; pt.y = y; pt.z = z;
    }

    if( !isOrganized )    
    {        
        ClearZeroZ(cloud);
        cloud.width = cloud.size();
        cloud.height = 1;
    }

    // camera -> world coordinate    
    if( cam_ex.size() == 16 )
    {
        Eigen::Matrix4f tf;
        tf << cam_ex[0],  cam_ex[1],  cam_ex[2],  cam_ex[3],
              cam_ex[4],  cam_ex[5],  cam_ex[6],  cam_ex[7],
              cam_ex[8],  cam_ex[9],  cam_ex[10], cam_ex[11],
              cam_ex[12], cam_ex[13], cam_ex[14], cam_ex[15];
        transformPointCloud(cloud, cloud, tf);
    }
}

template<typename PointT, typename TYPE_DEPTH>
void PointCloudfromDepth( PointCloud<PointT> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,                          
                          const cv::Mat &image,
                          const cv::Mat &mask,
                          const bool isOrganized            )
{
    vector<float> cam_ex_id(16);
    cam_ex_id[0] =1; cam_ex_id[1] =0; cam_ex_id[2] =0; cam_ex_id[3] =0; 
    cam_ex_id[4] =0; cam_ex_id[5] =1; cam_ex_id[6] =0; cam_ex_id[7] =0; 
    cam_ex_id[8] =0; cam_ex_id[9] =0; cam_ex_id[10]=1; cam_ex_id[11]=0; 
    cam_ex_id[12]=0; cam_ex_id[13]=0; cam_ex_id[14]=0; cam_ex_id[15]=1;

    PointCloudfromDepth<PointT,TYPE_DEPTH>(
        cloud, depth, depth_scale, cam_in, cam_ex_id, mask, true);

    int rows = depth.rows;
    int cols = depth.cols;
    for( int r=0; r<rows; r++ )
    for( int c=0; c<cols; c++ )
    {
        PointT &pt = cloud(c,r);

        pt.r = image.at<cv::Vec3b>(r,c)[2];
        pt.g = image.at<cv::Vec3b>(r,c)[1];
        pt.b = image.at<cv::Vec3b>(r,c)[0];
    }

    if( !isOrganized )    
    {        
        ClearZeroZ(cloud);
        cloud.width = cloud.size();
        cloud.height = 1;
    }

    // camera -> world coordinate
    if( cam_ex.size() == 16 )
    {
        Eigen::Matrix4f tf;
        tf << cam_ex[0],  cam_ex[1],  cam_ex[2],  cam_ex[3],
              cam_ex[4],  cam_ex[5],  cam_ex[6],  cam_ex[7],
              cam_ex[8],  cam_ex[9],  cam_ex[10], cam_ex[11],
              cam_ex[12], cam_ex[13], cam_ex[14], cam_ex[15];
        transformPointCloud(cloud, cloud, tf);
    }    
}

template<typename PointT, typename TYPE_DEPTH>
void PointCloudsfromDepth( map<uint8_t,typename PointCloud<PointT>::Ptr> &clouds,
                           const cv::Mat &seg,
                           const cv::Mat &depth,
                           const float depth_scale,
                           const std::vector<float> &cam_in,
                           const std::vector<float> &cam_ex,                          
                           const cv::Mat &image,
                           const cv::Mat &mask              )
{
    vector<float> cam_ex_id(16);
    cam_ex_id[0] =1; cam_ex_id[1] =0; cam_ex_id[2] =0; cam_ex_id[3] =0; 
    cam_ex_id[4] =0; cam_ex_id[5] =1; cam_ex_id[6] =0; cam_ex_id[7] =0; 
    cam_ex_id[8] =0; cam_ex_id[9] =0; cam_ex_id[10]=1; cam_ex_id[11]=0; 
    cam_ex_id[12]=0; cam_ex_id[13]=0; cam_ex_id[14]=0; cam_ex_id[15]=1;

    clouds.clear();

    PointCloud<PointT> cloud;
    PointCloudfromDepth<PointT,TYPE_DEPTH>(
        cloud, depth, depth_scale, cam_in, cam_ex_id, mask, true);

    int rows = depth.rows;
    int cols = depth.cols;
    for( int r=0; r<rows; r++ )
    for( int c=0; c<cols; c++ )
    {
        PointT &pt = cloud(c,r);

        pt.b = image.at<cv::Vec3b>(r,c)[0];
        pt.g = image.at<cv::Vec3b>(r,c)[1];
        pt.r = image.at<cv::Vec3b>(r,c)[2];

        uint8_t label = seg.at<uint8_t>(r,c);

        typename PointCloud<PointT>::Ptr cloud_seg;
        typename map< uint8_t,
                      typename PointCloud<PointT>::Ptr >::iterator it_cloud
         = clouds.find(label);
        if( it_cloud == clouds.end() )
        {
            cloud_seg
             = (typename PointCloud<PointT>::Ptr)(new PointCloud<PointT>);
            clouds.insert(
              pair<uint8_t,typename PointCloud<PointT>::Ptr>(label, cloud_seg));
        }
        else
        {
            cloud_seg = it_cloud->second;
        }
        cloud_seg->push_back(pt);
    }

    Eigen::Matrix4f tf;
    if( cam_ex.size()==0 )
    {
        tf = Eigen::Matrix4f::Identity();
    }
    else
    {
        tf << cam_ex[0],  cam_ex[1],  cam_ex[2],  cam_ex[3],
              cam_ex[4],  cam_ex[5],  cam_ex[6],  cam_ex[7],
              cam_ex[8],  cam_ex[9],  cam_ex[10], cam_ex[11],
              cam_ex[12], cam_ex[13], cam_ex[14], cam_ex[15];
    }

    for( typename map<uint8_t,typename PointCloud<PointT>::Ptr>::iterator 
         it=clouds.begin(); it!=clouds.end(); it++ )
    {        
        ClearZeroZ(*it->second);

        // camera -> world coordinate    
        transformPointCloud(*it->second, *it->second, tf);
    }    
}

template
void PointCloudfromDepth<PointXYZ, uint16_t>( 
                          PointCloud<PointXYZ> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &mask,
                          const bool isOrganized               );

template 
void PointCloudfromDepth<PointXYZRGB, uint16_t>( 
                          PointCloud<PointXYZRGB> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &mask,
                          const bool isOrganized               );

template 
void PointCloudfromDepth<PointXYZRGBNormal, uint16_t>( 
                          PointCloud<PointXYZRGBNormal> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &mask,
                          const bool isOrganized               );

template 
void PointCloudfromDepth<PointXYZRGB, uint16_t>( 
                          PointCloud<PointXYZRGB> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &image,
                          const cv::Mat &mask,
                          const bool isOrganized              );

template 
void PointCloudfromDepth<PointXYZRGBNormal, uint16_t>( 
                          PointCloud<PointXYZRGBNormal> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &image,
                          const cv::Mat &mask,
                          const bool isOrganized              );

template
void PointCloudsfromDepth<PointXYZRGB, uint16_t>( 
                           map<uint8_t,PointCloud<PointXYZRGB>::Ptr> &clouds,
                           const cv::Mat &seg,
                           const cv::Mat &depth,
                           const float depth_scale,
                           const std::vector<float> &cam_in,
                           const std::vector<float> &cam_ex,                          
                           const cv::Mat &image,
                           const cv::Mat &mask              );

void publish_pointcloud(ros::NodeHandle& nh ,std::string topic_name, pcl::PointCloud<pcl::PointXYZRGB>& pointcloud){
    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> (topic_name, 1);

    std::cout<<"publish pointcloud in base link, point num:"<<pointcloud.points.size()<<std::endl;
    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        pub.publish(pointcloud);
        ros::spinOnce ();
        loop_rate.sleep ();
    }

}

void publish_pointcloud(ros::NodeHandle& nh ,std::string topic_name, pcl::PointCloud<pcl::PointXYZRGBNormal>& pointcloud){
    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> (topic_name, 1);

    std::cout<<"publish pointcloud in base link"<<std::endl;
    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        pub.publish(pointcloud);
        ros::spinOnce ();
        loop_rate.sleep ();
    }

}

std::tuple<double, double, double> get_pointcloud_center(pcl::PointCloud<pcl::PointXYZRGBNormal>& pc){
    double center_x = 0;
    double center_y = 0;
    double center_z = 0;
    for(int i = 0; i < pc.points.size(); i++){
        center_x += pc.points[i].x;
        center_y += pc.points[i].y;
        center_z += pc.points[i].z;

    }
    center_x = center_x / pc.points.size();
    center_y = center_y / pc.points.size();
    center_z = center_z / pc.points.size();
    return std::make_tuple(center_x, center_y, center_z);
}
