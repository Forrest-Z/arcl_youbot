#include "ros/ros.h"
#include <ros/package.h>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <placeplan_service/PlacePlanAction.h>
#include <placeplan_service/PlacePlan.h>
#include <pluginlib/class_loader.h>
#include "opencv2/opencv.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#define COLLISION_THRESHOLD 50
// #define
//#define USE_ACTIONLIB
#define USE_SERVICE
#define X_INFLATION 0.05
#define Y_INFLATION 0.05
#define Z_DEFLATION 0.0
#define Z_RAISE 0.01

class PlacePlanBaselineAction
{
public:
    PlacePlanBaselineAction(std::string name) : as_(nh_, name, false),
                                                action_name_(name)
    {
        ROS_INFO_STREAM("start place plan server!");
//register the goal and feeback callbacks
#ifdef USE_ACTIONLIB
        as_.registerGoalCallback(boost::bind(&PlacePlanBaselineAction::goalCB, this));
        as_.start();
#endif

        cloud_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("graspin_point_cloud", 1);
        filter_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("filtered_point_cloud", 1);
        cube_pub = nh_.advertise<visualization_msgs::Marker>("target_region", 1);
        arrow_pub = nh_.advertise<visualization_msgs::Marker>("retract_arrow", 1);
        retract_cube_pub = nh_.advertise<visualization_msgs::Marker>("retract_cube", 1);

        std::string resources_path = ros::package::getPath("chimp_resources");
        system(("rosparam load " + resources_path + "/config/obj_config.yaml").c_str());
    }

    ~PlacePlanBaselineAction(void)
    {
    }
#ifdef USE_SERVICE
    bool goalCB(placeplan_service::PlacePlan::Request &req,
                placeplan_service::PlacePlan::Response &res);
#endif
#ifdef USE_ACTIONLIB
    void goalCB();
#endif
    void initWorldPointCloud();
    void initWorldPointCloudFromFile(std::string);
    void convert3dUnOrganized(cv::Mat &objDepth, Eigen::Matrix3f &camIntrinsic, pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud);
    void toTransformationMatrix(Eigen::Matrix4f &camPose, std::vector<double> camPose7D);
    void readDepthImage(cv::Mat &depthImg, std::string path);
    void InverseTransformationMatrix(Eigen::Matrix4f &origin, Eigen::Matrix4f &inverse);
    Eigen::Matrix4f transformPclFromWorldToTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcl, geometry_msgs::Pose target_pose);
    bool checkRegionCollision(geometry_msgs::Pose center, double x_length, double y_length, double z_length);
    geometry_msgs::Pose getRetractPose(geometry_msgs::Pose target_pose, double x_length, double y_length, double z_length);
    void publish_cube(geometry_msgs::Pose pose, double x, double y, double z, ros::Publisher pub, double r = 0, double g = 1, double b = 0);
    void publish_arrow(geometry_msgs::Point start, geometry_msgs::Point end);

protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<placeplan_service::PlacePlanAction> as_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr objCloudWorld;
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetObjCloud;
    ros::ServiceClient clt_perception;
    std::string action_name_;
    int data_count_;
    float sum_, sum_sq_;
    placeplan_service::PlacePlanFeedback feedback_;
    placeplan_service::PlacePlanResult result_;
    placeplan_service::PlacePlanGoal goal_;
    std::string target_object_name_;
    std::string last_target_object_name_;
    geometry_msgs::Pose target_object_pose_;
    int last_target_object_index_;
    ros::Publisher marker_pub;
    ros::Publisher cube_pub;
    ros::Publisher cloud_pub;
    ros::Publisher filter_pub;
    ros::Publisher arrow_pub;
    ros::Publisher retract_cube_pub;
};

void PlacePlanBaselineAction::convert3dUnOrganized(cv::Mat &objDepth, Eigen::Matrix3f &camIntrinsic, pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud)
{
    int imgWidth = objDepth.cols;
    int imgHeight = objDepth.rows;
    ROS_WARN_STREAM("imgWidth:" << imgWidth);
    ROS_WARN_STREAM("imgHeight:" << imgHeight);

    for (int u = 0; u < imgHeight; u++)
        for (int v = 0; v < imgWidth; v++)
        {
            float depth = objDepth.at<float>(u, v);
            // ROS_WARN_STREAM("depth:"<<depth);
            if (depth > 0.1 && depth < 2.0)
            {
                pcl::PointXYZ pt;
                pt.x = (float)((v - camIntrinsic(0, 2)) * depth / camIntrinsic(0, 0));
                pt.y = (float)((u - camIntrinsic(1, 2)) * depth / camIntrinsic(1, 1));
                pt.z = depth;
                objCloud->points.push_back(pt);
            }
        }
}

void PlacePlanBaselineAction::readDepthImage(cv::Mat &depthImg, std::string path)
{
    float depth_scale = 1.25; // constant scale factor for realsense sr300
    cv::Mat depthImgRaw = cv::imread(path, CV_16UC1);
    depthImg = cv::Mat::zeros(depthImgRaw.rows, depthImgRaw.cols, CV_32FC1);

    for (int u = 0; u < depthImgRaw.rows; u++)
        for (int v = 0; v < depthImgRaw.cols; v++)
        {
            unsigned short depthShort = depthImgRaw.at<unsigned short>(u, v);

            float depth = (float)depthShort * depth_scale / 10000;
            depthImg.at<float>(u, v) = depth;
        }
}

void PlacePlanBaselineAction::toTransformationMatrix(Eigen::Matrix4f &camPose, std::vector<double> camPose7D)
{
    camPose(0, 3) = camPose7D[0];
    camPose(1, 3) = camPose7D[1];
    camPose(2, 3) = camPose7D[2];
    camPose(3, 3) = 1;

    Eigen::Quaternionf q;
    q.w() = camPose7D[3];
    q.x() = camPose7D[4];
    q.y() = camPose7D[5];
    q.z() = camPose7D[6];
    Eigen::Matrix3f rotMat;
    rotMat = q.toRotationMatrix();

    for (int ii = 0; ii < 3; ii++)
        for (int jj = 0; jj < 3; jj++)
        {
            camPose(ii, jj) = rotMat(ii, jj);
        }
}

void PlacePlanBaselineAction::InverseTransformationMatrix(Eigen::Matrix4f &origin, Eigen::Matrix4f &inverse)
{
    // inverse(0,3) = - origin(0,3);
    // inverse(1,3) =  origin(1,3);
    // inverse(2,3) =  origin(2,3);
    // inverse(3,3) = 1;
    Eigen::Matrix3f rotMat = origin.block(0, 0, 3, 3);
    Eigen::Matrix3f inverse_rotMat = rotMat.inverse();
    for (int ii = 0; ii < 3; ii++)
        for (int jj = 0; jj < 3; jj++)
        {
            inverse(ii, jj) = inverse_rotMat(ii, jj);
        }
    Eigen::Vector3f temp(-origin(0, 3), -origin(1, 3), -origin(2, 3));
    Eigen::Vector3f final = inverse_rotMat * temp;
    inverse(0, 3) = final(0);
    inverse(1, 3) = final(1);
    inverse(2, 3) = final(2);

    //std::cout<<final(0)<<","<<final(1)<<","<<final(2)<<std::endl;
}

Eigen::Matrix4f PlacePlanBaselineAction::transformPclFromWorldToTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcl, geometry_msgs::Pose target_pose)
{
    double target_x = target_pose.position.x;
    double target_y = target_pose.position.y;
    double target_z = target_pose.position.z;
    Eigen::Matrix4f target_matrix = Eigen::Matrix4f::Zero(4, 4);
    Eigen::Matrix4f world_to_target_pose = Eigen::Matrix4f::Zero(4, 4);
    std::vector<double> target_pose7d = {target_x, target_y, target_z, target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z};
    toTransformationMatrix(target_matrix, target_pose7d);
    //std::cout << "target matrix: " << std::endl << target_matrix << std::endl;
    InverseTransformationMatrix(target_matrix, world_to_target_pose);
    target_pcl->header.frame_id = "world";
    //std::cout << "World to target pose: " << std::endl << world_to_target_pose << std::endl;
    pcl::transformPointCloud(*objCloudWorld, *target_pcl, world_to_target_pose);
    return target_matrix;
}
bool PlacePlanBaselineAction::checkRegionCollision(geometry_msgs::Pose center, double x_length, double y_length, double z_length)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr checkCloud(new pcl::PointCloud<pcl::PointXYZ>);
    transformPclFromWorldToTarget(checkCloud, center);
    double x = center.position.x;
    double y = center.position.y;
    double z = center.position.z;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(checkCloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-x_length / 2.0, x_length / 2.0);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-y_length / 2.0, +y_length / 2.0);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-z_length / 2.0, +z_length / 2.0);
    pass.filter(*cloud_filtered);

    std::cout << "point cloud size: " << cloud_filtered->size() << std::endl;
    if (cloud_filtered->size() > COLLISION_THRESHOLD)
    {
        return true;
    }
    else
    {
        return false;
    }
}

geometry_msgs::Pose PlacePlanBaselineAction::getRetractPose(geometry_msgs::Pose target_pose, double x_length, double y_length, double z_length)
{
    int max_step = 25;
    int step_index = 0;
    bool path_in_collision = false;
    double origin_x_length = x_length - X_INFLATION;
    double origin_y_length = y_length - Y_INFLATION;
    geometry_msgs::Pose retract_pose = target_pose;
    geometry_msgs::Pose check_pose = target_pose;
    double combined_vector_x, combined_vector_y, combined_vector_length;
    while (step_index < max_step)
    {
        if (step_index >= max_step - 4)
        {
            x_length -= 0.01;
            y_length -= 0.01;
        }
        if (step_index == 0)
        {
            check_pose = retract_pose;
            Eigen::Matrix4f target_matrix = transformPclFromWorldToTarget(targetObjCloud, check_pose);
            double center_x = check_pose.position.x;
            double center_y = check_pose.position.y;
            double center_z = check_pose.position.z;
            objCloudWorld->header.frame_id = "world";
            cloud_pub.publish(objCloudWorld);
            ros::spinOnce();

            ROS_WARN_STREAM("target cloud has " << targetObjCloud->points.size() << " points");
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_world(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PassThrough<pcl::PointXYZ> pass;
            ROS_WARN_STREAM("x_length:" << x_length);
            ROS_WARN_STREAM("y_length:" << y_length);
            ROS_WARN_STREAM("z_length:" << z_length);

            pass.setInputCloud(targetObjCloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(-x_length / 2.0, x_length / 2.0);
            pass.filter(*cloud_filtered);
            ROS_WARN_STREAM("filtered cloud has " << cloud_filtered->points.size() << " points");
            pass.setInputCloud(cloud_filtered);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(-y_length / 2.0, y_length / 2.0);
            pass.filter(*cloud_filtered);
            ROS_WARN_STREAM("filtered cloud has " << cloud_filtered->points.size() << " points");

            pass.setInputCloud(cloud_filtered);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(-z_length / 2.0, z_length / 2.0);
            pass.filter(*cloud_filtered);
            ROS_WARN_STREAM("filtered cloud has " << cloud_filtered->points.size() << " points");
            // pcl::visualization::PCLVisualizer viewer("debug");
            //  viewer.setPosition(0,0);
            // viewer.setSize(600,480);
            // viewer.setCameraPosition(0,0,-1,0,0,1,0,-1,0);
            // viewer.addPointCloud(cloud_filtered);

            pcl::transformPointCloud(*cloud_filtered, *cloud_filtered_world, target_matrix);

            filter_pub.publish(cloud_filtered_world);

            double point_x, point_y, point_z;
            std::cout << "total num:" << cloud_filtered_world->points.size() << std::endl;

            combined_vector_x = 0;
            if (!checkRegionCollision(retract_pose, x_length, y_length, z_length))
            {
                geometry_msgs::Pose start = target_pose;
                geometry_msgs::Pose end = retract_pose;
                geometry_msgs::Pose temp = start;
                int seg_num = 10;
                double seg_x = (end.position.x - start.position.x) / seg_num;
                double seg_y = (end.position.y - start.position.y) / seg_num;
                // for(int j = 1; j < seg_num; j++){
                //   temp.position.x += seg_x*j;
                //   temp.position.y += seg_y*j;
                //   if(checkRegionCollision(temp, origin_x_length , origin_y_length, z_length)){
                //     ROS_WARN_STREAM("path_in_collision == true, at "<<j);
                //     path_in_collision = true;
                //     break;
                //   }
                // }
                //if( sqrt( pow( end.position.x - start.position.x,2 ) + pow(end.position.y - start.position.y,2) ) > 0.1 )
                //    path_in_collision = true;

                break;
            }
            combined_vector_y = 0;
            for (int ii = 0; ii < cloud_filtered_world->points.size(); ii++)
            {
                //std::cout <<"||"<< objCloudGripper->points[ii].x << " " << objCloudGripper->points[ii].y << " " << objCloudGripper->points[ii].z;
                point_x = cloud_filtered_world->points[ii].x;
                point_y = cloud_filtered_world->points[ii].y;
                point_z = cloud_filtered_world->points[ii].z;
                double vector_x = center_x - point_x;
                double vector_y = center_y - point_y;
                double vector_length = sqrt(vector_x * vector_x + vector_y * vector_y);
                vector_x = vector_x / vector_length;
                vector_y = vector_y / vector_length;
                combined_vector_y += vector_y;
                combined_vector_x += vector_x;
            }
            combined_vector_length = sqrt(combined_vector_x * combined_vector_x + combined_vector_y * combined_vector_y);
            combined_vector_x = combined_vector_x / combined_vector_length;
            combined_vector_y = combined_vector_y / combined_vector_length;
            ROS_ERROR_STREAM("first step combined_vector:" << combined_vector_x << "," << combined_vector_y);
        }
        // retract_pose = check_pose;
        // ROS_ERROR_STREAM("first step combined_vector:"<<combined_vector_x<<","<<combined_vector_y);

        retract_pose.position.x += combined_vector_x * 0.01;
        retract_pose.position.y += combined_vector_y * 0.01;
        publish_cube(retract_pose, x_length, y_length, z_length, retract_cube_pub, 1, 0, 0);
        ROS_WARN_STREAM("retract_cube:" << retract_pose.position.x << "," << retract_pose.position.y);
        if (!checkRegionCollision(retract_pose, x_length, y_length, z_length))
        {
            geometry_msgs::Pose start = target_pose;
            geometry_msgs::Pose end = retract_pose;
            geometry_msgs::Pose temp = start;
            int seg_num = 10;
            double seg_x = (end.position.x - start.position.x) / seg_num;
            double seg_y = (end.position.y - start.position.y) / seg_num;
            // for(int j = 1; j < seg_num; j++){
            //   temp.position.x += seg_x*j;
            //   temp.position.y += seg_y*j;
            //   if(checkRegionCollision(temp, origin_x_length , origin_y_length, z_length)){
            //     ROS_WARN_STREAM("path_in_collision == true, at "<<j);
            //     path_in_collision = true;
            //     break;
            //   }
            // }
            //if( sqrt( pow( end.position.x - start.position.x,2 ) + pow(end.position.y - start.position.y,2) ) > 0.1 )
            //    path_in_collision = true;

            break;
        }
        step_index++;
        if (step_index == max_step)
        {
            if (checkRegionCollision(retract_pose, origin_x_length, origin_y_length, z_length))
            {

                // retract_pose = target_pose;
                // retract_pose.position.z = -1000;
                // ROS_ERROR_STREAM("step_index == max_step and in collision");
                step_index--;
                x_length += 0.02;
                y_length += 0.02;
            }
        }
        if (x_length - origin_x_length > X_INFLATION)
        {
            retract_pose = target_pose;
            retract_pose.position.z = -1000;
            ROS_ERROR_STREAM("step_index == max_step and in collision");
            break;
        }
    }

    geometry_msgs::Point arrow_start, arrow_end;
    arrow_start.x = target_pose.position.x;
    arrow_start.y = target_pose.position.y;
    arrow_start.z = target_pose.position.z;
    arrow_end.x = retract_pose.position.x;
    arrow_end.y = retract_pose.position.y;
    arrow_end.z = retract_pose.position.z;

    publish_arrow(arrow_start, arrow_end);
    publish_cube(retract_pose, origin_x_length, origin_y_length, z_length, retract_cube_pub, 1, 0, 0);

    if (step_index == max_step)
    {
        if (checkRegionCollision(retract_pose, origin_x_length, origin_y_length, z_length))
        {
            retract_pose = target_pose;
            retract_pose.position.z = -1000;
            ROS_ERROR_STREAM("step_index == max_step and in collision");
        }
    }

    if (path_in_collision)
    {
        retract_pose = target_pose;
        retract_pose.position.z = -1000;
        ROS_ERROR_STREAM("RETRACT POSE INVALID, PATH IN COLLISION");
    }
    // viewer.setSize(640,480);
    // viewer.setPosition(0,0);
    // viewer.setCameraPosition(2,0,1,0.5,0,-0.2,0,0,1);
    // viewer.setBackgroundColor (0.2,0.2,0.2);
    // viewer.addCoordinateSystem(0.1);
    // viewer.addPointCloud(cloud_filtered, "cloud_filtered");

    return retract_pose;
    // geometry_msgs::Pose pre_push_pose =
}
#ifdef USE_ACTIONLIB
void PlacePlanBaselineAction::goalCB()
#endif
#ifdef USE_SERVICE
    bool PlacePlanBaselineAction::goalCB(placeplan_service::PlacePlan::Request &req,
                                         placeplan_service::PlacePlan::Response &res)
#endif
{
#ifdef USE_ACTIONLIB
    ROS_INFO("receive new goal");
    auto new_goal_ = as_.acceptNewGoal();
    target_object_pose_ = new_goal_->target_object_pose;
    target_object_name_ = new_goal_->target_object_name;
    ROS_WARN_STREAM("target obj pose:" << target_object_pose_);
    ros::spinOnce();
#endif
#ifdef USE_SERVICE
    ROS_INFO("receive new goal");
    target_object_pose_ = req.target_object_pose;
    target_object_name_ = req.target_object_name;
    ros::spinOnce();
#endif

    std::cout << "Call received!" << std::endl;
    std::cout << "Path: " << req.target_object_name << std::endl;
    std::cout << "Position: " << req.target_object_pose.position.x << ", " << req.target_object_pose.position.y << ", " << req.target_object_pose.position.z << ", " << std::endl;
    std::cout << "Dimensions: " << req.x_dim << ", " << req.y_dim << ", " << req.z_dim << ", " << std::endl;
    std::cout << "Call received!" << std::endl;

    target_object_pose_.position.z += Z_RAISE;
    initWorldPointCloudFromFile(target_object_name_);

    double x_length = req.x_dim;
    double y_length = req.y_dim;
    double z_length = req.z_dim;

    publish_cube(target_object_pose_, x_length + X_INFLATION, y_length + Y_INFLATION, z_length, cube_pub);
    // publish_cube(target_object_pose_, x_length, y_length , z_length, cube_pub);
    geometry_msgs::Pose pre_push_pose = getRetractPose(target_object_pose_, x_length + X_INFLATION, y_length + Y_INFLATION, z_length - Z_DEFLATION);

#ifdef USE_ACTIONLIB
    result_.pre_push_pose = pre_push_pose;
    result_.post_push_pose = target_object_pose_;
    as_.setSucceeded(result_);
    ros::spinOnce();
#endif
#ifdef USE_SERVICE
    res.pre_push_pose = pre_push_pose;
    res.post_push_pose = target_object_pose_;
    return true;

#endif
}

void PlacePlanBaselineAction::initWorldPointCloudFromFile(std::string pc_filename)
{

    objCloudWorld = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    targetObjCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pc_filename, *objCloudWorld) == -1) //* load the file
    {
        ROS_WARN_STREAM("Couldn't read file " << pc_filename);
    }
}

void PlacePlanBaselineAction::initWorldPointCloud()
{
    double detected_length = 0.15;
    char cam_intrinsic_topic[100];
    char cam_pose_topic[100];
    sensor_msgs::Image::ConstPtr msg_depth;
    std::string resources_path = ros::package::getPath("chimp_resources");
    system(("rosparam load " + resources_path + "/config/bin_config.yaml").c_str());
    char depth_image_topic_param[100];
    int binId = 1;
    XmlRpc::XmlRpcValue camIntr;
    std::vector<double> camPose7D;
    std::string depth_image_topic;
    sprintf(cam_pose_topic, "/bins/bin_%d/camera_pose", binId);
    nh_.getParam(cam_pose_topic, camPose7D);
    Eigen::Matrix4f camPose = Eigen::Matrix4f::Zero(4, 4);
    toTransformationMatrix(camPose, camPose7D);
    std::cout << "Camera Pose: " << std::endl
              << camPose << std::endl;

    // Reading camera intrinsic matrix
    sprintf(cam_intrinsic_topic, "/bins/bin_%d/camera_intrinsic", binId);
    Eigen::Matrix3f camIntrinsic = Eigen::Matrix3f::Zero(3, 3);
    nh_.getParam(cam_intrinsic_topic, camIntr);

    for (int32_t ii = 0; ii < camIntr.size(); ii++)
        for (int32_t jj = 0; jj < camIntr[ii].size(); jj++)
            camIntrinsic(ii, jj) = static_cast<double>(camIntr[ii][jj]);
    std::cout << "Camera Intrinsics: " << std::endl
              << camIntrinsic << std::endl;

    sprintf(depth_image_topic_param, "/bins/bin_%d/depth_image_topic", binId);
    nh_.getParam(depth_image_topic_param, depth_image_topic);

    std::cout << "Waiting for depth image on topic: " << depth_image_topic << std::endl;
    msg_depth = ros::topic::waitForMessage<sensor_msgs::Image>(depth_image_topic, nh_);
    cv_bridge::CvImagePtr cv_ptr_depth;
    cv::Mat depth_image;
    try
    {

        cv_ptr_depth = cv_bridge::toCvCopy(*msg_depth, (*msg_depth).encoding);
        //depth_image = cv_ptr_depth->image;
        cv::imwrite("/home/cm1074/Desktop/frame-000000.depth.png", cv_ptr_depth->image);
        readDepthImage(depth_image, "/home/cm1074/Desktop/frame-000000.depth.png");
        // cv_ptr_depth = cv_bridge::toCvCopy(*msg_depth, (*msg_depth).encoding);
        //depth_image = cv_ptr_depth->image;
        //    cv::imwrite("/home/cm1074/Desktop/frame-000000.depth.png", cv_ptr_depth->image);
        // readDepthImage(depth_image, "/home/cm1074/Desktop/frame-000000.depth.png");
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        exit(-1);
    }

    std::cout << "got depth image" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud(new pcl::PointCloud<pcl::PointXYZ>);
    objCloudWorld = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    targetObjCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    convert3dUnOrganized(depth_image, camIntrinsic, objCloud);
    pcl::transformPointCloud(*objCloud, *objCloudWorld, camPose);

    ROS_WARN_STREAM("worldCloud has " << objCloud->points.size() << " points");
    objCloudWorld->header.frame_id = "world";
    cloud_pub.publish(objCloudWorld);

    //   pcl::visualization::PCLVisualizer viewer("world");
    //   viewer.setSize(640,480);
    //   viewer.setPosition(0,0);
    //   viewer.setBackgroundColor (0.2,0.2,0.2);
    //   viewer.addCoordinateSystem(0.1);
    //   viewer.setCameraPosition(2,0,1,0.5,0,-0.2,0,0,1);
    //   viewer.addPointCloud(objCloudWorld);
    // viewer.spinOnce(3);
    ros::spinOnce();
}

void PlacePlanBaselineAction::publish_cube(geometry_msgs::Pose pose, double x, double y, double z, ros::Publisher pub, double r, double g, double b)
{
    std::cout << "Publishing marker..." << std::endl;

    visualization_msgs::Marker cube;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    cube.header.frame_id = "/world";
    cube.header.stamp = ros::Time::now();

    // Set the namespace and id for this cube.  This serves to create a unique ID
    // Any cube sent with the same namespace and id will overwrite the old one
    cube.ns = "basic_cube";
    //cube.id = std::rand();
    cube.id = 9999;
    // Set the cube type
    cube.type = visualization_msgs::Marker::CUBE;
    cube.action = visualization_msgs::Marker::MODIFY;

    cube.pose = pose;
    cube.color.r = r;
    cube.color.g = g;
    cube.color.b = b;
    cube.color.a = 0.5;

    cube.scale.x = x;
    cube.scale.y = y;
    cube.scale.z = z;

    cube.lifetime = ros::Duration(140);
    pub.publish(cube);

    ros::spinOnce();
}

void PlacePlanBaselineAction::publish_arrow(geometry_msgs::Point start, geometry_msgs::Point end)
{
    std::cout << "Publishing marker..." << std::endl;

    ROS_WARN_STREAM("start:" << start.x << "," << start.y);
    ROS_WARN_STREAM("end:" << end.x << "," << end.y);

    visualization_msgs::Marker cube;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    cube.header.frame_id = "/world";
    cube.header.stamp = ros::Time::now();

    // Set the namespace and id for this cube.  This serves to create a unique ID
    // Any cube sent with the same namespace and id will overwrite the old one
    cube.ns = "basic_cube";
    cube.id = 912398;

    // Set the cube type
    cube.type = visualization_msgs::Marker::ARROW;
    cube.action = visualization_msgs::Marker::ADD;

    cube.color.r = 0.0f;
    cube.color.g = 0.0f;
    cube.color.b = 1.0f;
    cube.color.a = 1;

    cube.scale.x = 0.002;
    cube.scale.y = 0.003;
    cube.scale.z = 0.004;
    cube.points.push_back(start);
    cube.points.push_back(end);

    cube.lifetime = ros::Duration(50);
    arrow_pub.publish(cube);

    ros::spinOnce();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "placeplan_action_baseline");
    ROS_WARN_STREAM("starting");
    ros::NodeHandle nh;
    PlacePlanBaselineAction placeplan_action(ros::this_node::getName());
#ifdef USE_SERVICE
    ros::ServiceServer service = nh.advertiseService("/placeplan_service_baseline", &PlacePlanBaselineAction::goalCB, &placeplan_action);
#endif
    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();

    return 0;
}