#include "arcl_youbot_vision/util.hpp"
#include "arcl_youbot_vision/target_selector.hpp"
// typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;








// void callback(const PointCloud::ConstPtr& msg)
// {
//   printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
// //   BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//     // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_process");
    ros::NodeHandle nh;

    t_selector = target_selector(nh);    


    t_selector.set_rgb_topic("/youbot_rgb_image");
    t_selector.set_depth_topic("/youbot_depth_image");
    t_selector.set_camera_info_topic("/youbot_camera_info");
    t_selector.set_camera_frame("camera_rgb_optical_frame");
    t_selector.set_base_frame("base_link");
    t_selector.set_image_segment_topic("/image_segment");
    t_selector.set_segment_mask_topic("/segment_mask");
    t_selector.get_current_scene();


    //publish_pointcloud(nh, "point_base_link", cloud_base_link);
    
    pub = rospy.Publisher('/image_segment', Image, queue_size=10)
        while not rospy.is_shutdown():
            connections = pub.get_num_connections()
            rospy.loginfo('Connections: %d', connections)
            if connections > 0:
                pub.publish(rgb_image_msg)
                break

        mask_list =rospy.wait_for_message("/segment_mask", AllMask)

    ros::Publisher image_segment_pub = nh.advertise<sensor_msgs::Image>("/image_segment", 3);
    while(1){
        if(image_segment_pub.getNumSubscribers() > 0){
            image_segment_pub.publish(msg_image);
            break;
        }
    }

    
    ros::spin();
}