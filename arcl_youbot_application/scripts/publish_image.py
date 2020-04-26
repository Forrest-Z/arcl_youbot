#!/usr/bin/env python
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
from __future__ import division
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import rospy
import time
import tf
import arcl_youbot_application.application_util as app_util
import arcl_youbot_planner.arm_planner.arm_util as arm_util
import arcl_youbot_application.vision_util as vision_util
from geometry_msgs.msg import Point32
from arcl_youbot_msgs.msg import SingleMask, AllMask
from sensor_msgs.msg import PointCloud, Image, ChannelFloat32, CameraInfo
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_about_axis, quaternion_from_matrix
from cv_bridge import CvBridge, CvBridgeError


DEBUG_ = True 

def get_pointcloud_center(pc):
    num_pts = 0
    center_x = 0
    center_y = 0
    center_z = 0

    for pt in pc.points:
        center_x = center_x + pt.x
        center_y = center_y + pt.y
        center_z = center_z + pt.z
        num_pts = num_pts + 1
    center_x = center_x / num_pts
    center_y = center_y / num_pts
    center_z = center_z / num_pts
    return [center_x, center_y, center_z]    

if __name__ == "__main__":

    rospy.init_node('pick_from_rgbd')
    # Create a pipeline
    env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    # env = app_util.YoubotEnvironment(-2.5, 2.5, 0.0, 5.0)
    env.mode = 1
    pipeline = rs.pipeline()

    # env.move_to_local_target('youbot_0', 0, -0.0334055007874, 0)

    #Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start streaming and get color frame intrinsics
    profile = pipeline.start(config)

    profile = pipeline.get_active_profile()
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    color_intrinsics = color_profile.get_intrinsics()
    w, h = color_intrinsics.width, color_intrinsics.height
    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    if DEBUG_:
        print("Depth Scale is: " , depth_scale)

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 2 #1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)





    # Streaming loop
    try:
        time.sleep(0.8)
        # Get frameset of color and depth

        rgb_pub = rospy.Publisher('/youbot_rgb_image', Image, queue_size=10)
        depth_pub = rospy.Publisher('/youbot_depth_image', Image, queue_size=10)
        camera_info_pub = rospy.Publisher('/youbot_camera_info', CameraInfo, queue_size=10)

        camera_info_msg = CameraInfo()
        camera_info_msg.K = [color_intrinsics.fx, 0, color_intrinsics.ppx, 0, color_intrinsics.fy, color_intrinsics.ppy, 0, 0, depth_scale]
        r = rospy.Rate(3)
        while not rospy.is_shutdown():
            start_time = time.time()
            repeated_time = 0
            while repeated_time < 6:
                frames = pipeline.wait_for_frames()
                # frames.get_depth_frame() is a 640x360 depth image

                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    exit

                new_depth_image = np.asanyarray(aligned_depth_frame.get_data())
                if repeated_time > 0:
                    depth_image[depth_image==0] = new_depth_image[depth_image==0]
                else:
                    depth_image = new_depth_image
                # print(temp_depth_image.shape)
                
                # for row_index in range(depth_image.shape[0]):
                #     for col_index in range(depth_image.shape[1]):
                #         if depth_image[row_index,col_index] == 0:
                #             depth_image[row_index,col_index] = temp_depth_image[row_index,col_index]
                color_image = np.asanyarray(color_frame.get_data())
                repeated_time += 1

            # cv2.imwrite('scene_rgb.png', color_image)
            bridge = CvBridge()
            rgb_image_msg = bridge.cv2_to_imgmsg(color_image, "bgr8")
            depth_image_msg = bridge.cv2_to_imgmsg(depth_image, "passthrough")
            # print(depth_image_msg)
            
            # cv2.imshow("bgr8",color_image)
            # cv2.waitKey(-1)

            


            

            
            rgb_pub.publish(rgb_image_msg)
            depth_pub.publish(depth_image_msg)
            camera_info_pub.publish(camera_info_msg)
            r.sleep()
            # print("--- %s seconds ---" % (time.time() - start_time))  
    finally:
        pipeline.stop()




