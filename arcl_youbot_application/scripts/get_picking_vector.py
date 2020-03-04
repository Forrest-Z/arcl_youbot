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
from geometry_msgs.msg import Point32
from arcl_youbot_msgs.srv import ImageSegment
from arcl_youbot_msgs.msg import SingleMask, AllMask
from sensor_msgs.msg import PointCloud, Image, ChannelFloat32
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
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

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
        frames = pipeline.wait_for_frames()
        listener = tf.TransformListener()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            exit

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        cv2.imwrite('scene_rgb.png', color_image)
        cv2.imwrite('scene_depth.png', depth_image)
        bridge = CvBridge()
        rgb_image_msg = bridge.cv2_to_imgmsg(color_image, "bgr8")
        cv2.imshow("bgr8",color_image)
        cv2.waitKey(-1)


        pub = rospy.Publisher('/image_segment', Image, queue_size=10)
        while not rospy.is_shutdown():
            connections = pub.get_num_connections()
            rospy.loginfo('Connections: %d', connections)
            if connections > 0:
                pub.publish(rgb_image_msg)
                break

        mask_list =rospy.wait_for_message("/segment_mask", AllMask)
        print(mask_list)









        print(depth_image)
        print('depth shape:')
        print(depth_image.shape)
        print('color shape:')
        print(color_image.shape)

        target_pointcloud = np.dstack((depth_image,depth_image,depth_image)).astype(np.float) #depth image is 1 channel, color is 3 channels
        print("pointlcoud size:")
        print(target_pointcloud.shape)

        pointcloud_in_youbot_base_cut = PointCloud()
        pointcloud_in_youbot_base_cut.header.frame_id = 'base_link'
        pointcloud_in_camera_frame = PointCloud()
        pointcloud_in_camera_frame.header.frame_id = 'camera_rgb_optical_frame'
        point_num = 0
        for ins in mask_list.mask_list:
            for pt in range(len(ins.col_list)):
                u = ins.row_list[pt]
                v = ins.col_list[pt]
                target_pointcloud[u,v,2] = float(depth_image[u][v]) * depth_scale
                target_pointcloud[u,v,0] = (v - color_intrinsics.ppx) / color_intrinsics.fx * target_pointcloud[u][v][2]
                target_pointcloud[u,v,1] = (u - color_intrinsics.ppy) / color_intrinsics.fy * target_pointcloud[u][v][2]
                target_pt = Point32()
                target_pt.x = target_pointcloud[u][v][0]
                target_pt.y = target_pointcloud[u][v][1]
                target_pt.z = target_pointcloud[u][v][2]
                if DEBUG_:
                    print("x:"+str(target_pt.x) + " y:"+str(target_pt.y) + " z:"+str(target_pt.z))
                pointcloud_in_camera_frame.points.append(target_pt)
                rgb_pt = ChannelFloat32()
                rgb_pt.name = "rgb"
                rgb_pt.values.append(color_image[u][v][2]) # r
                rgb_pt.values.append(color_image[u][v][1]) # g
                rgb_pt.values.append(color_image[u][v][0]) # b
                # pointcloud_in_camera_frame.channels.append(rgb_pt)

                point_num += 1
        



        pointcloud_in_youbot_base = listener.transformPointCloud("base_link", pointcloud_in_camera_frame)
        [center_pt_x, center_pt_y, center_pt_z] = get_pointcloud_center(pointcloud_in_youbot_base)

        
        print("center_x:")
        print(center_pt_x)
        print("center_y:")
        print(center_pt_y)
        print("center_z:")
        print(center_pt_z)
        # point_num = pointcloud_in_youbot_base.points.count
        print("all point_num:" + str(point_num))
  

        pointcloud_pub = rospy.Publisher('pointcloud_in_youbot_base', PointCloud, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            pointcloud_pub.publish(pointcloud_in_youbot_base)
        rate.sleep()
        pca_data = np.zeros((cube_point_num, 3))
        index = 0
        for pt in pointcloud_in_youbot_base.points:
            if pt.z > -0.03 and pt.z < 0.05:
            # if pt.z < -0.03:
                pca_data[index][:] = [pt.x, pt.y, pt.z]
                index += 1
        
        # print(pca)
        
        centered_pca_data = pca_data
        cube_center = np.mean(pca_data, axis = 0)
        print("cube_center_x:" + str(cube_center[0]))
        print("cube_center_y:" + str(cube_center[1]))
        print("cube_center_z:" + str(cube_center[2]))

        centered_pca_data[:,0] = np.subtract(centered_pca_data[:,0], cube_center[0])
        centered_pca_data[:,1] = np.subtract(centered_pca_data[:,1], cube_center[1])
        centered_pca_data[:,2] = np.subtract(centered_pca_data[:,2], cube_center[2])

        print(pca_data.shape)
        print(pca_data.ndim)
        u, s, vh = np.linalg.svd(centered_pca_data)
        u.shape, s.shape, vh.shape
        print("u:")
        print(u)
        print("s:")
        print(s)
        print("vh:")
        print(vh)
        print(vh[0][0])
        print(vh[1][0])
 
        primary_axis_x = vh[0][0]
        primary_axis_y = vh[1][0]
        # env.send_local_grasp_action("cube", cube_center[0], cube_center[1], cube_center[2], primary_axis_x, primary_axis_y)
        # target_pos_2d = [0, 0, 0]
        # target_pose = env.local_grasp_plan_result.final_base_pose
        # target_pos_2d[0] = target_pose.position.x
        # target_pos_2d[1] = target_pose.position.y
        # q = (target_pose.orientation.x,
        #      target_pose.orientation.y,
        #      target_pose.orientation.z,
        #      target_pose.orientation.w)
        # (_, _, yaw) = euler_from_quaternion(q)
        # target_pos_2d[2] = yaw


        # print("target_pose_x:" + str(target_pos_2d[0]))
        # print("target_pose_y:" + str(target_pos_2d[1]))
        # print("target_pose_theta:" + str(target_pos_2d[2]))
        # target_pos_2d[0] += 0.02
        # env.move_to_local_target('youbot_0', target_pos_2d[0], target_pos_2d[1], target_pos_2d[2])
        # pick_joint_value = [env.local_grasp_plan_result.q1, env.local_grasp_plan_result.q2, env.local_grasp_plan_result.q3, env.local_grasp_plan_result.q4, env.local_grasp_plan_result.q5]
        # pre_pick_joint_value = [env.local_grasp_plan_result.q1_pre, env.local_grasp_plan_result.q2_pre, env.local_grasp_plan_result.q3_pre, env.local_grasp_plan_result.q4_pre, env.local_grasp_plan_result.q5_pre]    
        # env.pick_object("youbot_0", pick_joint_value, pre_pick_joint_value)
        
        




        # Render images
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # images = np.hstack((bg_removed, depth_colormap))
        # cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('Align Example', images)
        # # cv2.imwrite('red.png', images)
        # key = cv2.waitKey(-1)
        # # Press esc or 'q' to close the image window
        # if key & 0xFF == ord('q') or key == 27:
        #     cv2.destroyAllWindows()
        #     exit


            
    finally:
        pipeline.stop()
