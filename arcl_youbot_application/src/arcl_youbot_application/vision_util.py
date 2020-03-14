#!/usr/bin/env python
import rospy
import math
import pybullet as p
import time
import numpy as np
from sensor_msgs.msg import PointCloud, Image, ChannelFloat32
from geometry_msgs.msg import Point32
import scipy.spatial
import tf


class CameraInfo():
    def __init__(self, depth_scale, width, height, fx, fy, offset_x, offset_y):
        self.depth_scale = depth_scale
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.offset_x = offset_x
        self.offset_y = offset_y

class TargetSelector():
    # method:
    #    0:  choose the instance with highest z-coordinate among all the graspable instances
    #    
    def __init__(self, camera_info):
        self.method = 0
        self.camera_info = camera_info

    # method 0
    def get_mask_list(self, mask_list):
        self.mask_list = mask_list
    
    def get_depth_image(self, depth_image):
        self.depth_image = depth_image

    def get_pointcloud_center(self, pc):
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


    def get_top_surface(self, target_pc):
        neighbor_num = 30
        top_surface = PointCloud()
        top_surface.header.frame_id = "base_link"
        point_num = len(target_pc.points)
        point_mat = np.zeros([3, len(target_pc.points)])
        for index, pt in enumerate(target_pc.points):
            point_mat[0][index] = pt.x 
            point_mat[1][index] = pt.y 
            point_mat[2][index] = pt.z 

        pc_tree = scipy.spatial.cKDTree(point_mat.T)

        point_normal_mat = np.zeros([3, len(target_pc.points)])
        print("total point num:" + str(point_num))
        [center_x, center_y, center_z] = self.get_pointcloud_center(target_pc)
        print("center_x:" + str(center_x) + ", center_y:" + str(center_y) + ", center_z:" + str(center_z))






        for index in range(point_num):
            # calculate each point's normal direction
            dist, neigh_index_list = pc_tree.query(point_mat[:, index], neighbor_num)
            # print("neigh_index_list:")
            # print(neigh_index_list)
            pca_data = np.zeros((neighbor_num + 1, 3))
            pca_data[neighbor_num, :] = point_mat[:, index].T 
            for i, neigh_index in enumerate(neigh_index_list):     
                pca_data[i][:] = point_mat[:, neigh_index].T
            # print(pca_data)

            centered_pca_data = pca_data
            cube_center = np.mean(pca_data, axis = 0)
            # print("cube_center_x:" + str(cube_center[0]))
            # print("cube_center_y:" + str(cube_center[1]))
            # print("cube_center_z:" + str(cube_center[2]))

            centered_pca_data[:,0] = np.subtract(centered_pca_data[:,0], cube_center[0])
            centered_pca_data[:,1] = np.subtract(centered_pca_data[:,1], cube_center[1])
            centered_pca_data[:,2] = np.subtract(centered_pca_data[:,2], cube_center[2])
            # print(centered_pca_data)
            u, s, vh = np.linalg.svd(centered_pca_data)
            u.shape, s.shape, vh.shape
            # print("u:")
            # print(u)
            # print("s:")
            # print(s)
            # print("vh:")
            # print(vh)

            min_axis_index = np.where(s == np.amin(s))
            # if len(min_axis_index)>1 :
                # print(s)
            # print("min_axis_index:" + str(min_axis_index))
            normal_axis = np.reshape(vh[:, min_axis_index], 3)
            # print(normal_axis.shape)
            # print(normal_axis.dtype)
            # print(normal_axis)
            if normal_axis[2] > 0.7 or normal_axis[2] < -0.7:
                top_surface_pt = Point32()
                top_surface_pt.x = point_mat[0, index]
                top_surface_pt.y = point_mat[1, index]
                top_surface_pt.z = point_mat[2, index]
                top_surface.points.append(top_surface_pt)
                # print("add")
                
        print("top surface point num:" + str(len(top_surface.points)))
        return top_surface

    def get_next_target(self):
        if self.method == 0:
            target_pointcloud = np.dstack((self.depth_image,self.depth_image,self.depth_image)).astype(np.float) #depth image is 1 channel, color is 3 channels
            # print("pointlcoud size:")
            # print(target_pointcloud.shape)
            listener = tf.TransformListener()

            pointcloud_in_camera_frame = PointCloud()
            pointcloud_in_camera_frame.header.frame_id = 'camera_rgb_optical_frame'
            point_num = 0
            highest_ins_index = 0
            highest_z = -1.0
            chosen_pointcloud = PointCloud()
            for ins_index, ins in enumerate(self.mask_list.mask_list):

                z_height = 0
                current_ins_pc = PointCloud()
                current_ins_pc.header.frame_id = 'camera_rgb_optical_frame'
                for pt in range(len(ins.col_list)):
                    u = ins.row_list[pt]
                    v = ins.col_list[pt]
                    if self.depth_image[u][v] != 0:
                        target_pointcloud[u,v,2] = float(self.depth_image[u][v]) * self.camera_info.depth_scale
                        target_pointcloud[u,v,0] = (v - self.camera_info.offset_x) / self.camera_info.fx * target_pointcloud[u][v][2]
                        target_pointcloud[u,v,1] = (u - self.camera_info.offset_y) / self.camera_info.fy * target_pointcloud[u][v][2]
                        target_pt = Point32()
                        target_pt.x = target_pointcloud[u][v][0]
                        target_pt.y = target_pointcloud[u][v][1]
                        target_pt.z = target_pointcloud[u][v][2]
                        # if DEBUG_:
                        # if target_pt.z < 0.1:
                            # print("x:"+str(target_pt.x) + " y:"+str(target_pt.y) + " z:"+str(target_pt.z))
                            # print("u:"+str(u))
                            # print("v:"+str(v))

                        current_ins_pc.points.append(target_pt)
                        point_num += 1

                current_ins_base_frame = listener.transformPointCloud("base_link", current_ins_pc)
                [center_pt_x, center_pt_y, center_pt_z] = self.get_pointcloud_center(current_ins_base_frame)
                if center_pt_z > highest_z:
                    highest_z = center_pt_z
                    highest_ins_index = ins_index
                    chosen_pointcloud = current_ins_base_frame

            
            top_surface = self.get_top_surface(chosen_pointcloud)            
            
            #DEBUG START, plotting the pointcloud
            pointcloud_pub = rospy.Publisher('top_surface_pointcloud_in_youbot_base', PointCloud, queue_size=10)
            rate = rospy.Rate(10) # 10hz
            while not rospy.is_shutdown():
                pointcloud_pub.publish(top_surface)
                rate.sleep()  
            #DEBUG END, plotting the pointcloud

    