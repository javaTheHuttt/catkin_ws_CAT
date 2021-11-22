#!/usr/bin/env python3

import rospy
import numpy as np
import time
from our_msgs.msg import Points, Point, CameraParameters
import cv2

center_points = np.array([], ndmin=2)
intrinsic_camera_matrix = np.array([], ndmin=3)
distortion_parameters = np.array([], ndmin=1)

def sort_center_points(points):
    # sorted based on lowest x value
    sorted_points = sorted(points.data, key=lambda point: point.x)
    middle = len(sorted_points)//2

    # identify left and right points
    left = sorted_points[:middle]
    right = sorted_points[middle:]

    # sort points based on lowest y value + reverse
    sorted_left = sorted(left, key=lambda point: point.y)
    sorted_left.reverse()

    sorted_right = sorted(right, key=lambda point: point.y)
    sorted_right.reverse()

    # create array in right point order
    return_array = []
    for i in range(len(sorted_left)):
        return_array.append(sorted_left[i])
        return_array.append(sorted_right[i])
    return return_array

def set_center_points(points):
    global center_points
    my_array = np.zeros(shape=(6,2), dtype=np.float32)
    index = 0
    sorted_points = sort_center_points(points)
    for point in sorted_points:
        my_array[index] = [point.x, point.y]
        index += 1
    center_points = my_array[:]

def received_point_callback(points):
    if center_points.size == 0:
        set_center_points(points)

def set_intrinsic_camera_matrix(cam_params):
    global intrinsic_camera_matrix
    intrinsic_camera_matrix = np.array([[cam_params.fx, 0, cam_params.cx],[0, cam_params.fy, cam_params.cy],[0,0,1]],dtype=np.float32)

def set_distortion_parameters(cam_params):
    global distortion_parameters
    distortion_parameters = np.array([[cam_params.k1], [cam_params.k2], [cam_params.t1], [cam_params.t2], [cam_params.k3]], dtype=np.float32)

def received_cam_parameters_callback(cam_params):
    if intrinsic_camera_matrix.size == 0:
        set_intrinsic_camera_matrix(cam_params)
    if distortion_parameters.size == 0:
        set_distortion_parameters(cam_params)

rospy.init_node("calculate_extrinsic_parameters_node")
rospy.Subscriber("/sensors/camera/infra1/image_rect_raw_points", Points, received_point_callback)
rospy.Subscriber("/sensors/camera/infra1/image_rect_raw_cam_params", CameraParameters, received_cam_parameters_callback)

while(center_points.size == 0 and intrinsic_camera_matrix.size == 0 and distortion_parameters.size == 0):
    time.sleep(1)

# assume that z value is 0
object_points = np.array([[0.5, 0.2, 0], [0.5, -0.2, 0], [0.8, 0.2, 0], [0.8, -0.2, 0], [1.1, 0.2, 0], [1.1, -0.2, 0]], dtype=np.float32)

retval, rvec, tvec = cv2.solvePnP(object_points, center_points, intrinsic_camera_matrix, distortion_parameters)
print("Results -> retval:", retval, "\nrvec: ", rvec, "\ntvec: ", tvec)

rotation_matrix, _ = cv2.Rodrigues(rvec)
print("rotation_matrix: ", rotation_matrix)