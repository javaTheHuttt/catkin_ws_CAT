#!/usr/bin/env python3
from sensor_msgs.msg import CameraInfo
import rospy
from our_msgs.msg import CameraParameters

def received_info_callback(camera_info):
    print("Received camera Info:")
    #     [fx  0 cx]
    # K = [ 0 fy cy]
    #     [ 0  0  1]
    intrinsic_camera_matrix = camera_info.K
    fx = intrinsic_camera_matrix[0]
    fy = intrinsic_camera_matrix[4]
    cx = intrinsic_camera_matrix[2]
    cy = intrinsic_camera_matrix[5]
    print(f"fx = {fx}; fy = {fy}; cx = {cx}; cy = {cy}")

    # distortion parameters
    distortion_parameters = camera_info.D
    k1 = distortion_parameters[0]
    k2 = distortion_parameters[1]
    t1 = distortion_parameters[2]
    t2 = distortion_parameters[3]
    k3 = distortion_parameters[4]
    print(f"k1 = {k1}; k2 = {k2}; t1 = {t1}; t2 = {t2}; k3 = {k3}")
    print()
    msg = CameraParameters()
    msg.fx = fx
    msg.fy = fy
    msg.cx = cx
    msg.cy = cy

    msg.k1 = k1
    msg.k2 = k2
    msg.t1 = t1
    msg.t2 = t2
    msg.k3 = k3
    publisher.publish(msg)

rospy.init_node("camera_info_extraction_node")
rospy.Subscriber("/sensors/camera/infra1/camera_info", CameraInfo, received_info_callback)
publisher = rospy.Publisher("/sensors/camera/infra1/image_rect_raw_cam_params", CameraParameters, queue_size=10)

# block until node shuts down
rospy.spin()
