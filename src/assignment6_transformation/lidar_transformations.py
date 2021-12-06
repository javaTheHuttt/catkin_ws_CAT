import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs import point_cloud2
import tf
import tf2_ros

# we did not used the laser_geomerty package but analysed its code:
# https://github.com/ros-perception/laser_geometry/blob/kinetic-devel/src/laser_geometry/laser_geometry.py
# to understand how we can calculate the points

def transforme_points_to_lab_frame(points):
    # read from execise sheet command
    qx = 0
    qy = 0
    qz = 0
    qw = 1
    
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "laser"
    poses_in_lab = PoseArray()
    for i in range(len(points)):
        point = points[i]
        # fill pose
        pose = Pose()
        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = point[2]
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        #fill rest of pose stamp
        pose_stamped.pose = pose
        pose_stamped.header.stamp = rospy.Time(0)
        
        # transforme pose to lab frame and add to PoseArray
        poses_in_lab.poses.append(tf_buffer.transform(pose_stamped, "lab", rospy.Duration(2)).pose)
    poses_in_lab.header = pose_stamped.header
    return poses_in_lab

def received_scan_callback(scan):
    point_amount = len(scan.ranges)
    # calculation for map and point data
    angles = scan.angle_min + np.arange(point_amount) * scan.angle_increment
    our_map = np.array([np.cos(angles), np.sin(angles)])
    # ranges relatively to our map
    data = np.array(scan.ranges) * our_map

    points = []
    for i in range(point_amount):
        range_elem = scan.ranges[i]
        if range_elem >= scan.range_min and range_elem <= scan.range_max: # filters the infinities etc out
            point = data[:, i].tolist()
            # z = height does not change
            point.append(0)
            points.append(point)
    pc2 = point_cloud2.create_cloud_xyz32(scan.header, points)
    publisher.publish(pc2)
    
    transformed_points = transforme_points_to_lab_frame(points)
    publisher_pose.publish(transformed_points)
    


rospy.init_node("speed_from_tick_node")

# register subscribers
rospy.Subscriber("/sensors/rplidar/scan", LaserScan, received_scan_callback)

# register publishers
publisher = rospy.Publisher("/sensors/rplidar/point_cloud_2", PointCloud2, queue_size=10)
publisher_pose = rospy.Publisher("/sensors/rplidar/lab_frame_poses", PoseArray, queue_size=10)
# block until node shuts down
rospy.spin()
