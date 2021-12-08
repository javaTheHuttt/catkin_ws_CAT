#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseArray

class OccupancyGridNode:

    # initialize ros, grid, publisher and subscriber
    def __init__(self) -> None:

        rospy.init_node("assignment6_occupancy_grid")

        self.seq_header = 0 # increased after each update of the grid

        self.width_meters = 8
        self.length_meters = 9
        self.meters_per_cell = 0.01
        self.num_cells_width = int(self.width_meters / self.meters_per_cell)
        self.num_cells_height = int(self.length_meters / self.meters_per_cell)

        self.grid = None
        self.init_grid()

        rospy.Subscriber("/sensors/rplidar/lab_frame_poses", PoseArray, self.received_pose_array_callback)
        self.publisher = rospy.Publisher("/sensors/rplidar/occupancy_grid", OccupancyGrid, queue_size=10)


    def init_grid(self):
        self.grid = OccupancyGrid()

        # init header
        self.grid.header.seq = self.seq_header
        self.grid.header.stamp = rospy.Time.now()
        self.grid.header.frame_id = "occupancy_grid_lidar"

        # init map meta data
        self.grid.info.map_load_time = rospy.Time.now()
        self.grid.info.resolution = self.meters_per_cell
        self.grid.info.width = self.num_cells_width
        self.grid.info.height = self.num_cells_height
        # origin matches map origin, no rotation
        self.grid.info.origin.position.x = 0
        self.grid.info.origin.position.y = 0
        self.grid.info.origin.position.z = 0
        self.grid.info.origin.orientation.w = 1
        self.grid.info.origin.orientation.x = 0
        self.grid.info.origin.orientation.y = 0
        self.grid.info.origin.orientation.z = 0

        self.grid.data = np.zeros(dtype=np.int8, shape=(self.num_cells_width * self.num_cells_height))

    # calculate, update and publish grid if new lidar poses were received
    def received_pose_array_callback(self, pose_array):
        for pose in pose_array.poses:
            self.update_grid(pose.position.x, pose.position.y)

        self.publish_grid()

    def update_grid(self, x, y):

        # ignore point if outside map
        if x > self.width_meters or y > self.length_meters:
            print(x, y, "outside map")
            return

        # calculate grid index based on coordinates
        index = int(int(x) * self.num_cells_width + int(y) / self.meters_per_cell)
        # same grid used every time, so that previous points remain
        self.grid.data[index] = 100 # cell occupied

        # update header
        self.seq_header += 1
        self.grid.header.seq = self.seq_header

    def publish_grid(self):
        self.publisher.publish(self.grid)

if __name__ == "__main__":
    OccupancyGridNode()

    # block until node shuts down
    rospy.spin()
