#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan

# import from parent directory
# import sys
# sys.path.append('../AutoMiny-exercises')
from simple_drive_control.srv import *

points_skipped = 0

class ObstacleAvoidanceNode:

    def __init__(self) -> None:

        rospy.init_node("ObstacleAvoidance")

        # register subscribers
        rospy.Subscriber("/sensors/rplidar/scan", LaserScan, self.received_scan_callback)

        self.driving_maneuver_client = rospy.ServiceProxy("driving_maneuver", DrivingManeuver)

    # callback for lidar scan
    def received_scan_callback(self, scan):
        global points_skipped
        if points_skipped < 10:
            points_skipped += 1
            return
        else:
            points_skipped = 0
        # print("scan.range_min", scan.range_min)
        # print("scan.range_max", scan.range_max)
        # print("scan.angle_increment", scan.angle_increment)
        # print("scan.angle_min", scan.angle_min)
        # print("scan.angle_max", scan.angle_max)
        # print("scan.ranges", scan.ranges)

        # TODO: filter points hitting the car itself
        num_points = round(2 * scan.angle_max / scan.angle_increment) + 1 # equals len(scan.ranges)
        front_range = scan.ranges[0]
        # point at back at 0 degree in the middle of array
        back_range = scan.ranges[(num_points - 1) // 2]
        right_range = scan.ranges[-(num_points - 1) // 4]
        left_range = scan.ranges[(num_points - 1) // 4]
        
        print("front range", front_range)
        print("left_range", left_range)
        print("right_range",right_range)
        # print("back_range", back_range)

        # calculate steering direction if obstacle is < 0.5m in front of the car
        if front_range < 0.8:
            # compare points left and right around the front point
            increment = 10 # index increment (for 360 points 1 increment equals 1 degree)
            left_increment_range = 0
            right_increment_range = 0
            # if the difference between the points is very small increase increment
            while (abs(left_increment_range - right_increment_range) < 0.05):
                left_increment_range = scan.ranges[increment]
                right_increment_range = scan.ranges[-increment]

                increment += 10

                # 180 degree in both directions
                # TODO: increment should be relative to num_points, this works only with 360 points
                if increment > (num_points // 2):
                    print("Help, idk where to go, human take over!")
                    return

            print("left_increment_range", left_increment_range)
            print("right_increment_range", right_increment_range)

            action = "right"
            if left_increment_range > right_increment_range:
                action = "left"
            self.driving_maneuver_client.call(direction="forward", steering=action, distance=0.1)
        # if obstacle appears on the side of the car move a bit to the opposite side
        elif left_range < 0.1:
            if right_range >= 0.3:
                self.driving_maneuver_client.call(direction="forward", steering="right", distance=0.2)
                self.driving_maneuver_client.call(direction="forward", steering="left", distance=0.1)
            else:
                print("Help, idk where to go, human take over!")
                return
        elif right_range < 0.1:
            if left_range >= 0.3:
                self.driving_maneuver_client.call(direction="forward", steering="left", distance=0.2)
                self.driving_maneuver_client.call(direction="forward", steering="right", distance=0.1)
            else:
                print("Help, idk where to go, human take over!")
                return
        else:
            # if there is no obstacle close continue driving straight
            self.driving_maneuver_client.call(direction="forward", steering="straight", distance=0.1)
           


if __name__ == "__main__":
    ObstacleAvoidanceNode()

    # block until node shuts down
    rospy.spin()
