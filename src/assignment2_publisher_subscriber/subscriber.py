#!/usr/bin/env python3

# using the example of the slides 02-19

import rospy

# this does not work
# TODO: find out which type to import
from autominy_msgs.msg import Speed

def received_speed_callback(raw_msg):
    # TODO: we probably need to process this to get the speed e.g. as a float
    print(raw_msg)


rospy.init_node("assignment2_speed_subscriber")
# TODO: find out python speed data type
rospy.Subscriber("/sensors/speed", Speed, received_speed_callback)

# block until node shuts down
rospy.spin()
