#!/usr/bin/env python3

# using the example of the slides 02-19

import rospy
from autominy_msgs.msg import Speed

def received_speed_callback(raw_msg):
    print(raw_msg)

rospy.init_node("assignment2_speed_subscriber")
rospy.Subscriber("/sensors/speed", Speed, received_speed_callback)

# block until node shuts down
rospy.spin()
