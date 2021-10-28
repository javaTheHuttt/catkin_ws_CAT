#!/usr/bin/env python3

import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand

rospy.init_node("assignment2_speed_publisher")

# register publishers 
pubSteering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
pubSpeed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)

interval = 5
print(f"start publishing ever {interval} second(s)")
while not rospy.is_shutdown():

    # publish steering value
    steeringCmd = NormalizedSteeringCommand()
    steeringCmd.value = 1
    pubSteering.publish(steeringCmd)

    # publish steering value
    speedCmd = SpeedCommand()
    speedCmd.value = 0.3
    pubSpeed.publish(speedCmd)

    rospy.sleep(interval)
