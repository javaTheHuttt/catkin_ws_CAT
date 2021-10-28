#!/usr/bin/env python3

import rospy
from autominy_msgs.msg import Speed

from time import sleep

rospy.init_node("assignment2_speed_publisher")

publisher = rospy.Publisher("/sensors/speed", Speed, queue_size=10)

while not rospy.is_shutdown():
    speed = Speed()
    speed.value = 0.3
    publisher.publish(speed)
    sleep(5)
