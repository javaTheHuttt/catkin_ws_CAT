#!/usr/bin/env python3

import rospy
from autominy_msgs.msg import Tick
from simple_drive_control.srv import *

tick_value = 0
do_tick_cal = False

def execute_test_drive(client, steering, distance):
    global do_tick_cal

    # start measuring ticks
    do_tick_cal = True

    # request test drive
    client.call(direction="forward", steering=steering, distance=distance)

    # stop measuring ticks
    do_tick_cal = False

def received_tick_callback(tick):
    global do_tick_cal
    if not do_tick_cal:
        return
    global tick_amount
    global tick_value
    tick_value += tick.value

rospy.init_node("meter_per_tick_node")

# get driving client
driving_client = rospy.ServiceProxy("driving_maneuver", DrivingManeuver)

# subscribe to tick topic
rospy.Subscriber("/sensors/arduino/ticks", Tick, received_tick_callback)

# execute test drive
steering="straight" 
distance=2.0
execute_test_drive(driving_client, steering, distance)

print("tick value: ", tick_value, "meters per tick: ", distance/tick_value)
