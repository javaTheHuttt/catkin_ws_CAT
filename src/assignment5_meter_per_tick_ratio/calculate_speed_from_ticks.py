#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from autominy_msgs.msg import Tick, Speed
from collections import deque
import math

# assumption
# fixed tick publish rate: 100 Hz = 0.01 s(p)

# last 30 ticks
tick_collection = deque(maxlen=30)
position_collection = deque(maxlen=30)
distance_collection = deque(maxlen=29)
latest_odometry = Odometry()

def calculate_distance(position_a, position_b):
    return math.sqrt((position_b.x - position_a.x)**2 + (position_b.y - position_a.y)**2)

def save_distance(position_a, position_b):
    global distance_collection
    distance = calculate_distance(position_a, position_b)
    # save distance between points to avoid recalculating parts of the total distance
    distance_collection.append(distance)

def received_tick_callback(tick):
    global tick_collection
    global position_collection
    global distance_collection
    global latest_odometry

    # save current car position
    position_collection.append(latest_odometry.pose.pose.position)
    if len(position_collection) >= 2:
        save_distance(position_collection[-2], position_collection[-1])

    #save current tick
    tick_collection.append(tick.value)

    length = len(tick_collection)
    speed = Speed()
    if length >= 30: # start calculating speed if 30 ticks are collected
        # calculate total distance based on saved subdistances
        distance_list = list(distance_collection)
        total_distance = sum(distance_list)
        # calculate distance from the latest few subdistances to detect stopped car faster
        last_distance = sum(distance_list[-6:]) # if the last few position are the same, car does not move
        
        # calculate total tick value
        total_tick_value = sum(list(tick_collection))

        # coordinated are not perfect. stopped car can have slightly changing coordinates
        # if tick value is 0 for a while car is also stopped
        if last_distance < 0.001 or total_tick_value < 1.0e-6:
            speed.value = 0.0
        else:
            # calculate speed (meter per seconds)
            # see exercise sheet expllanation for more details
            speed.value = (total_distance/(total_tick_value/length))*100
        print("speed: ", speed.value)
        # publish driving speed
        publisher.publish(speed)


def received_ground_truth_callback(odometry):
    global latest_odometry
    latest_odometry = odometry

rospy.init_node("speed_from_tick_node")

rospy.Subscriber("/sensors/arduino/ticks", Tick, received_tick_callback)
rospy.Subscriber("/simulation/odom_ground_truth", Odometry, received_ground_truth_callback)

publisher = rospy.Publisher("/sensors/speed", Speed, queue_size=10)

# block until node shuts down
rospy.spin()
