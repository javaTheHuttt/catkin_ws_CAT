#!/usr/bin/env python3

from sensor_msgs.msg import Image
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt

###
### Debug methods
###
# open window displaying the given image, waiting for any key and quits
def print_debug_image(image, name="Debug Image"):
    cv2.imshow(name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


###
### Image calculations
###

# converts a buffer of pixel values (1D array) to a matrix with height and with (2D)
def image_buffer_to_matrix(buffer, width, height):
    matrix = np.empty(shape=(height, width))

    for i in range(len(buffer)):
        pixel = buffer[i]
        row = int(i / width)
        column = i % width
        matrix[row][column] = pixel

    return matrix

# draw black rectangles so that only the lanes remain
def mask_unrelated_areas(image, width, height):
    black = (0,0,0)
    # black = (255,255,255)

    # top area (e.g. the white walls)
    cv2.rectangle(image, (0,0), (width,100), black, -1) # -1 means fill

    # area on the right hand side of the right lane (walls, calibration markers, rectangle around the field)
    cv2.rectangle(image, (width-50,0), (width,height), black, -1)
    cv2.rectangle(image, (width-100,0), (width,height-100), black, -1)
    cv2.rectangle(image, (width-150,0), (width,height-160), black, -1)
    cv2.rectangle(image, (width-200,0), (width,height-220), black, -1)
    cv2.rectangle(image, (width-250,0), (width,height-310), black, -1)
    cv2.rectangle(image, (width-300,0), (width,height-400), black, -1)

    # white pixels of the car
    cv2.rectangle(image, ((width//2)-65,(height//2)+50), ((width//2) + 50,(height//2) + 100), black, -1)


# publish image from 2D matrix
def publish_image(camera_image, image_matrix):
    # convert 2D matrix to buffer (1D array) of uint8
    image_matrix = image_matrix.ravel() # flatten 2D array
    image_matrix = np.uint8(image_matrix) # convert from float32 (required by cv2) to uint8 (required for image message type)
    image_matrix = image_matrix.tolist() # returns python array

    camera_image.data = image_matrix
    publisher.publish(camera_image)

# subscriber callback
def received_image_callback(camera_image):
    print(f"Received {camera_image.width} x {camera_image.height} camera image with {camera_image.encoding} encoding")

    image = image_buffer_to_matrix(camera_image.data, camera_image.width, camera_image.height)
    # openCV requires numpy float array
    image = np.float32(image)

    # mask unrelated areas with black rectangles
    mask_unrelated_areas(image, camera_image.width, camera_image.height)

    # 254 is the maximum threshhold value usable here
    _, binary_image = cv2.threshold(image, 190, 255, cv2.THRESH_BINARY)
    # print_debug_image(binary_image, "Binary")

    # publish the binary image
    publish_image(camera_image, binary_image)

rospy.init_node("lane_image_to_binary_conversion")

# register subscribers
rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, received_image_callback)

# register publishers
publisher = rospy.Publisher("/sensors/camera/infra1/image_rect_raw_binary", Image, queue_size=10)

# block until node shuts down
rospy.spin()
