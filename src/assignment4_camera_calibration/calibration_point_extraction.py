#!/usr/bin/env python3

from sensor_msgs.msg import Image
import rospy
import cv2
import numpy as np

# open window displaying the given image, waiting for any key and quits
def print_debug_image(image, name="Debug Image"):
    cv2.imshow(name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# converts a buffer of pixel values (1D array) to a matrix with height and with (2D)
def image_buffer_to_matrix(buffer, width, height):
    matrix = np.empty(shape=(height, width))

    pixel_count = 0
    for pixel in buffer:
        row = int(pixel_count / width)
        column = pixel_count % width
        matrix[row][column] = pixel
        pixel_count += 1

    return matrix

# draw black rectangles so that only the calibration markers remain
def mask_unrelated_areas(image, width, height):
    black = (0,0,0)
    # top area
    cv2.rectangle(image, (0,0), (width,100), black, -1) # -1 means fill

    # bottom area
    cv2.rectangle(image, (0,height-230), (width,height), black, -1)

# publish image from 2D matrix
def publish_image(camera_image, image_matrix):
    # convert 2D matrix to buffer (1D array) of uint8
    image_matrix = image_matrix.ravel()
    image_matrix = np.uint8(image_matrix)
    image_matrix = image_matrix.tolist()

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
    _, binary_image = cv2.threshold(image, 254, 255, cv2.THRESH_BINARY)
    # print_debug_image(binary_image, "Binary")

    # publish the binary image to view in rviz
    publish_image(camera_image, binary_image)


rospy.init_node("binary_image_conversion_node")

# register subscribers
rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, received_image_callback)

# register publishers
publisher = rospy.Publisher("/sensors/camera/infra1/image_rect_raw_binary", Image, queue_size=10)

# block until node shuts down
rospy.spin()
