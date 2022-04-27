#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# By Trey Castle

img_received = False
# define a 720x1280 3-channel image with all pixels equal to zero
rgb_img = np.zeros((720, 1280, 3), dtype="uint8")


# get the image message
def get_image(ros_img):
	global rgb_img
	global img_received
	# convert to opencv image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	# raise flag
	img_received = True

#filter out background color
def mask(image):
	# constructs an image of a rectangle then ANDs it against the recieved image to cut out the background
	filter = np.zeros((720, 1280), dtype="uint8")
	start = (150, 150)
	end = (1000, 600)
	cv2.rectangle(filter, start, end, (255, 255, 255), -1)
	masked = cv2.bitwise_and(image, filter)
	return masked

# identifies which parts of the image fall in the bounds of the ball
def filter_image(image):
	var = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
	lower_yellow = np.array([25,0,1])
	upper_yellow = np.array([60,255,255])
	yellow_mask = cv2.inRange(var, lower_yellow, upper_yellow)
	return yellow_mask


if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('detect_ball', anonymous=True)
	# define a subscriber to ream images
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
	# define a publisher to publish images
	img_pub = rospy.Publisher('/ball_2D', Image, queue_size=1)
	# set the loop frequency
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if img_received:
			ball = filter_image(rgb_img)
			ball = mask(ball)
			# convert it to ros msg and publish it
			img_msg = CvBridge().cv2_to_imgmsg(ball, encoding="mono8")
			# publish the image
			img_pub.publish(img_msg)
		# pause until the next iteration
		rate.sleep()
