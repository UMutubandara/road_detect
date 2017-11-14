import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import math
import os

import sys
import rospy
import cv2
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#from __future__ import print_function




class RoadDetect:

	def __init__(self):
		self.debug_image_pub = rospy.Publisher("image_debug",Image,queue_size=10)
		self.result_image_pub = rospy.Publisher("image_result",Image,queue_size=10)
		self.point_pub = rospy.Publisher("image_target",Point,queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/videofile/image_raw",Image, self.callback)

	def callback(self,data):
		try:
			self.raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		cropped_image = None
		result_image = None

		try:
			rospy.loginfo("Got new image")
			cropped_image = self.detectLines(self.raw_image)
			result_image = self.estimateRoadCenter(cropped_image)
		except TypeError as e:
			#rospy.loginfo("Empty Vec")
			print(e)

		# CV images back into ROS
		#if cropped_image != None
		try:
			self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(cropped_image, "mono8"))
		except CvBridgeError as e:
			print(e)


		try:
			self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
		except CvBridgeError as e:
			print(e)



	def region_of_interest(self, img, vertices):
		# Define a blank matrix that matches the image height/width.
		mask = np.zeros_like(img)

		# Create a match color with the same color channel counts.
		match_mask_color = 255
		
		# Fill inside the polygon
		cv2.fillPoly(mask, vertices, match_mask_color)

		# Returning the image only where mask pixels match
		masked_image = cv2.bitwise_and(img, mask)
		return masked_image

	def draw_lines(self, img, lines, color=[255, 0, 255], thickness=5):
		# If there are no lines to draw, exit.
		if lines is None:
			return

		# Create a blank image that matches the original in size.
		line_img = np.zeros(
			(
				img.shape[0],
				img.shape[1],
				3
			),
			dtype=np.uint8,
		)

		# Loop over all lines and draw them on the blank image.
		for line in lines:
			for x1, y1, x2, y2 in line:
				cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)

		# Merge the image with the lines onto the original.
		img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

		# Return the modified image.
		return img

	def detectLines(self, image):
		# Read in the image and print some stats
		
		
		#copy new image
		ysize = image.shape[0]
		xsize = image.shape[1]


		#region of interests 333
		corners = [(985, 468), (800, 582),(1200, 582)]

		
		# plt.imshow(cropped_image)
		# plt.show()

		#add greyscale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		#add canny detection with guassian blur
		kernel_size = 11	
		blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
		 

		low_threshold = 40
		high_threshold = 80
		edges = cv2.Canny(blur_gray, low_threshold, high_threshold)
		#plt.imshow(edges, cmap='gray')
		#plt.show()


		cropped_image = self.region_of_interest(edges, np.array([corners],np.int32))
		# plt.imshow(cropped_image, cmap='gray')
		# plt.show()

		return cropped_image


	def estimateRoadCenter(self, image):
		lines_image = cv2.HoughLinesP(	image,
										rho=3,
										theta=np.pi /60,
										threshold=25,
										lines=np.array([]),
										minLineLength=10,
										maxLineGap=20
									)
		

		# for x in range(0, len(lines_image)):
		# 	for x1,y1,x2,y2 in lines_image[x]:
				#cv2.line(newimage,(x1,y1),(x2,y2),(0,255,0),5)

		lines_image = lines_image.astype('float32')
		#print(lines_image)

		# plt.imshow(newimage)	
		# plt.show()
		


		left_line_x = []
		left_line_y = []
		right_line_x = []
		right_line_y = []

		
		for x in range(0, len(lines_image)):
			for x1,y1,x2,y2 in lines_image[x]:
				slope = (y2 - y1) / (x2 - x1)
				if math.fabs(slope) < 0.5: # <-- Only consider extreme slope
					continue
				if slope <= 0: # <-- If the slope is negative, left group.
					left_line_x.extend([x1, x2])
					left_line_y.extend([y1, y2])
				else: # <-- Otherwise, right group.
					right_line_x.extend([x1, x2])
					right_line_y.extend([y1, y2])


		min_y = 460 # <-- Just below the horizon
		max_y = 580 # <-- The bottom of the image

		#print(right_line_x, right_line_y, left_line_x, left_line_y)

		poly_left = np.poly1d(np.polyfit(left_line_y,left_line_x, 1))

		left_x_start = int(poly_left(max_y))
		left_x_end = int(poly_left(min_y))

		poly_right = np.poly1d(np.polyfit(right_line_y,right_line_x, 1))

		right_x_start = int(poly_right(max_y))
		right_x_end = int(poly_right(min_y))

		line_image = self.draw_lines(
								self.raw_image,
											[[
											[left_x_start, max_y, left_x_end, min_y],
											[right_x_start, max_y, right_x_end, min_y],
											]],
								thickness=5,
								)

		return line_image
