import matplotlib
import numpy as np
import math
import sys
import cv2


#from __future__ import print_function

		
	
def region_of_interest( img, vertices):
	# Define a blank matrix that matches the image height/width.
	mask = np.zeros_like(img)
	# Create a match color with the same color channel counts.
	match_mask_color = 255
	# Fill inside the polygon
	cv2.fillPoly(mask, vertices, match_mask_color)
	# Returning the image only where mask pixels match
	masked_image = cv2.bitwise_and(img, mask)
	return masked_image

def draw_lines( img, lines, color=[255, 0, 255], thickness=5):
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
def find_pt_inline(p1, p2, y):
    """
    Here we use point-slope formula in order to find a point that is present on the line
    that passes through our vanishing point (vp). 
    input: points p1, p2, and y. They come is as tuples [x, y]
    We then use the point-slope formula: y - b = m(x - a)
    y: y-coordinate of desired point on the line
    x: x-coordinate of desired point on the line
    m: slope
    b: y-coordinate of p1
    a: x-coodrinate of p1
    x = p1x + (1/m)(y - p1y)
    """
    m_inv = (p2[0] - p1[0]) / float(p2[1] - p1[1])
    delta_y = (y - p1[1])
    x = p1[0] + m_inv * delta_y
    return [x, y]


def detectLines( image):
	# Read in the image and print some stats

	#region of interests 333
	corners = [(985, 468), (800, 582),(1200, 582)]

	
	# plt.imshow(cropped_image)
	plt.show()

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


	cropped_image = region_of_interest(edges, np.array([corners],np.int32))
	# plt.imshow(cropped_image, cmap='gray')
	# plt.show()

	return cropped_image
def estimateRoadCenter( image, raw_pic):
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

	 line_image = draw_lines(
							raw_pic,
										[[
										[left_x_start, max_y, left_x_end, min_y],
										[right_x_start, max_y, right_x_end, min_y],
										]],
							thickness=2,
							)


	 output = []
	 output.append(line_image)
	 output_lines = [[left_x_start, max_y],
					[left_x_end, min_y],
					[right_x_start, max_y],
					[right_x_end, min_y]]



	 vp_x = (left_x_end + right_x_end)/2
	 vp = [vp_x,min_y] #vanishing point
	#print vp
	 top = vp[1] + 35
	 bot = max_y
	 width = 500

	# Find point trapezoid the planar homoargy 
	 p1 = [vp[0] - width/2, top]
	 p2 = [vp[0] + width/2, top]
	 p3 = find_pt_inline(p2, vp, bot)
	 p4 = find_pt_inline(p1, vp, bot)
	 height, width = image.shape
	#print width,height


	#define source and destination points
	 src_pts = np.float32([p1, p2, p3, p4])# Source points
	 dst_pts = np.float32([[0, 0], [width, 0],
	 					[width ,height],
						[0, height]])
	
	
	#figure(1)
	#plt.imshow(warped)

	#draw Trapezoid
	 plt.figure(2)
	 cv2.polylines(line_image, [src_pts.astype(np.int32)],True, (0,200,100), thickness=3)
	 plt.plot(vp[0], vp[1], 'c^')
	 plt.plot(p1[0], p1[1], 'r+')
	 plt.plot(p2[0], p2[1], 'b+')
	 plt.plot(p3[0], p3[1], 'r^')
	 plt.plot(p4[0], p4[1], 'g^')
	 plt.imshow(	line_image)


     M = cv2.getPerspectiveTransform(src_pts, dst_pts)
	  warped = cv2.warpPerspective(line_image, M,(width,height))
	  output.append(warped)
	#figure(2)

	#plt.show()
	return output

def process_image(image):
    raw_image = cv2.imread(image,1)

    cropped_image = detectLines(raw_image)
    result_image = estimateRoadCenter(cropped_image, raw_image)
    
    return result_image[]    

if __name__ == '__main__':
    h = process_image('test2.png')
    plt.imshow(h)
    plt.show()
	