import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import cv2
from moviepy.editor import VideoFileClip
from IPython.display import HTML
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
    point-slope formula in order to find a point that is present on the line
    that passes through our vanishing point (vp). 
    input: points p1, p2, and y. They come is as tuples [x, y]
    using the point-slope formula: y - b = m(x - a)
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
    corners = [(985, 465), (800, 582),(1200, 582)]

    
    # plt.imshow(cropped_image)
    plt.show()

    #add greyscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #add canny detection with guassian blur
    kernel_size = 11    
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
     

    low_threshold = 15
    high_threshold = 40
    edges = cv2.Canny(blur_gray, low_threshold, high_threshold)
    #plt.imshow(edges, cmap='gray')
    #plt.show()


    cropped_image = region_of_interest(edges, np.array([corners],np.int32))
    # plt.imshow(cropped_image, cmap='gray')
    # plt.show()

    return cropped_image

left_line_x_pre = []
left_line_y_pre = []
right_line_x_pre = []
right_line_y_pre = []

point_one = 0
point_two = 0
point_three = 0
point_four=0
def estimateRoadCenter( image, raw_pic):
    lines_image = cv2.HoughLinesP(    image,
                                rho=3,
                                theta=np.pi /180,
                                threshold=25,
                                lines=np.array([]),
                                minLineLength=5,
                                maxLineGap=100
                            )


    # for x in range(0, len(lines_image)):
    #     for x1,y1,x2,y2 in lines_image[x]:
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
    
    global right_line_x_pre
    global right_line_y_pre
    if len(right_line_x)==0 or len(right_line_y)==0:
        
        right_line_x = right_line_x_pre
        right_line_y = right_line_y_pre
    else:
        right_line_x_pre = right_line_x
        right_line_y_pre = right_line_y


    min_y = 500 # <-- Just below the horizon
    max_y = 610 # <-- The bottom of the image

   # print(right_line_x, right_line_y, left_line_x, left_line_y)

    poly_left = np.poly1d(np.polyfit(left_line_y,left_line_x, 1))

    left_x_start = int(poly_left(max_y))
    left_x_end = int(poly_left(min_y))

    poly_right = np.poly1d(np.polyfit(right_line_y,right_line_x, 1))

    right_x_start = int(poly_right(max_y))
    right_x_end = int(poly_right(min_y))

#    line_image = draw_lines(
#                        raw_pic,
#                                    [[
#                                    [left_x_start, max_y, left_x_end, min_y],
#                                    [right_x_start, max_y, right_x_end, min_y],
#                                    ]],
#                        thickness=2,
#                        )
    line_image = np.copy(raw_pic)

    output = []
    output.append(line_image)
    output_lines = [[left_x_start, max_y],
                [left_x_end, min_y],
                [right_x_start, max_y],
                [right_x_end, min_y]]



    vp_x = (left_x_end + right_x_end)/2
    vp = [988,460] #vanishing point
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


    min_y_lane = 515 # <-- Min lane polygon height end pt
    max_y_lane = 600 # <-- The bottom of the image
    
    point_1 = [left_x_start,max_y_lane]
    point_2 = [right_x_start, max_y_lane]
    point_3 = find_pt_inline([left_x_start,max_y], [left_x_end,min_y], min_y_lane)
    point_4 = find_pt_inline([right_x_start,max_y], [right_x_end,min_y], min_y_lane)
#    plt.plot(point_1[0], point_1[1], 'c^')
#    plt.plot(point_2[0],point_2[1], 'r+')
#    plt.plot(point_3[0], point_3[1], 'c^')
#    plt.plot(point_4[0],point_4[1], 'r+')
#    plt.imshow(line_image)
    
    #MAKE A LOW PASS FILTER- alpha determines the trust given to new data
    global point_one
    global point_two
    global point_three
    global point_four
    
    alpha = 0.9
    if (point_one== 0 or point_two ==0 or point_three==0 or point_four==0):
        point_one = point_1
        point_two = point_2
        point_three = point_3
        point_four = point_4
    
    point_one[0] = (1-alpha)*point_1[0]+alpha*point_one[0]
    point_two[0]= (1-alpha)*point_2[0]+alpha*point_two[0]
    point_three[0] = (1-alpha)*point_3[0]+alpha*point_three[0]
    point_four[0] = (1-alpha)*point_4[0] +alpha*point_four[0]
        
    lane_lpf= np.array([point_one,point_two,point_four,point_three])
    lane_raw = np.array([point_1,point_2,point_4,point_3])
    #draw Trapezoid
    #plt.figure(2)
    
    cv2.polylines(line_image, [lane_lpf.astype(np.int32)],True, (0,200,200), thickness=2)
    #cv2.polylines(line_image, [lane_raw.astype(np.int32)],True, (0,0,0), thickness=2)
    #cv2.polylines(line_image, [src_pts.astype(np.int32)],True, (0,200,100), thickness=3)

   #cv2.fillPoly(line_image, [lane_pts], (0,255,255))
    
    #plt.figure(1)
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(raw_pic, M,(width,height))

    warped_line = cv2.warpPerspective(line_image, M,(width,height))
    output.append(warped)
    output.append(warped_line)
    #figure(2)

    #plt.show()
    return output

def process_image(image):
    #raw_image = cv2.imread(image,1)
    cropped_image = detectLines(image)
    result_image = estimateRoadCenter(cropped_image, image)  
    return result_image[2] 
def process_image_lines(image):
    #raw_image = cv2.imread(image,1)
    cropped_image = detectLines(image)
    result_image = estimateRoadCenter(cropped_image, image)   
    return result_image[0]

if __name__ == '__main__':
    video = 'testing_lane_topview.mp4'
    clip1 = VideoFileClip("GOPR8538.mp4").subclip(70,90)
    white_clip = clip1.fl_image(process_image) #NOTE: this function expects color images!!
    white_clip.write_videofile(video, audio=False)
    
    video_2 = 'testing_lane_detection.mp4'
    black_clip = clip1.fl_image(process_image_lines)
    black_clip.write_videofile(video_2, audio=False)
    

  

