#!/usr/bin/env python

from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Initialize
bridge = CvBridge()
kernel = np.ones((30, 30), np.uint8)

def image_callback(msg):
    # Convert ROS image message to OpenCV image
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    nframe = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray', gray)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    edges = cv2.Canny(frame, 100, 200)
    cv2.imshow('canny image', edges)
    cv2.waitKey(0) # Wait for keypress to continue
    cv2.destroyAllWindows()

    ret, binary = cv2.threshold(gray, 100, 155, cv2.THRESH_OTSU)
    cv2.imshow('Binary image', binary)
    cv2.waitKey(0) # Wait for keypress to continue
    cv2.destroyAllWindows()

    inverted_binary = ~binary
    cv2.imshow('Inverted binary image', inverted_binary)
    cv2.waitKey(0) # Wait for keypress to continue
    cv2.destroyAllWindows()

    contours, hierarchy = cv2.findContours(edges,
    cv2.RETR_TREE,
    cv2.CHAIN_APPROX_SIMPLE)
    
    with_contours = cv2.drawContours(frame, contours, -1,(255,0,255),3)
    cv2.imshow('Detected contours', with_contours)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print('Total number of contours detected: ' + str(len(contours)))
    first_contour = cv2.drawContours(nframe, contours, 0,(255,0,255),3)
    cv2.imshow('First detected contour', first_contour)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    x, y, w, h = cv2.boundingRect(contours[0])
    cv2.rectangle(first_contour,(x,y), (x+w,y+h), (255,0,0), 5)
    cv2.imshow('First contour with bounding box', first_contour)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # Draw a bounding box around all contours
    for i, c in enumerate(contours):
    
        # Calculate the area of each contour
        area = cv2.contourArea(c)
        
        # Ignore contours that are too small or too large
        if area < 500 or 100000 < area:
            continue
    
    # cv2.minAreaRect returns:
        # (center(x, y), (width, height), angle of rotation) = cv2.minAreaRect(c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
    
    # Retrieve the key parameters of the rotated bounding box
        center = (int(rect[0][0]),int(rect[0][1])) 
        width = int(rect[1][0])
        height = int(rect[1][1])
        angle = int(rect[2])
    
        
        if width < height:
            angle = 90 - angle
        else:
            angle = -angle
                
        label = "  Rotation Angle: " + str(angle) + " degrees"
        textbox = cv2.rectangle(frame, (center[0]-35, center[1]-25), 
            (center[0] + 295, center[1] + 10), (255,255,255), -1)
        cv2.putText(frame, label, (center[0]-50, center[1]), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 1, cv2.LINE_AA)
        cv2.drawContours(frame,[box],0,(0,0,255),2)
        
    cv2.imshow('All contours with bounding box', with_contours)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
def main():
    rospy.init_node('object_detection_node', anonymous=True)
    rospy.Subscriber("/camera1/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
































#     # Apply background subtraction
#     fg_mask = back_sub.apply(frame)
#     cv2.imshow('Foreground Mask', fg_mask)
#     cv2.waitKey(1)

#     # Apply morphology operations
#     fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)
#     cv2.imshow('Morphology', fg_mask)
#     cv2.waitKey(1)

#     # Apply median blur
#     fg_mask = cv2.medianBlur(fg_mask, 1)
#     cv2.imshow('Median Blur', fg_mask)
#     cv2.waitKey(1)

#     # Apply thresholding
#     _, fg_mask = cv2.threshold(fg_mask, 30, 255, cv2.THRESH_BINARY)
#     cv2.imshow('Threshold', fg_mask)
#     cv2.waitKey(1)

#     # Find contours
#     contours, hierarchy = cv2.findContours(fg_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
#     areas = [cv2.contourArea(c) for c in contours]

#     if len(areas) < 1:
#         cv2.imshow('capture', frame)
#         key = cv2.waitKey(1)
#         if key & 0xFF == ord('q'):
#             return

#     # Find the largest moving object in the image
#     max_index = np.argmax(areas)

#     # Draw the bounding box
#     cnt = contours[max_index]
#     x, y, w, h = cv2.boundingRect(cnt)
#     cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

#     # Draw circle in the center of the bounding box
#     x2 = x + int(w / 2)
#     y2 = y + int(h / 2)
#     cv2.circle(frame, (x2, y2), 4, (0, 255, 0), -1)

#     # Print the centroid coordinates (we'll use the center of the bounding box) on the image
#     text = "x: " + str(x2) + ", y: " + str(y2)
#     cv2.putText(frame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#     # Display the resulting frame
#     cv2.imshow('Frame', frame)
#     key = cv2.waitKey(1)
#     if key & 0xFF == ord('q'):
#         return


