#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

markers = dict()

def callback(data:Image):
    
    ellipseBlackPixelThreshold = 0.5
    
    bridge = CvBridge()
    
    img = bridge.imgmsg_to_cv2(data)

    resImg = img.copy()
    width,height = img.shape

    # Convert the image to grayscale
    gray = img.copy()

    thresh_value = 200
    _,grayWhiter = cv2.threshold(gray, thresh_value, 100, cv2.THRESH_BINARY_INV)


    # Apply Gaussian blur to reduce noise
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection to find edges in the image
    edges = cv2.Canny(gray, 100, 200)

    # Find contours in the edge image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detectedEllipses = []
    id = 0
    # Loop over each contour and fit an ellipse
    for contour in contours:
        if contour.shape[0] >= 5:            
            ellipse = cv2.fitEllipse(contour)                
            cv2.ellipse(img, ellipse, (0, 255, 0), 2)        
            (x, y), (major_axis, minor_axis), angle = ellipse
            angle = np.deg2rad(angle)
            vertices = cv2.ellipse2Poly((int(x), int(y)), (int(major_axis / 2), int(minor_axis / 2)), int(round(angle / np.pi * 180)), 0, 360, 10)

            mask = np.zeros((width,height),np.uint8)
            white = np.full((width,height),255, np.uint8)

            cv2.fillConvexPoly(mask,vertices,(255,0,0))

            filledShape = cv2.bitwise_not(mask)
            masked = 255 - cv2.bitwise_or(grayWhiter, grayWhiter, mask=mask)
            _, masked = cv2.threshold(masked, thresh_value, 255, cv2.THRESH_BINARY)

            countFilled = width*height - cv2.countNonZero(filledShape)
            countMasked = width*height - cv2.countNonZero(masked)
            
            if countMasked/countFilled > ellipseBlackPixelThreshold:
                markers[id] = ellipse
                id +=1
                
    
    [print(marker) for marker in markers.items()]
        



pub = rospy.Publisher()
        
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("image", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()





















