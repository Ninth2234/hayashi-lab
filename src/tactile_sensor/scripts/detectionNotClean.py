import cv2
import numpy as np
from time import sleep

imagePath = "TactileSensor/Detection/"
imageFileName = "captureImage1.jpg"

ellipseBlackPixelThreshold = 0.5

# Load an image
img = cv2.imread(imagePath+imageFileName)
resImg = img.copy()
width,height,_ = img.shape

# Convert the image to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

thresh_value = 200
_,grayWhiter = cv2.threshold(gray, thresh_value, 100, cv2.THRESH_BINARY_INV)


# Apply Gaussian blur to reduce noise
gray = cv2.GaussianBlur(gray, (5, 5), 0)

# Apply Canny edge detection to find edges in the image
edges = cv2.Canny(gray, 100, 200)

# Find contours in the edge image
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

detectedEllipses = []

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
        cv2.imshow("masked", masked)
        cv2.imshow("filledShape",filledShape)

        countFilled = width*height - cv2.countNonZero(filledShape)
        countMasked = width*height - cv2.countNonZero(masked)
        print(countMasked/countFilled)
        if countMasked/countFilled > ellipseBlackPixelThreshold:
            detectedEllipses.append(ellipse)

print(len(detectedEllipses))
for ellipse in detectedEllipses:
    print(ellipse)
    cv2.ellipse(resImg, ellipse, (0, 255, 0), 2)        

# Display the image with detected ellipses
cv2.imshow('Image with detected ellipses', resImg)
cv2.waitKey(0)
cv2.destroyAllWindows()
