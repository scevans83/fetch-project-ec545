import cv2
import numpy as np

image_path = "sample.jpg"
environment_base = cv2.imread(image_path, -1)
enable_plots = False

def show_img_helper(image, title, enable):
    if not enable:
        return
    cv2.imshow(title, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

show_img_helper(environment_base, "starting image", enable_plots)

#problem outline:
#   - detect edges in the image
#   - construct grid as a grouping of pixels
#   - construct occupancy grid - if edge pass through pixel grouping, treat as occupied

#subproblem 1: detect edges in the image
#following tutorial here: https://learnopencv.com/edge-detection-using-opencv/

#this tutorial compares two algorithms - sobel edge detection and canny edge detection.
#I will implement both for comparison (going in, I have a bias towards Canny)

#preliminary steps - convert to grayscale (color doesn't matter) and do a gaussian blur
gray_environment = cv2.cvtColor(environment_base, cv2.COLOR_BGR2GRAY)
gray_blur = cv2.GaussianBlur(gray_environment, (5,5), sigmaX=0, sigmaY=0) #(3,3) indicates to use a 3x3 kernel for the convolutinal blurring

show_img_helper(gray_blur, "grayscale blurred image", enable_plots)


#sobel image detection (only doing composite xy detection)
sobelxy_environment = cv2.Sobel(src = gray_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5)

show_img_helper(sobelxy_environment, "sobel edge detection", enable_plots)

#this look terrible, lets hope canny is better

#canny
canny_edges = cv2.Canny(image=gray_blur, threshold1=100, threshold2=200)

show_img_helper(canny_edges, "Canny edge detection", enable_plots)

print(np.shape(canny_edges))
print(canny_edges)
print(np.max(canny_edges))

#Part 2: Break environment into groupings of pixels
