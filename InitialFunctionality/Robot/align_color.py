import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

view_exploration = True

#here's our list of supported colors
#helpful source: https://www.rapidtables.com/web/color/RGB_Color.html
supported_colors = {"red":        (0, 0, 255),
                    "green":      (0, 255, 0),
                    "blue":       (255, 0, 0),
                    "white":      (255, 255, 255),
                    "black":      (0, 0, 0),
                    "yellow":     (0, 255, 255),
                    "magenta":   (255, 0, 255),
                    "cyan":       (255, 255, 0)}

#convert the bgr data to lab color space
lab_color_vals = np.zeros((len(supported_colors), 1, 3))
color_names = []
for (idx, (color_name, bgr)) in enumerate(supported_colors.items()):
    lab_color_vals[idx] = bgr
    color_names.append(color_name)

lab_color_vals = cv2.cvtColor(lab_color_vals, cv2.COLOR_BGR2LAB)
    

def show_img_helper(image, title, enable):
    if not enable:
        return
    cv2.imshow(title, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def find_contours(img, tree, approx, lower = 5000, upper = 500000):
    """
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray, (5,5), sigmaX=0, sigmaY=0)
    _, threshold = cv2.threshold(gray_blur, 127, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(threshold, tree, approx)

    filtered_contours = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < lower or area > upper:
            continue
        filtered_contours.append(contour)

    return filtered_contours

def label_contours(img, contours, color = (0, 0, 255), thickness = 3):
    labeled_img = cv2.drawContours(img, contours, -1, color, thickness)
    return labeled_img

def find_colored_contours(contours, color):
    """
    """
    
    




def exploration():

    #Goal: Look for objects of a certain color. When you detect them, drive a control input to move them towards the center of the image
    #Outline:
    #   - detect contours in the environment
    #   - classify the colors of each object
    #   - select the largest object of the color of interest
    #   - make a fake control input to drive it towards the center

    #Source 1: https://pyimagesearch.com/2016/02/08/opencv-shape-detection/
    img = cv2.imread("sample.jpg")
    contours = find_contours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    labeled_img = label_contours(img, contours, color=(0, 255, 0))
    cv2.imwrite("contours_traced.jpg", labeled_img)

    #source 2: https://pyimagesearch.com/2016/02/15/determining-object-color-with-opencv/









def main():
    pass


if __name__ == "__main__":
    
    if(view_exploration):
        exploration()
    else:
        main()
