import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

view_exploration = True

#here's our list of supported colors
#helpful source: https://www.rapidtables.com/web/color/RGB_Color.html
max_color_val = 255
supported_colors = {"red":        (0, 0, max_color_val),
                    "green":      (0, max_color_val, 0),
                    "blue":       (max_color_val, 0, 0),
                    "white":      (max_color_val, max_color_val, max_color_val),
                    "black":      (0, 0, 0),
                    "yellow":     (0, max_color_val, max_color_val),
                    "magenta":   (max_color_val, 0, max_color_val),
                    "cyan":       (max_color_val, max_color_val, 0)
                    }

#convert the bgr data to lab color space
lab_color_vals = np.zeros((len(supported_colors), 1, 3), dtype=np.uint8)
color_names = []
for (idx, (color_name, bgr)) in enumerate(supported_colors.items()):
    lab_color_vals[idx] = bgr
    color_names.append(color_name)


lab_color_vals = cv2.cvtColor(lab_color_vals, cv2.COLOR_BGR2LAB)
# lab_color_vals[0] = np.array([29, 52.48, 22.23])
# print(lab_color_vals)   

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

def label_contours(img, contours, color = (0, 0, max_color_val), thickness = 3):
    labeled_img = cv2.drawContours(img, contours, -1, color, thickness)
    return labeled_img

def get_average_value(lab_img, contour):
    """
    """
    mask = np.zeros(lab_img.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [contour], 0, 255, thickness=-1)
    mask = cv2.erode(mask, None, iterations=2)
    mean = np.array(cv2.mean(lab_img, mask = mask)[:3])

    return mean

def predict_color(avg_value, dist_threshold = 70):
    """
    """
  
    # distances = np.sqrt(np.sum((np.square(avg_value - lab_color_vals)), axis=0))
    distances = np.zeros(len(lab_color_vals))
    for idx in range(len(distances)):
        distances[idx] = np.sqrt(np.sum(np.square(avg_value - lab_color_vals[idx])))
    
    min_idx = np.argmin(distances)

    #Minimum distance threshold for color ID
    #TODO: may want to have a relative threshold - i.e. only classifier a color if it is significantly closer to said color than any other color
    if distances[min_idx] > dist_threshold:
        color = None
    else:
        color = color_names[min_idx]
    

  
    

    return color


def find_contour_colors(img, contours):
    """
    """

    lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

    color_estimates = []
    for contour in contours:
        mean_color = get_average_value(lab_img, contour)
        color_estimates.append(predict_color(mean_color))

    return color_estimates

def label_contour_colors(img, contours, color_estimates, color = (0, 0, 0)):
    """
    """

    #find the center of each contour
    for idx in range(len(contours)):
        contour = contours[idx]
        M = cv2.moments(contour)
        cX = int((M["m10"] / M["m00"]))
        cY = int((M["m01"] / M["m00"]))

        msg = " contour" + str(idx)
        if color_estimates[idx] is not None:
            msg = color_estimates[idx] + msg

        cv2.putText(img, msg, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

def find_contours_and_colors(img, lower = 5000, upper = 500000):
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
    img = cv2.imread("sample.jpg", -1)
    contours = find_contours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #source 2: https://pyimagesearch.com/2016/02/15/determining-object-color-with-opencv/
    color_list = find_contour_colors(img, contours)


    labeled_img = label_contours(img, contours, color=(0, max_color_val, 0))
    label_contour_colors(labeled_img, contours, color_list, color=(max_color_val, max_color_val, max_color_val))
    cv2.imwrite("fully_labeled.jpg", labeled_img)

    













def main():
    pass


if __name__ == "__main__":
    
    if(view_exploration):
        exploration()
    else:
        main()
