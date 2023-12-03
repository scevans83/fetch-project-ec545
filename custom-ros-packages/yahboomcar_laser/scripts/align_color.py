import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from Rosmaster_Lib import Rosmaster

view_exploration = False

#here's our list of supported colors
#helpful source: https://www.rapidtables.com/web/color/RGB_Color.html
max_color_val = 255
supported_colors = {"red":        (0, 0, max_color_val),
                    "green":      (0, max_color_val, 0),
                    "blue":       (max_color_val, 0, 0),
                    # "white":      (max_color_val, max_color_val, max_color_val),
                    # "black":      (0, 0, 0),
                    # "yellow":     (0, max_color_val, max_color_val),
                    # "magenta":   (max_color_val, 0, max_color_val),
                    # "cyan":       (max_color_val, max_color_val, 0)
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

def simple_mask(img, mask_dim, threshold = 200):
    """
    idea: mask out anything that doesn't have a sufficiently high red pixel value. Then, use the distance coloring from red in LAB space
    """
    mask = np.zeros(np.shape(img)[0:2], dtype = np.uint8)

    mask = np.uint8(255*(img[:, :, mask_dim] > threshold))
    return mask

def bgr_mask(img, lower_thresholds, upper_thresholds):
    """
    allows us to do basic upper and lower bound thresholding mask on bgr values in an image.
    """
    condition0 = (img[:, :, 0] > lower_thresholds[0]) & (img[:, :, 0] < upper_thresholds[0])
    condition1 = (img[:, :, 1] > lower_thresholds[1]) & (img[:, :, 0] < upper_thresholds[1])
    condition2 = (img[:, :, 2] > lower_thresholds[2]) & (img[:, :, 0] < upper_thresholds[2])

    mask = np.uint8(255*(condition0 & condition1 & condition2))

    return mask

def find_contours(img, tree, approx, lower = 5000, upper = 500000):
    """
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray, (5,5), sigmaX=0, sigmaY=0)
    # _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    threshold = cv2.adaptiveThreshold(gray_blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    # canny_edges = cv2.Canny(image=gray_blur, threshold1=100, threshold2=200)
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

def predict_color(avg_value, threshold = 100):
    """
    """
  
    # distances = np.sqrt(np.sum((np.square(avg_value - lab_color_vals)), axis=0))
    distances = np.zeros(len(lab_color_vals))
    for idx in range(len(distances)):
        distances[idx] = np.sqrt(np.sum(np.square(avg_value - lab_color_vals[idx])))
    
    min_idx = np.argmin(distances)

    if distances[min_idx] > threshold:
        color = None
    else:
        color = color_names[min_idx]
    

  
    

    return color


def find_contour_colors(img, contours, threshold = 100):
    """
    """

    lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

    color_estimates = []
    for contour in contours:
        mean_color = get_average_value(lab_img, contour)
        color_estimates.append(predict_color(mean_color, threshold))

    return color_estimates

def find_contour_center(contour):
    """
    """
    M= cv2.moments(contour)
    cX = int((M["m10"] / M["m00"]))
    cY = int((M["m01"] / M["m00"]))

    return (cX, cY)

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

def find_contours_and_colors(img, lower_threshold, upper_threshold, lower = 5000, upper = 500000,):
    """
    """

    mask = bgr_mask(img, lower_threshold, upper_threshold)
    img = cv2.bitwise_and(img, img, mask = mask)
    contours = find_contours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE, lower=1500, upper=200000)
    color_list = find_contour_colors(img, contours, threshold=100)

    return contours, color_list


    # labeled_img = label_contours(img, contours, color=(0, max_color_val, 0))
    # label_contour_colors(labeled_img, contours, color_list, color=(max_color_val, max_color_val, max_color_val))




    
    




def exploration():

    #Goal: Look for objects of a certain color. When you detect them, drive a control input to move them towards the center of the image
    #Outline:
    #   - detect contours in the environment
    #   - classify the colors of each object
    #   - select the largest object of the color of interest
    #   - make a fake control input to drive it towards the center

    #Source 1: https://pyimagesearch.com/2016/02/08/opencv-shape-detection/
    # img = cv2.imread("sample.jpg", -1)
    
    contours = find_contours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #source 2: https://pyimagesearch.com/2016/02/15/determining-object-color-with-opencv/
    color_list = find_contour_colors(img, contours)


    labeled_img = label_contours(img, contours, color=(0, max_color_val, 0))
    label_contour_colors(labeled_img, contours, color_list, color=(max_color_val, max_color_val, max_color_val))
    cv2.imwrite("fully_labeled.jpg", labeled_img)

    













def main():

    #FOR LATER:
    #Outline of thoughts on how to explore/charge
    # - 

    car = Rosmaster()
    car.set_car_motion(0, 0, 0)

    frame = cv2.VideoCapture(0)


    while frame.isOpened():
        ret,img = frame.read()

        # mask = simple_mask(img, 2, threshold = 125) #red
        mask = bgr_mask(img, (0, 0, 175), (200, 200, 255))

        #Only the red simple mask works well enough with the red objects I have. We may want to filter on multiple channels (eg. blue > thresh1, red > thresh2, etc.)
        # mask = simple_mask(img, 1, threshold = 150) #green
        # mask = simple_mask(img, 1, threshold = 200) #blue


        img = cv2.bitwise_and(img, img, mask = mask)
        contours = find_contours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE, lower=1500, upper=200000)

        color_list = find_contour_colors(img, contours, threshold=100)


        labeled_img = label_contours(img, contours, color=(0, max_color_val, 0))
        label_contour_colors(labeled_img, contours, color_list, color=(max_color_val, max_color_val, max_color_val))

        cv2.imshow('labeled image',img)
        action = cv2.waitKey(20)
        if action == ord('q'):
            print("reached break")
            break


        #Do some angular adjustment to track the red contours
        num_frames = 5
        image_center_x = np.array(np.int32(np.shape(img)[0]/2))
        moving_average = np.ones(num_frames)*image_center_x

        color_of_interest = "red"
        # color_of_interest = "green"
        # color_of_interest = "blue"
        if color_of_interest in color_list:

            #find the max red contour
            max_color_idx = None
            max_color_area = -np.inf
            for idx in range(len(color_list)):
                if color_list[idx] == color_of_interest:
                    curr_area = cv2.contourArea(contours[idx])
                    if curr_area > max_color_area:
                        max_color_area = curr_area
                        max_color_idx = idx


            curr_offset = find_contour_center(contours[max_color_idx])[0]
            


            
        else:
            curr_offset = image_center_x

        #update velocity
        moving_average[1:-1] = moving_average[0:-2]
        moving_average[0] = curr_offset

        #find the center of the max red contour
        

        kp = 5 #currenlty is random.
        angular_velocity = kp*(image_center_x - np.mean(moving_average))/np.shape(img)[0]

        print("desired velocity: ", angular_velocity)
        max_velocity = 1.0
        angular_velocity = np.sign(angular_velocity) * min(np.abs(angular_velocity), max_velocity)

        car.set_car_motion(0, 0, angular_velocity)

    print("done")
    car.set_car_motion(0, 0, 0)
    frame.release()
    cv2.destroyAllWindows()

    # except Exception as error:
        
    #     print(error)

    #     print("exception caught")
    #     car.set_car_motion(0, 0, 0)
    #     frame.release()
    #     cv2.destroyAllWindows()


if __name__ == "__main__":
    
    if(view_exploration):
        exploration()
    else:
        main()
