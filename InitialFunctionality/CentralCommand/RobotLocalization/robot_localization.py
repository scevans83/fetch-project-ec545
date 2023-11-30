import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import qrcode

view_exploration = False
live_webcam = True

#todo: create a centralized location for helper methods
def show_img_helper(image, title, enable):
    if not enable:
        return
    cv2.imshow(title, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def calc_dist(p1, p2):
    """
    """
    return np.sum(np.sqrt(np.square(p1 - p2)))

def contour_midpoint(contour):
    """
    """
    M = cv2.moments(contour) 
    if M['m00'] != 0.0: 
        x = int(M['m10']/M['m00']) 
        y = int(M['m01']/M['m00'])

    return np.array([x, y]) 


def calc_midpoint(p1, p2):

    midpoint = np.abs(p1 - p2)/2 + np.min(np.stack([p1, p2]), axis = 0)
    return midpoint


def qr_mid_point(qr_corners):
    """
    returns the midpoint of the provided QR corner points in x, y coordiantes (in units of pixels). Assumes provided a single array of corner points.
    """
    return calc_midpoint(qr_corners[0], qr_corners[2])


def find_contours(img, tree, approx, lower = 5000, upper = 500000):
    """
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray, (5,5), sigmaX=0, sigmaY=0)
    _, threshold = cv2.threshold(gray_blur, 127, 255, cv2.THRESH_BINARY)

    # show_img_helper(threshold, "test", True)
    contours, _ = cv2.findContours(threshold, tree, approx)

    filtered_contours = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < lower or area > upper:
            continue
        filtered_contours.append(contour)

    return filtered_contours

def find_triangles(contours, qr_corners = []):
    """

    """
    triangle_idxs = []
    for idx in range(1, len(contours)):

        approx = cv2.approxPolyDP(contours[idx], 0.1*cv2.arcLength(contours[idx], True), True)

        if(len(approx) == 3):
            
            #optionally, filter out values that are in the QR code
            #source: https://stackoverflow.com/questions/8508096/how-to-check-if-one-contour-is-nested-embedded-in-opencv
            #we skip the polynomial if any of its indicies are inside the QR code
            if(len(qr_corners) >0):
                
                in_poly = []
                for approx_idx in range(len(approx)):
                    in_poly.append(cv2.pointPolygonTest(qr_corners.astype(int)[0],  approx[approx_idx][0].tolist(), False))

                if(np.any(np.array(in_poly) > 0)):
                    continue

            triangle_idxs.append(idx)

    return triangle_idxs

def estimate_triangle(contours, triangle_idxs, qr_center):
    """
    """
    distances = np.full(len(triangle_idxs), np.inf)
    for idx in range(len(triangle_idxs)):

        distances[idx] = calc_dist(contour_midpoint(contours[triangle_idxs[idx]]), qr_center)
    
    return triangle_idxs[np.argmin(distances)]

def estimate_triangle2(contours, triangle_idxs):
    """
    """

    #maybe we do something more interesting with color later
    #https://pyimagesearch.com/2016/02/15/determining-object-color-with-opencv/
    return triangle_idxs[0]

def find_localization_aids(img, lower = 5000, upper = 500000 ):

    qr_corners = []
    triangle_vertices = []

    qr_detector = cv2.QRCodeDetector()

    ret_val, qr_corners = qr_detector.detect(img)

    #only do the rest if we saw a QR code
    if(ret_val):

        #find the qr midpoint, used to find best triangle guess
        qr_mid = contour_midpoint(qr_corners)

        #retrieve set of contours (filtered by area)
        contours = find_contours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE, lower=lower, upper=upper)
        
        #isolate the contours that look like a triangle
        triangle_idxs = find_triangles(contours, qr_corners)

        if(len(triangle_idxs) != 0):
            #best guess is triangle closed to the QR code (by center point)
            est_triangle = estimate_triangle(contours, triangle_idxs, qr_mid)

            #get the triangle vertices
            triangle_vertices = cv2.approxPolyDP(contours[est_triangle], 0.1*cv2.arcLength(contours[est_triangle], True), True)

    return qr_corners, triangle_vertices

def label_localization_aids(image, qr_corners, triangle_vertices, save_name):

    if(qr_corners is not None):
        qr_labeled = cv2.polylines(image, qr_corners.astype(int), True, (0, 0, 255), 5)
    else:
        qr_labeled = image

    if(len(triangle_vertices) > 0):
        both_labeled = cv2.drawContours(qr_labeled, [triangle_vertices], 0, (255, 0, 0), 5)
    else:
        both_labeled = qr_labeled

    if save_name != "":
        cv2.imwrite(save_name, both_labeled)

    return both_labeled


def find_localization_aids2(img, lower = 5000, upper = 500000 ):

    triangle_vertices = []

    #retrieve set of contours (filtered by area)
    contours = find_contours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE, lower=lower, upper=upper)
    
    #isolate the contours that look like a triangle
    triangle_idxs = find_triangles(contours)

    if(len(triangle_idxs) != 0):
        #best guess is triangle closed to the QR code (by center point)
        est_triangle = estimate_triangle2(contours, triangle_idxs)

        #get the triangle vertices
        triangle_vertices = cv2.approxPolyDP(contours[est_triangle], 0.1*cv2.arcLength(contours[est_triangle], True), True)

    return triangle_vertices

def label_localization_aids2(image, triangle_vertices, save_name):

    if(len(triangle_vertices) > 0):
        labeled = cv2.drawContours(image, [triangle_vertices], 0, (255, 0, 0), 5)
    else:
        labeled = image

    if save_name != "":
        cv2.imwrite(save_name, labeled)

    return labeled



def exploration():
    #problem: detect and localize robot

    #outline: 
    #   - Detect robot via QR code (?)
    #   - Detect triangle shape attached to robot (isoleces)
    #       - Use triangle to estimate orientation of the robot
    #           - TLDR: Perpendicular bisector is aligned with the forward direction
    #   - estimate robot position
    #       - idea: canny edge/contours, centroid of grouping
    #       - idea: qr code center or triangle centroid are aligned with robot center
    #   - update map with robot position (in grid)
    #       -cells with robot edges are treated as containing it
    #           - possible conflict if both in there huh. Leaving as an open problem for the time being
    #   


    #step 1: need to generate an image and print it for testing.

    #starting with a QR code.
    # source: https://note.nkmk.me/en/python-pillow-qrcode/ 
    qr_img = qrcode.make("Do I dream of electric sheep?")


    qr_img.save("test_qrcode.png")

    #I also need to make an isosceles  triangle

    #create a base image (numpy array)
    side_len = 1024
    base_img = np.empty([side_len, side_len, 3], dtype=np.uint8)
    base_img.fill(255)


    show_img_helper(base_img, "test white", False)

    #draw a polygon
    #recall: x is column, y is row
    pts = np.array([[side_len/2, side_len/8], [side_len/4, side_len/4 + side_len/2], [side_len/4 + side_len/2, side_len/4 + side_len/2]], dtype = np.int32)
    print(pts)

    color = (0, 0, 0) #black, may do red
    thickness = 5
    isClosed = True

    triangle_img = cv2.polylines(base_img,  [pts], isClosed, color, thickness)

    show_img_helper(triangle_img, "triangle?", False)

    cv2.imwrite("triangle.png", triangle_img)

    #approach 1: detect QR code, find center
    #sources:
    #   - https://docs.opencv.org/4.x/d7/d90/classcv_1_1GraphicalCodeDetector.html#ad28ae6b728add0ca8538ea0fc9d61732
    #   - https://note.nkmk.me/en/python-opencv-qrcode/

    localization_sample = cv2.imread("sample_localization.jpg")

    show_img_helper(localization_sample, "check", False)

    qr_detector = cv2.QRCodeDetector()

    ret_val, qr_corners = qr_detector.detect(localization_sample)
    print(qr_corners)

    #gonna draw a box to see if we are detecting correctly
    outlined_qr = cv2.polylines(localization_sample, qr_corners.astype(int), isClosed, (0, 0, 255), 5)
    cv2.imwrite("outlined_qr.jpg", outlined_qr)

    #use the edge length of the QR code to get a physical scale
    measured_edge = 2.409 #inches
    image_edge = np.sum(np.sqrt(np.square(qr_corners[0][0] - qr_corners[0][1])))
    inch_to_pix_ratio = measured_edge/image_edge
    print("image inch to pixel ratio: ", inch_to_pix_ratio)

    #find the midpoint, label it in the image
    qr_mid = qr_mid_point(qr_corners[0])
    qr_mid_2 = contour_midpoint(qr_corners)
    print("qr_mid:", qr_mid)
    print("qr_mid2:", qr_mid_2)
    qr_mid_labeled = cv2.circle(outlined_qr, qr_mid.astype(int), 10, (0, 0, 255), -1)
    cv2.imwrite("center_qr.jpg", qr_mid_labeled)

    print("midpoint from origin in inches", qr_mid*inch_to_pix_ratio)

    #TODO: https://www.geeksforgeeks.org/how-to-detect-shapes-in-images-in-python-using-opencv/

    #retrieve set of contours
    contours = find_contours(localization_sample, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    print(len(contours))

    triangle_idxs = find_triangles(contours)
    print(len(triangle_idxs))

    est_triangle = estimate_triangle(contours, triangle_idxs, qr_mid)

    # for idx in range(len(triangle_idxs)):
    #     cv2.drawContours(localization_sample, [contours[triangle_idxs[idx]]], 0, (255, 0, 0), 5)
    cv2.drawContours(localization_sample, [contours[est_triangle]], 0, (255, 0, 0), 5)
    # show_img_helper(localization_sample, "test", True)
    cv2.imwrite("find_triangle.jpg", localization_sample)

    # print(triangle_idxs)

    print(cv2.approxPolyDP(contours[est_triangle], 0.1*cv2.arcLength(contours[est_triangle], True), True))


def main():
    loc1 = cv2.imread("sample_localization.jpg")
    qr_corners1, triangle_vertices1 = find_localization_aids(loc1)
    label_localization_aids(loc1, qr_corners1, triangle_vertices1, "loc1_labeled.jpg")


    loc2 = cv2.imread("loc2.jpg")
    qr_corners2, triangle_vertices2 = find_localization_aids(loc2)
    label_localization_aids(loc2, qr_corners2, triangle_vertices2, "loc2_labeled.jpg")

    loc3 = cv2.imread("loc3.jpg")
    qr_corners3, triangle_vertices3 = find_localization_aids(loc3)
    label_localization_aids(loc3, qr_corners3, triangle_vertices3, "loc3_labeled.jpg")

if __name__ == "__main__":

    if(live_webcam):
        vid_capture = cv2.VideoCapture(1, cv2.CAP_DSHOW)

        while(True):
            ret, frame = vid_capture.read()
            if ret == True:
                triangle_vertices = find_localization_aids2(frame, lower=100, upper=100000)
                labeled = label_localization_aids2(frame, triangle_vertices, "")
                cv2.imshow('Frame', labeled)
                k_input = cv2.waitKey(20) #wait for 20 ms on each frame and monitor user input

                #in this case, if the user presses q, we quit
                if k_input == ord('q'):
                    print("early exit")
                    break
            else:
                break

            #when finished with a caputre, release it, and destroy the windows
        vid_capture.release()
        cv2.destroyAllWindows()
    else:
        if(view_exploration):
            exploration()
        else:
            main()

