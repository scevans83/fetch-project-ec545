import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import qrcode

view_exploration = True

#todo: create a centralized location for helper methods
def show_img_helper(image, title, enable):
    if not enable:
        return
    cv2.imshow(title, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def calc_midpoint(p1, p2):

    midpoint = np.abs(p1 - p2)/2 + np.min(np.stack([p1, p2]), axis = 0)
    return midpoint


def qr_mid_point(qr_corners):
    """
    returns the midpoint of the provided QR corner points in x, y coordiantes (in units of pixels). Assumes provided a single array of corner points.
    """
    return calc_midpoint(qr_corners[0], qr_corners[2])


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
    qr_mid_labeled = cv2.circle(outlined_qr, qr_mid.astype(int), 10, (0, 0, 255), -1)
    cv2.imwrite("center_qr.jpg", qr_mid_labeled)

    print("midpoint from origin in inches", qr_mid*inch_to_pix_ratio)

    #TODO: https://www.geeksforgeeks.org/how-to-detect-shapes-in-images-in-python-using-opencv/
    

    






def main():
    pass

if __name__ == "__main__":
    if(view_exploration):
        exploration()
    else:
        main()

