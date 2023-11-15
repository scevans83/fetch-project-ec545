import cv2
import numpy as np
from scipy.spatial import distance as dist


def order_points(pts):
    # sort points by x-coords
    xSorted = pts[np.argsort(pts[:, 0]), :]

    # get left-most and right-most points
    leftMost = xSorted[:2, :]
    rightMost = xSorted[2:, :]

    # sort left points by y-coords to get top-left and bottom-left
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, bl) = leftMost

    # use top-left as anchor to find bottom-right using euclidean distance
    D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    (br, tr) = rightMost[np.argsort(D)[::-1], :]

    # return coordinates in order: top-left, top-right, bottom-right, bottom-left
    return np.array([tl, tr, br, bl], dtype="float32")


# load image
image = cv2.imread("test1.jpg")

# convert to hsv for color detection
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# define the range of red color in HSV
# may change depending on the lighting conditions and the exact color red of the marker we end up using
# lower_red = np.array([153, 58, 116])
# upper_red = np.array([179, 150, 177])
lower_red = np.array([160, 100, 100])
upper_red = np.array([180, 255, 255])

# create mask for red colors
blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
mask = cv2.inRange(hsv, lower_red, upper_red)

# find contours in mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# filter contours by size
# 100 seems to be a decent threshold, can be changed if need be
filtered_contours = [c for c in contours if cv2.contourArea(c) > 100]

# check for contours of the 4 red dots
if len(filtered_contours) == 4:
    centers = []
    for contour in filtered_contours:
        # get center of each contour
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centers.append([cx, cy])

            # draw center for debugging
            cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)

    # sort corner points
    sorted_corners = order_points(np.array(centers, dtype="float32"))

    # draw corners on image to check order
    for i, corner in enumerate(sorted_corners):
        int_corner = (int(corner[0]), int(corner[1]))
        cv2.circle(image, int_corner, 5, (0, 255, 0), -1)
        cv2.putText(
            image, str(i), int_corner, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2
        )

    # cv2.imshow("Corner Order", image)
    # cv2.waitKey(0)

    # set new width and height
    width, height = 2400, 1800

    # set shift values
    shift_x, shift_y = 35, 35

    # set destination points
    destination_points = np.float32(
        [
            [0 + shift_x, 0 + shift_y],
            [width - 1 - shift_x, 0 + shift_y],
            [width - 1 - shift_x, height - 1 - shift_y],
            [0 + shift_x, height - 1 - shift_y],
        ]
    )

    # get transformation matrix
    M = cv2.getPerspectiveTransform(sorted_corners, destination_points)

    # warp image
    warped_image = cv2.warpPerspective(image, M, (width, height))

    # show the original and warped image
    # cv2.imshow("Original Image", image)
    # cv2.imshow("Warped Image", warped_image)

    # save warped image
    cv2.imwrite("bev_map.jpg", warped_image)
    cv2.destroyAllWindows()
else:
    print("rectangle not detected.")
