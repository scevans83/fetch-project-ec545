import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

view_exploration = False


def show_img_helper(image, title, enable):
    if not enable:
        return
    cv2.imshow(title, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def image_to_occpancy(image, grid_side_length_pixels, gaussian_blur_kernel = (5,5), canny_thresh_1 = 100, canny_thresh_2 = 200):
    
    #convert to gray scale and perform gaussian blurring
    gray_environment = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray_environment, gaussian_blur_kernel, sigmaX=0, sigmaY=0)

    #perform canny edge detection
    canny_edges = cv2.Canny(image=gray_blur, threshold1=canny_thresh_1, threshold2=canny_thresh_2)

    #create the baseline for the occupancy grid
    occupancy_shape = (np.int64(np.ceil(np.shape(canny_edges)[0]/grid_side_length_pixels)), np.int64(np.ceil(np.shape(canny_edges)[1]/grid_side_length_pixels)))
    occupancy_grid = np.zeros(occupancy_shape) 

    #iterate over each occupancy pixel
    #NOTE: Pretty sure there isn't a good way to vectorize this (likely not big enough to warrant it anyways). If performance is an issue, I imagine you could do a 2d convolution and remove excess elements?
    for row in range(occupancy_shape[0]):
        row_lower = row*grid_side_length_pixels
        row_upper = row_lower + grid_side_length_pixels - 1
        for col in range(occupancy_shape[1]):
            #calculate the bounds for this grid element
            col_lower = col*grid_side_length_pixels
            col_upper = col_lower + grid_side_length_pixels - 1 #NOTE: numpy naturally truncates if array indicies exceed, no need to add logic doing so


            sub_array = canny_edges[row_lower:row_upper+1, col_lower:col_upper+1]

            not_full = np.shape(sub_array) != (grid_side_length_pixels, grid_side_length_pixels)

            #if the grid element is not a full side_length x side_length element, or if any of the pixels have an edge in them, treat it as occupied
            if(not_full or np.any(sub_array > 0)):
                occupancy_grid[row, col] = 1 

    return occupancy_grid

#based on code found here: https://stackoverflow.com/questions/56614725/generate-grid-cells-occupancy-grid-color-cells-and-remove-xlabels
def plot_occupancy(map):
    
    #make a pleasing color map

    #TODO: define an enumerator for these - JPF 2023-11-09
    #TODO: plotting class to define these elsewhere
    FREE_SPACE = 0
    OCCUPIED_SPACE = 1
    ROBOT = 2
    BALL = 3
    GOAL = 4

    occupancy_color_map = colors.ListedColormap(["white", "black", "blue", "red", "yellow"])
    value_map = [FREE_SPACE, OCCUPIED_SPACE, ROBOT, BALL, GOAL]

    occupancy_color_normalization = colors.BoundaryNorm(value_map, occupancy_color_map.N)

    #using the minor ticks as the grid so we can display the major ones as integers
    plt.imshow(map, cmap=occupancy_color_map, norm=occupancy_color_normalization, aspect="equal")
    plt.grid(which="minor", axis="both", color = "k", linestyle = "-", linewidth = 1) #need to set the ticks for each major element to be shown
    ax = plt.gca()
    ax.set_xticks(np.arange(0.5, np.shape(map)[1], 1), minor = True) #x and y bounds are fliped from what is normal to plots like this. x = rows, y = cols
    ax.set_yticks(np.arange(0.5, np.shape(map)[0], 1), minor = True)
    ax.set_xticks(np.arange(0, np.shape(map)[1], 1), minor = False) #x and y bounds are fliped from what is normal to plots like this. x = rows, y = cols
    ax.set_yticks(np.arange(0, np.shape(map)[0], 1), minor = False)
    ax.tick_params(top=True, labeltop=True, bottom=False, labelbottom=False) #from here: https://matplotlib.org/stable/gallery/ticks/tick_xlabel_top.html
    # plt.tick_params(axis='both', which='both', bottom=False,   
    #                 left=False, labelbottom=False, labelleft=False)
    # plt.minorticks_off() 
    plt.show()


def exploration():

    image_path = "test1.jpg" #"test2_3d.jpg" #
    environment_base = cv2.imread(image_path, -1)
    enable_plots = True

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
    canny_edges = cv2.Canny(image=gray_blur, threshold1=50, threshold2=150)

    show_img_helper(canny_edges, "Canny edge detection", enable_plots)

    print(np.shape(canny_edges))
    print(canny_edges)
    print(np.max(canny_edges))

    #this is a lot better

    #Part 2: Break environment into groupings of pixels

    #take the image and break it a grid. Each grid element is a 50x50 pixel square (note: for now, pixels are truncated at the bounds of the photo, no special behavior implemented)
    side_length = 50
    occupancy_shape = (np.int64(np.ceil(np.shape(canny_edges)[0]/side_length)), np.int64(np.ceil(np.shape(canny_edges)[1]/side_length)))

    occupancy_grid = np.zeros(occupancy_shape) 
    print(np.shape(occupancy_grid))

    #iterate over each occupancy pixel
    #NOTE: Pretty sure there isn't a good way to vectorize this (likely not big enough to warrant it anyways). If performance is an issue, I imagine you could do a 2d convolution and remove excess elements?
    for row in range(occupancy_shape[0]):
        row_lower = row*side_length
        row_upper = row_lower + side_length - 1
        for col in range(occupancy_shape[1]):
            #calculate the bounds for this grid element
            col_lower = col*side_length
            col_upper = col_lower + side_length - 1 #NOTE: numpy naturally truncates if array indicies exceed, no need to add logic doing so


            sub_array = canny_edges[row_lower:row_upper+1, col_lower:col_upper+1]

            not_full = np.shape(sub_array) != (side_length, side_length)

            #if the grid element is not a full side_length x side_length element, or if any of the pixels have an edge in them, treat it as occupied
            if(not_full or np.any(sub_array > 0)):
                occupancy_grid[row, col] = 1 


    print(occupancy_grid)
    print(occupancy_grid[0:5, 0:3])


def main():
    
    #test 1: a basic environment consisting of whiteboard drawings only
    test1_path = "test1.jpg"
    test1_image = cv2.imread(test1_path, -1)
    test1_occupancy = image_to_occpancy(test1_image, 50, (5,5), canny_thresh_1 = 100, canny_thresh_2= 200)
    show_img_helper(test1_image, "Test 1 Base", True)
    plot_occupancy(test1_occupancy)

    #test 2: an environment consisting of some 3D elements and blackboard drawing
    test2_path = "test2_3d.jpg"
    test2_image = cv2.imread(test2_path, -1)
    test2_occupancy = image_to_occpancy(test2_image, 25, (5,5), canny_thresh_1 = 50, canny_thresh_2= 150)
    show_img_helper(test2_image, "Test 2 Base", True)
    plot_occupancy(test2_occupancy)


    #test 3: same as environment 2, but with less effort into lighting (therby reducing the lighting artifacts in theory)
    test3_path = "test3_3d.jpg"
    test3_image = cv2.imread(test3_path, -1)
    test3_occupancy = image_to_occpancy(test3_image, 75, (5,5), canny_thresh_1 = 50, canny_thresh_2= 150)
    show_img_helper(test3_image, "Test 3 Base", True)
    plot_occupancy(test3_occupancy)

    #comments: 
    #   - finnicking with the side length, kernel, and thresh parmeters can have impacts on how well the decomposition works
    #   - there is definitely the possibility of image errors causing "phantom obstacles." Local robot detection is recommended
    


#run the main function (streamlined results) or exploration (detailed step by step) depending on the global boolean when desired
if __name__ == "__main__":
    
    if(view_exploration):
        exploration()
    else:
        main()

