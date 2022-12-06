import cv2
import numpy as np
import math

def IMG2W(col, row, P):
    img_height = 480
    img_width = 640
    pix_over_world = 456
    col = float(col - img_width / 2)
    row = float(row - img_height / 2)
    world_coord = [row / pix_over_world, col / pix_over_world]
    #print("calculated world coordinate: ", world_coord)
    return world_coord

def blob_search(image_raw, color, P):
    #cv2.namedWindow("Camera View")
    #cv2.imshow("Camera View", image_raw)
    #cv2.waitKey(2)

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True

    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 43, 46])
    upper_red = np.array([10, 255, 255])
    green_lower = (50,60, 60)   
    green_upper = (70,255,255)

    if color == "red":
        lower = lower_red
        upper = upper_red
    else:
        lower = green_lower
        upper = green_upper


    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    keypoints = detector.detect(mask_image)
    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0, (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        #print("centroids: ", blob_image_center)
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1], P))
            #print(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))
        #print("")

    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)
    cv2.waitKey(2)
    return xw_yw