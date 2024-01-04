import cv2
import numpy as np
import math

H_LOWER = 63.0
S_LOWER = 8.0
V_LOWER = 5.0

H_UPPER = 92.0
S_UPPER = 255.0
V_UPPER = 255.0

# screen width (px) = 
SCREEN_WIDTH = 320
# screen height (px) 
SCREEN_HEIGHT = 240
# screen FOV x (deg) 
FOV_X = 59.6
# screen FOV y (deg)
FOV_Y = 49.7

# deg
MOUNTING_ANGLE = 4.5

AREA_SIZE_FILTER = 4
#in
TAPE_HEIGHT = 8

#in
PIXELS_UP = 50

#in
CAMERA_X_OFFSET = 8.875
CAMERA_Y_OFFSET = 11.375
CAMERA_HEIGHT = 19.0

MID_HEIGHT = 22.125
HIGH_HEIGHT = 41.875


# convert from pixels to angles
# screen width (px) = 320
# screen height (px) = 240
# screen FOV x (deg) = 59.6
# screen FOV y (deg) = 49.7

def px_to_deg(cx, cy):
    tx = ((cx - 160.0) / SCREEN_WIDTH) * FOV_X
    ty = ((cy - 120.0) / SCREEN_HEIGHT) * FOV_Y
    return tx, -ty

def draw_point(image, x, y):
    cv2.circle(image, (int(x), int(y)), 5, (0,0,255), cv2.FILLED)

def draw_center_line(image):
    cv2.line(image, (int(0), int(PIXELS_UP)), (int(SCREEN_WIDTH), int(PIXELS_UP)), (255,0, 0), 2, cv2.FILLED)

# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    # convert the input image to the HSV color space
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # convert the hsv to a binary image by removing any pixels
    # that do not fall within the following HSV Min/Max values
    img_threshold = cv2.inRange(img_hsv, (H_LOWER, S_LOWER, V_LOWER), (H_UPPER, S_UPPER, V_UPPER))
    # draw_center_line(image)

    # find contours in the new binary image
    contours, _ = cv2.findContours(img_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContour = np.array([[]])

    # initialize an empty array of values to send back to the robot
    llpython = [0,0,0]

    # if contours have been detected, draw them
    if len(contours) > 0:
        # cv2.drawContours(image, contours, -1, 255, 2)
        # record the largest contour

        
        # largestContour = max(contours, key=cv2.contourArea)

        points = []
    
        for i in range(len(contours)):
            if cv2.contourArea(contours[i]) < AREA_SIZE_FILTER: continue
            for coord in contours[i]:
                points.append(coord[0])

        distances = []
        for point in points:
            # draw_point(image, point[0], point[1])
            tx, ty = px_to_deg(point[0], point[1])
            # print(tx, ty)
            if (point[0] > PIXELS_UP):
                divisor = math.tan((ty + MOUNTING_ANGLE) * math.pi/ 180.0)
                if (divisor != 0):
                    dist_y = (MID_HEIGHT - CAMERA_HEIGHT) / math.tan((ty + MOUNTING_ANGLE) * math.pi/ 180.0)
                    dist_x = math.tan(tx *math.pi/ 180.0) * dist_y  + CAMERA_X_OFFSET
                    distances.append((dist_x, dist_y))
            else:
                divisor = math.tan((ty + MOUNTING_ANGLE) * math.pi/ 180.0)
                if (divisor != 0):
                    dist_y = (HIGH_HEIGHT - CAMERA_HEIGHT) / math.tan((ty + MOUNTING_ANGLE) * math.pi/ 180.0)
                    dist_x = math.tan(tx * math.pi/ 180.0) * dist_y + CAMERA_X_OFFSET
                    distances.append((dist_x, dist_y))
        
        least_dist = None
        # dist = None
        for distance in distances:
            if (least_dist == None):
                least_dist = distance[0]
                # dist = distance
            elif (abs(distance[0]) < abs(least_dist)):
                least_dist = distance[0]
                # dist = distance
        
        # record some custom data to send back to the robot
        if (least_dist != None):
            print(least_dist)
            # print(least_dist, dist[1])
            llpython = [1, least_dist, 0]

    #return the largest contour for the LL crosshair, the modified image, and custom robot data
    return largestContour, image, llpython