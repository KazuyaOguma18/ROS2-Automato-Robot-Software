## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import math

def get_depth_at_pixel(depth_frame, pixel_x, pixel_y):
    return depth_frame.as_depth_frame().get_distance(round(pixel_x), round(pixel_y))

def convert_depth_pixel_to_metric_coordinate(depth, pixel_x, pixel_y, x_1, x_2, y_1, y_2, sum_depth):
    theta_x_1 = math.atan(x_1*math.tan(math.radians(69.4/2)))
    theta_x_2 = math.atan(x_2*math.tan(math.radians(69.4/2)))
    theta_y_1 = math.atan(y_1*math.tan(math.radians(42.5/2)))
    theta_y_2 = math.atan(y_2*math.tan(math.radians(69.4/2)))
    theta_x = math.atan(pixel_x*math.tan(math.radians(69.4/2)))
    theta_y = math.atan(pixel_y*math.tan(math.radians(42.5/2)))
    X_1 = int(depth* math.tan(theta_x_1) )
    X_2 = int(depth* math.tan(theta_x_2) )
    Y_1 = int(depth* math.tan(theta_y_1) )
    Y_2 = int(depth* math.tan(theta_y_2) )

    if abs(X_2 - X_1) > abs(Y_2 - Y_1):
        radius = abs((X_2 - X_1)/2)
    else:
        radius = abs((Y_2 - Y_1)/2)
    X = int(depth + radius* math.tan(theta_x))
    Y = int((-1)* (depth + radius* math.tan(theta_y)))


    '''
    if sum_depth > int(depth + (X_2-X_1)/2):
        Z = sum_depth
        print("SUMyes:%d"%(sum_depth))

    else:
    '''
    Z = int(depth + radius)
    print(depth)
    
    return X, Y, Z, radius





    