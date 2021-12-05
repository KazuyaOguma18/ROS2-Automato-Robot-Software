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


def convert_depth_pixel_to_metric_coordinate(depth, x_1, x_2, y_1, y_2, sum_depth, mode):
    
    '''
   if intrinsics:
        result1 = rs.rs2_deproject_pixel_to_point(intrinsics, [x_1, y_1], depth)
        result2 = rs.rs2_deproject_pixel_to_point(intrinsics, [x_2, y_1], depth)
        result3 = rs.rs2_deproject_pixel_to_point(intrinsics, [x_1, y_2], depth)
        result4 = rs.rs2_deproject_pixel_to_point(intrinsics, [x_2, y_2], depth)

        X_1 = (result1[0] + result3[0]) / 2
        X_2 = (result2[0] + result4[0]) / 2
        Y_1 = (result1[1] + result2[1]) / 2
        Y_2 = (result3[1] + result4[1]) / 2
    '''


    if mode == "rs":
        theta_x_1 = math.atan(x_1*math.tan(math.radians(69.4/2)))
        theta_x_2 = math.atan(x_2*math.tan(math.radians(69.4/2)))
        theta_y_1 = math.atan(y_1*math.tan(math.radians(42.5/2)))
        theta_y_2 = math.atan(y_2*math.tan(math.radians(42.5/2)))
    elif mode == "azure":
        theta_x_1 = math.atan(x_1*math.tan(math.radians(90/2)))
        theta_x_2 = math.atan(x_2*math.tan(math.radians(90/2)))
        theta_y_1 = math.atan(y_1*math.tan(math.radians(74.3/2)))
        theta_y_2 = math.atan(y_2*math.tan(math.radians(74.3/2)))
                
    X_1 = int(depth* math.tan(theta_x_1) )
    X_2 = int(depth* math.tan(theta_x_2) )
    Y_1 = int(depth* math.tan(theta_y_1) )
    Y_2 = int(depth* math.tan(theta_y_2) )

    if abs(X_2 - X_1) > abs(Y_2 - Y_1):
        radius = abs((X_2 - X_1)/2)
    else:
        radius = abs((Y_2 - Y_1)/2)

    '''  
    X = int(depth + radius* math.tan(theta_x))
    Y = int((-1)* (depth + radius* math.tan(theta_y)))
    '''
    X = int((X_1 + X_2) / 2)
    Y = int((-1)*((Y_1 + Y_2) / 2))

    
    Z = int(depth + radius)
    print(depth)
    
    return X, Y, Z, radius






