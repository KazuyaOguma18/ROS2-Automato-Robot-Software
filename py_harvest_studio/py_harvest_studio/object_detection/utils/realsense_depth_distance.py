## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import math

class Intrinsics:
    def __init__(self):
        self.width = None
        self.height = None
        self.ppx = None
        self.ppy = None
        self.fx = None
        self.fy = None

def get_depth_at_pixel(depth_frame, pixel_x, pixel_y):
    return depth_frame.as_depth_frame().get_distance(round(pixel_x), round(pixel_y))


def convert_depth_pixel_to_metric_coordinate(depth, x_1, x_2, y_1, y_2, mode, intrinsics: Intrinsics):
    
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
    if intrinsics.width != None:
        hfov = 2*math.atan(intrinsics.width/(2*intrinsics.fx))
        vfov = 2*math.atan(intrinsics.height/(2*intrinsics.fy))

    if mode == "rs":
        theta_x_1 = math.atan(x_1*math.tan(hfov/2))
        theta_x_2 = math.atan(x_2*math.tan(hfov/2))
        theta_y_1 = math.atan(y_1*math.tan(vfov/2))
        theta_y_2 = math.atan(y_2*math.tan(vfov/2))
    elif mode == "azure":
        theta_x0 = math.atan(1/3* math.tan(hfov/2))
        
        theta_x_1 = math.atan(x_1*math.tan(theta_x0))
        theta_x_2 = math.atan(x_2*math.tan(theta_x0))
        theta_y_1 = math.atan(y_1*math.tan(vfov/2))
        theta_y_2 = math.atan(y_2*math.tan(vfov/2))
                
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
    # print(depth)
    
    return X, Y, Z, radius






