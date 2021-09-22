## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt


def rs_depth_counter(depth_select ,k):

    d_p = np.ravel(depth_select)
        
    d_sort = np.sort(d_p)[::1]

    d_list = d_sort.tolist()
    
    count = []
    counter = 0
    wide = 0
    i = 0
    
    up = 0
    for n in range(len(d_list)):
        
        if d_list[n]- up < 1:
            counter += 1
            
        else:
            count.append(counter)
            counter = 0 
            wide += 1
            up += 1

    count[0] = 0
        #print(wide)

        #print(dist)
        
    x = range(wide)
    y = count

    return x, y

    
        

        



