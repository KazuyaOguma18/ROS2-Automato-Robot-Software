## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
import pandas as pd
import collections


def rs_depth_counter(depth_select ,k):

    d_p = np.ravel(depth_select)
        
    d_sort = np.sort(d_p)[::1]

    d_list = d_sort.tolist()
    
    counter = []
    c = collections.Counter(d_list)

    for n in range(max(d_list)):
        counter.append(c[n])

    if not counter:
        pass
    
    else:
        counter[0] = 0
        #print(wide)

        #print(dist)
        
    x = range(max(d_list)-1)
    y = counter

    return x, y

    
        

        



