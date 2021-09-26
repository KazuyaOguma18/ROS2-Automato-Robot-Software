## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import numpy as np
import cv2
import matplotlib.pyplot as plt


def rs_depth_counter(depth_select ,k):

    d_p = np.ravel(depth_select)
        
    d_sort = np.sort(d_p)[::1]

    d_list = d_sort.tolist()
    
    count = []
    sum_depth = []
    sum_depth_indx = []
    counter = 0
    wide = 0
    i = 0
    
    up = 0
    sum_depth_counter = 0
    av = 0
    for_ave_sum = 0
    for n in range(len(d_list)):
        
        if d_list[n]- up < 1:
            counter += 1
            
        else:
            count.append(counter)
            counter = 0 
            wide += 1
            up += 1
            if count[up-1] > 100:
                sum_depth_counter += count[up-1]

            elif count[up-1] <= 100 and count[up-1] > 10:
                sum_depth_counter += count[up-1] 
                for_ave_sum += up-1
                av += 1

                
            else:
                sum_depth.append(sum_depth_counter)
                if not av == 0:
                    sum_depth_indx.append(int(for_ave_sum/av)) 
                else :
                    sum_depth_indx.append(up-1) 
                for_ave_sum = 0
                sum_depth_counter = 0
                av = 0


    if not count:
        pass
        
    if not sum_depth:
        sum_depth.append(0)
        sum_depth_indx.append(0)
    
    else:
        count[0] = 0
        #print(wide)

        #print(dist)
        
    x = range(wide-1)
    y = count

    return x, y, sum_depth_indx[sum_depth.index(max(sum_depth))]

    
        

        



