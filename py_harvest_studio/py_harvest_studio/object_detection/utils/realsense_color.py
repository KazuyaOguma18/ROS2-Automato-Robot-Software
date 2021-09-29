## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################


import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time

def color():
    # Configure depth and color streams

    width_color = 1280
    height_color = 720

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width_color, height_color, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    # Alignオブジェクト生成
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    try:
        for i in range(15):
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            if not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())


        


    finally:

        # Stop streaming
        pipeline.stop()


    return color_image




