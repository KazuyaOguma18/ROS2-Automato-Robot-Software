#!/usr/bin/env python
# coding: utf-8

# In[12]:


import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import cv2
import time
import pyrealsense2 as rs

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image


# In[13]:


# This is needed to display the images.


# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")


# In[14]:


from utils import label_map_util

from utils import visualization_utils as vis_util

from utils import realsense_color as rs_color


# In[15]:


# What model to download.
MODEL_NAME = 'tomato_graph'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('training', 'object-detection.pbtxt')

NUM_CLASSES = 2


# In[16]:


detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.compat.v1.GraphDef()
  with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')


# In[17]:


label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)


# In[18]:


def load_image_into_numpy_array(image):
  (im_width, im_height) = image.size
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)



# In[19]:


# For the sake of simplicity we will use only 2 images:
# image1.jpg
# image2.jpg
# If you want to test the code with your images, just add path to the images to the TEST_IMAGE_PATHS.
import glob
PATH_TO_TEST_IMAGES_DIR = 'test_images'
#TEST_IMAGE_PATHS = [ os.path.join(PATH_TO_TEST_IMAGES_DIR, '{0:03d}.JPG'.format(i)) for i in range(0, 11) ]
TEST_IMAGE_PATHS = glob.glob('./test_images/*.JPG', recursive=True)

# Size, in inches, of the output images.
IMAGE_SIZE = (12, 8)
TEST_IMAGE_PATHS

def scale_to_width(img, width):
    scale = width / img.shape[1]
    return cv2.resize(img, dsize=None, fx=scale, fy=scale)



# In[20]:
fig = plt.figure(figsize=(1080,720))

width_color = 640
height_color = 480
width_depth = 640
height_depth = 480

width_range = 200
height_range = 200


pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, width_depth, height_depth, rs.format.z16, 30)
config.enable_stream(rs.stream.color, width_color, height_color, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Alignオブジェクト生成
align_to = rs.stream.color
align = rs.align(align_to)
depth = np.zeros((width_depth, height_depth))

  
with detection_graph.as_default():
  with tf.compat.v1.Session(graph=detection_graph) as sess:
    while True:
      t1 = time.time()
      print('-----------------------------------------------------------------------')
      # Wait for a coherent pair of frames: depth and color
      frames = pipeline.wait_for_frames()
      aligned_frames = align.process(frames)
      depth_frame = aligned_frames.get_depth_frame()
      color_frame = aligned_frames.get_color_frame()
      if not depth_frame or not color_frame:
        continue  
       

      # Convert images to numpy arrays
      depth_image = np.asanyarray(depth_frame.get_data())
      color_image = np.asanyarray(color_frame.get_data())
      depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
      t3 = time.time()
      color_image = scale_to_width(color_image, 200)
      depth_image = scale_to_width(depth_image, 200)
      image = Image.fromarray(np.uint8(color_image))
      
      # the array based representation of the image will be used later in order to prepare the
      # result image with boxes and labels on it.
      image_np = color_image
      # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
      
      
      image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
      # Each box represents a part of the image where a particular object was detected.
      boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
      # Each score represent how level of confidence for each of the objects.
      # Score is shown on the result image, together with the class label.
      scores = detection_graph.get_tensor_by_name('detection_scores:0')
      classes = detection_graph.get_tensor_by_name('detection_classes:0')
      num_detections = detection_graph.get_tensor_by_name('num_detections:0')
      
      image_np_expanded = np.expand_dims(image_np, axis=0)     
      # Actual detection.
      
      (boxes, scores, classes, num_detections) = sess.run(
          [boxes, scores, classes, num_detections],
          feed_dict={image_tensor: image_np_expanded})
      
      # Visualization of the results of a detection.
      print('-----------------------------------------------------------------------')
      vis_util.visualize_boxes_and_labels_on_image_array(
         image_np,
          np.squeeze(boxes),
          np.squeeze(classes).astype(np.int32),
          np.squeeze(scores),
          category_index, 
          use_normalized_coordinates=True,
          min_score_thresh=.3,
          line_thickness=5)
          
       
      #image_np = np.asarray(image_np)
      t2 = time.time() 
      cv2.putText(image_np, 'FPS : '+str(round(1/(t2-t1), 2)), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), thickness=2)
      cv2.imshow('depth2',depth_colormap)
      cv2.imshow('tomato',image_np)
      cv2.waitKey(1)
      

      print('time : '+str(t3-t1))
    

# Stop streaming
pipeline.stop()
      
  
      


# In[ ]:





# In[ ]:




