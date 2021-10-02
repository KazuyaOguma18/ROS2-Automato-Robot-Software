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


# In[20]:


with detection_graph.as_default():
  with tf.compat.v1.Session(graph=detection_graph) as sess:
    for image_path in TEST_IMAGE_PATHS:
      print("-------------------------------------------------------------------------------",image_path)  
      image = Image.open(image_path)
      # the array based representation of the image will be used later in order to prepare the
      # result image with boxes and labels on it.
      image_np = load_image_into_numpy_array(image)
      print(type(image_np))
      # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
      image_np_expanded = np.expand_dims(image_np, axis=0)
      image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
      # Each box represents a part of the image where a particular object was detected.
      boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
      # Each score represent how level of confidence for each of the objects.
      # Score is shown on the result image, together with the class label.
      scores = detection_graph.get_tensor_by_name('detection_scores:0')
      classes = detection_graph.get_tensor_by_name('detection_classes:0')
      num_detections = detection_graph.get_tensor_by_name('num_detections:0')
      # Actual detection.
      (boxes, scores, classes, num_detections) = sess.run(
          [boxes, scores, classes, num_detections],
          feed_dict={image_tensor: image_np_expanded})
      # Visualization of the results of a detection.
      vis_util.visualize_boxes_and_labels_on_image_array(
          image_np,
          np.squeeze(boxes),
          np.squeeze(classes).astype(np.int32),
          np.squeeze(scores),
          category_index,

          use_normalized_coordinates=True,
          min_score_thresh=.3,
          line_thickness=20)

      
      plt.figure(figsize=IMAGE_SIZE)
      #plt.tick_params(labelbottom="False",bottom="False")
      #plt.tick_params(labelleft="False",left="False")
      w, h = image.size  

      plt.xticks([0,1*w/10,2*w/10,3*w/10,4*w/10,5*w/10,6*w/10,7*w/10,8*w/10,9*w/10,w], [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0])
      plt.yticks([0,1*h/10,2*h/10,3*h/10,4*h/10,5*h/10,6*h/10,7*h/10,8*h/10,9*h/10,h], [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0])
      plt.box("False")
      plt.imshow(image_np)
      plt.cla()


# In[ ]:





# In[ ]:




