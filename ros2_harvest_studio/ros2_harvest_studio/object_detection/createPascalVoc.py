import numpy as np
import os
import sys
import tensorflow as tf
from PIL import Image
from xml.etree import ElementTree
from xml.dom import minidom
import collections


# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")

from object_detection.utils import label_map_util

from object_detection.utils import visualization_utils as vis_util

# What model to download.
MODEL_NAME = 'tomato_graph'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('training', 'object-detection.pbtxt')

NUM_CLASSES = 2

detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

def load_image_into_numpy_array(image):
  (im_width, im_height) = image.size
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)

# For the sake of simplicity we will use only 2 images:
# image1.jpg
# image2.jpg
# If you want to test the code with your images, just add path to the images to the TEST_IMAGE_PATHS.
import glob
PATH_TO_TEST_IMAGES_DIR = 'test_images'
# TEST_IMAGE_PATHS = [ os.path.join(PATH_TO_TEST_IMAGES_DIR, '{0:03d}.JPG'.format(i)) for i in range(0, 11) ]
TEST_IMAGE_PATHS = glob.glob('./test_images/*.JPG', recursive=True)

# Size, in inches, of the output images.
IMAGE_SIZE = (12, 8)

def get_box_classes_scores(boxes,
                           classes,
                           scores,
                           category_index,
                           keypoints=None,
                           max_boxes_to_draw=20,
                           min_score_thresh=.95):
    box_to_keypoints_map = collections.defaultdict(list)
    data = []
    if not max_boxes_to_draw:
        max_boxes_to_draw = boxes.shape[0]
    for i in range(min(max_boxes_to_draw, boxes.shape[0])):
        if scores is None or scores[i] > min_score_thresh:
            box = tuple(boxes[i].tolist())
            if keypoints is not None:
                box_to_keypoints_map[box].extend(keypoints[i])

            if classes[i] in category_index.keys():
                class_name = category_index[classes[i]]['name']
            else:
                class_name = 'N/A'
            ymin, xmin, ymax, xmax = box
            data.append([class_name, int(100 * scores[i]),ymin, xmin, ymax, xmax])
    return data

def create_pascalVOC(full_name, width,height, data, output_file_name):
    def prettify(elem):
        rough_string = ElementTree.tostring(elem, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ")

    top = ElementTree.Element('annotation')

    dir_name, file_name = os.path.split(full_name)

    folder = ElementTree.SubElement(top, 'folder')
    folder.text = str(dir_name)

    filename = ElementTree.SubElement(top, 'filename')
    filename.text = str(file_name)

    path = ElementTree.SubElement(top, 'path')
    path.text = str(full_name)

    source = ElementTree.SubElement(top, 'source')
    source.text = 'Unknown'
    # owner = ElementTree.SubElement(top, 'owner')

    size_s = ElementTree.SubElement(top, 'size')
    w = ElementTree.SubElement(size_s, 'width')
    w.text = str(width)
    h = ElementTree.SubElement(size_s, 'height')
    h.text = str(height)
    d = ElementTree.SubElement(size_s, 'depth')
    d.text = str(3)

    seg = ElementTree.SubElement(top, 'segmented')
    seg.text = str(0)

    for c in range(len(data)):
        object = ElementTree.SubElement(top, 'object')

        name = ElementTree.SubElement(object, 'name')
        name.text = str(data[c][0])

        pose = ElementTree.SubElement(object, 'pose')
        pose.text = 'Unspecified'

        truncated = ElementTree.SubElement(object, 'truncated')
        truncated.text = str(1)

        difficult = ElementTree.SubElement(object, 'difficult')
        difficult.text = str(0)

        bboxElm = ElementTree.SubElement(object, 'bndbox')
        xmin = ElementTree.SubElement(bboxElm, 'xmin')
        xmin.text = str(int(data[c][3]*width))
        ymin = ElementTree.SubElement(bboxElm, 'ymin')
        ymin.text = str(int(data[c][2]*height))
        xmax = ElementTree.SubElement(bboxElm, 'xmax')
        xmax.text = str(int(data[c][5]*width))
        ymax = ElementTree.SubElement(bboxElm, 'ymax')
        ymax.text = str(int(data[c][4]*height))

    elm = prettify(top)
    with open(output_file_name, 'w') as fp:
        fp.write(elm)

with detection_graph.as_default():
  with tf.Session(graph=detection_graph) as sess:
    for image_path in TEST_IMAGE_PATHS:
      print("-------------------------------------------------------------------------------",image_path)
      image = Image.open(image_path)
      # the array based representation of the image will be used later in order to prepare the
      # result image with boxes and labels on it.
      image_np = load_image_into_numpy_array(image)
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

      w, h = image.size
      data = get_box_classes_scores(
          np.squeeze(boxes),
          np.squeeze(classes).astype(np.int32),
          np.squeeze(scores),
          category_index,
          min_score_thresh=.3)

      OUTPUT_DIR = 'test_images'
      imagepath = os.path.split(image_path)
      filename = os.path.splitext(imagepath[1])[0]
      output_name = os.path.join(OUTPUT_DIR, (filename + '.xml'))

      create_pascalVOC(image_path, w,h,data, output_name)
