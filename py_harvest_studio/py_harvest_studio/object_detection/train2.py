# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'train.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!
import sys
import cv2
import numpy as np
import os
import six.moves.urllib as urllib
import tarfile
import tensorflow as tf
import zipfile
import time
import pyrealsense2 as rs
import math
from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image


sys.path.append("..")

from utils import label_map_util

from utils import visualization_utils as vis_util

from utils import realsense_color as rs_color

from utils import realsense_depth_count as rs_depth

from utils import image_division as imdiv

from utils import realsense_depth_distance as rs_dis

from PIL import Image
from PIL.ImageQt import ImageQt
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QSlider, 
    QLabel, QApplication)
from PyQt5.QtCore import Qt, QThread
from PyQt5.QtGui import QPixmap, QImage, QPalette
from PyQt5.QtWidgets import QWidget, QLabel, QApplication
from PyQt5 import QtCore, QtGui, QtWidgets
from tomato_depth_realtime9out import tomato



class Ui_Dialog(object):
    def setupUi(self, Dialog):
        self.img = cv2.imread("golira.jpg")
        self.img = cv2.resize(self.img, dsize = (1280, 720))
        self.height ,self.width = self.img.shape[:2]
        Dialog.setObjectName("Dialog")
        Dialog.resize(1820, 1080)
        self.zoom_label = QtWidgets.QLabel(Dialog)
        self.zoom_label.setGeometry(QtCore.QRect(1730, 460, 31, 71))
        self.zoom_label.setObjectName("zoom_label")

        #slider
        self.horizontalSlider = QtWidgets.QSlider(Dialog)
        self.horizontalSlider.setGeometry(QtCore.QRect(1440, 480, 261, 41))
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.horizontalSlider.setEnabled(False)
        self.horizontalSlider.setFocusPolicy(Qt.NoFocus)
        self.horizontalSlider.valueChanged[int].connect(self.changeValue)
        

        #image
        img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
        self.pixmap = QImage(img, self.width, self.height, QImage.Format_RGB888)
        self.for_image_label = QtWidgets.QLabel(Dialog)
        self.for_image_label.setGeometry(QtCore.QRect(20, 10, 1280, 720))
        self.for_image_label.setObjectName("for_image_label")
        self.for_image_label.clear()
        


        self.textBrowser = QtWidgets.QTextBrowser(Dialog)
        self.textBrowser.setGeometry(QtCore.QRect(1410, 400, 101, 41))
        self.textBrowser.setObjectName("textBrowser")
        self.tomatolist = QtWidgets.QColumnView(Dialog)
        self.tomatolist.setGeometry(QtCore.QRect(30, 760, 800, 350))
        self.tomatolist.setObjectName("tomatolist")
        self.textBrowser_2 = QtWidgets.QTextBrowser(Dialog)
        self.textBrowser_2.setGeometry(QtCore.QRect(1410, 40, 101, 41))
        self.textBrowser_2.setObjectName("textBrowser_2")
        self.lcdNumber = QtWidgets.QLCDNumber(Dialog)
        self.lcdNumber.setGeometry(QtCore.QRect(1480, 680, 181, 111))
        self.lcdNumber.setObjectName("lcdNumber")
        self.textBrowser_3 = QtWidgets.QTextBrowser(Dialog)
        self.textBrowser_3.setGeometry(QtCore.QRect(1410, 610, 101, 41))
        self.textBrowser_3.setObjectName("textBrowser_3")
        self.toolButton = QtWidgets.QToolButton(Dialog)
        self.toolButton.setGeometry(QtCore.QRect(1430, 120, 111, 24))
        self.toolButton.setObjectName("toolButton")
        self.toolButton_2 = QtWidgets.QToolButton(Dialog)
        self.toolButton_2.setGeometry(QtCore.QRect(1600, 120, 111, 24))
        self.toolButton_2.setObjectName("toolButton_2")
        self.textBrowser_4 = QtWidgets.QTextBrowser(Dialog)
        self.textBrowser_4.setGeometry(QtCore.QRect(1410, 220, 101, 41))
        self.textBrowser_4.setObjectName("textBrowser_4")

        #start
        self.toolButton_3 = QtWidgets.QToolButton(Dialog)
        self.toolButton_3.setGeometry(QtCore.QRect(1430, 300, 222, 24))
        self.toolButton_3.setObjectName("toolButton_3")
        self.toolButton_3.clicked.connect(self.start)
        self.toolButton_3.setEnabled(True)
        self.toolButton_3.setVisible(True)

        #stop
        self.toolButton_4 = QtWidgets.QToolButton(Dialog)
        self.toolButton_4.setGeometry(QtCore.QRect(1430, 300, 222, 24))
        self.toolButton_4.setObjectName("toolButton_4")
        self.toolButton_4.clicked.connect(self.stop)
        self.toolButton_4.setEnabled(False)
        self.toolButton_4.setVisible(False)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)
        
        self.walker = Walker()


    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.zoom_label.setText(_translate("Dialog", "0"))
        #self.for_image_label.setText(_translate("Dialog", "TextLabel"))
        self.textBrowser.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">ズーム</p></body></html>"))
        self.textBrowser_2.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">モード選択</p></body></html>"))
        self.textBrowser_3.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">FPS</p></body></html>"))
        self.toolButton.setText(_translate("Dialog", "zoom mode"))
        self.toolButton_2.setText(_translate("Dialog", "split mode"))
        self.textBrowser_4.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">実行</p></body></html>"))
        self.toolButton_3.setText(_translate("Dialog", "start"))
        self.toolButton_4.setText(_translate("Dialog", "stop"))

    def changeValue(self, value):
        self.zoom_label.setText(str(value))
        scale = 1 + value/10
        img_resize = cv2.resize(self.img, dsize=None, fx=scale, fy=scale).copy()
        #img_resize = cv2.resize(self.img, dsize=(self.width, self.height), fx=1.1, fy=1.1).copy()
        reh, rew = img_resize.shape[:2]
        changed_image = img_resize[int((reh - self.height)/2) : int((reh + self.height)/2) , int((rew - self.width)/2) : int((rew + self.width)/2)].copy()
        changed_image = cv2.cvtColor(changed_image, cv2.COLOR_BGR2RGB)
        self.pixmap = QImage(changed_image, self.width, self.height, QImage.Format_RGB888)
        self.for_image_label.clear()
        self.for_image_label.setPixmap(QPixmap.fromImage(self.pixmap))


        


    def stop(self):
        self.toolButton_4.setEnabled(False)
        self.toolButton_4.setVisible(False)
        self.toolButton_3.setEnabled(True)
        self.toolButton_3.setVisible(True)
        self.horizontalSlider.setEnabled(False)
        self.for_image_label.clear()
        self.status = False
        


    def start(self):
        self.toolButton_3.setEnabled(False)
        self.toolButton_3.setVisible(False)
        self.toolButton_4.setEnabled(True)
        self.toolButton_4.setVisible(True)
        self.horizontalSlider.setEnabled(True)
        self.for_image_label.setPixmap(QPixmap.fromImage(self.pixmap)) 
        self.status = True
        self.run()


    def image_show(self, image):
        height, width = image.shape[:2]
        self.pixmap = QImage(image, width, height, QImage.Format_RGB888)
        self.for_image_label.clear()
        self.for_image_label.setPixmap(QPixmap.fromImage(self.pixmap))

    def run(self):
        self.img = cv2.imread("golira.jpg")
        self.img = cv2.resize(self.img, dsize = (1280, 720))
        self.height ,self.width = self.img.shape[:2]
        # What model to download.
        MODEL_NAME = 'tomato_graph'

        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

        # List of the strings that is used to add correct label for each box.
        PATH_TO_LABELS = os.path.join('training', 'object-detection.pbtxt')

        NUM_CLASSES = 2




        detection_graph = tf.Graph()
        with detection_graph.as_default():
          od_graph_def = tf.compat.v1.GraphDef()
          with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
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


        #realsenseの解像度指定
        width_color = 1280
        height_color = 720
        width_depth = 1280
        height_depth = 720



        #ストリーミングの設定

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, width_depth, height_depth, rs.format.z16, 30)
        other_stream, other_format = rs.stream.color, rs.format.rgb8
        config.enable_stream(other_stream, width_color, height_color, other_format, 30)

        # Start streaming
        pipeline.start(config)

        # Alignオブジェクト生成
        align_to = rs.stream.color
        align = rs.align(align_to)
        depth = np.zeros((width_depth, height_depth))

        self.ui_dialog = Ui_Dialog()
        length = 0

        with detection_graph.as_default():
            with tf.compat.v1.Session(graph=detection_graph) as sess:
                while self.status:
                    self.toolButton_4.clicked.connect(self.stop) 
                
                    t1 = time.time()


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
                    depth_colormap = depth_image
                    #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)



                    depth_select = []
                    max_depth = []



                    #image = Image.fromarray(np.uint8(color_image))
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
                    t3 = time.time()
                    (boxes, scores, classes, num_detections) = sess.run(
                        [boxes, scores, classes, num_detections],
                        feed_dict={image_tensor: image_np_expanded})
                    t4 = time.time()
                    # Visualization of the results of a detection.
                    print('-----------------------------------------------------------------------')
                    boxes, names_show = vis_util.visualize_boxes_and_labels_on_image_array(
                       image_np,
                        np.squeeze(boxes),
                        np.squeeze(classes).astype(np.int32),
                        np.squeeze(scores),
                        category_index, 
                        use_normalized_coordinates=True,
                        min_score_thresh=.6,
                        line_thickness=5)



                    #color画像内の検出されたトマトデータからdepth画像を切り出す
                    length = len(boxes)
                    for k in range(length):
                        depth_select.append(depth_image[int(boxes[k][0]*height_depth):int(boxes[k][2]*height_depth), int(boxes[k][1]*width_depth):int(boxes[k][3]*width_depth)])


                    #depth画像から切り出したトマトのデータから一番depthが小さい点を算出

                    x = list(range(length))
                    y = list(range(length))
                    sum_depth = list(range(length))
                    for k in range(length):
                        if names_show[k] == 'tomato':
                            x[k] ,y[k] ,sum_depth[k] = rs_depth.rs_depth_counter(depth_select[k], k)
                            if not y[k]:
                                pass

                            else:
                                y[k] = np.diff(y[k])
                                dist = depth_frame.get_distance(int((boxes[k][1] + (boxes[k][3]-boxes[k][1])/2)*width_color), int((boxes[k][0] + (boxes[k][2]-boxes[k][0])/2)*height_color))
                                _y = list(y[k])
                                if dist >= _y.index(max(_y)):
                                    max_depth.append(dist)
                                else:
                                
                                    max_depth.append(_y.index(max(_y)))
                        else:
                            continue  

                    #x軸とy軸をピクセル単位からメートル単位へ変換
                    i = 0
                    for k in range(length):  
                        if names_show[k] == 'tomato':
                            if len(max_depth) > i:
                                cv2.drawMarker(image_np, (int((boxes[k][1] + (boxes[k][3]-boxes[k][1])/2)*width_color), int((boxes[k][0] + (boxes[k][2]-boxes[k][0])/2)*height_color)), color=(0, 0, 0),markerType=cv2.MARKER_CROSS, thickness=2)
                                pixel_x = (boxes[k][1] + (boxes[k][3]-boxes[k][1])/2 - 0.5)*2
                                pixel_y = (boxes[k][0] + (boxes[k][2]-boxes[k][0])/2 - 0.5)*2
                                X_meter, Y_meter, Z_meter = rs_dis.convert_depth_pixel_to_metric_coordinate(max_depth[i], pixel_x, pixel_y, boxes[k][1]-0.5, boxes[k][3]-0.5 , sum_depth[k])
                                cv2.putText(image_np, '(%d,%d,%d)'%(X_meter,Y_meter,Z_meter), (int((boxes[k][1] + (boxes[k][3]-boxes[k][1])/2)*width_color-120), int((boxes[k][0] + (boxes[k][2]-boxes[k][0])/2)*height_color)), cv2.FONT_ITALIC, 1.0, (255, 255, 255), thickness=2)
                                i = i + 1
                            else:
                                continue
                        else:
                            continue

                    t2 = time.time()

                    #可視化

                    cv2.putText(image_np, 'FPS : '+str(round(1/(t2-t1), 2)), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), thickness=2)
                    image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
                    cv2.line(image_np,(int(width_color/2), 0),(int(width_color/2), int(height_color)),color=(255, 255,0),thickness=2)
                    cv2.line(image_np,(0, int(height_color/2)),(int(width_color), int(height_color/2)),color=(255, 255,0),thickness=2)
                    image_show(image_np)

                    print('time : '+str(t4-t3))


            # Stop streaming
        pipeline.stop()
        exit()