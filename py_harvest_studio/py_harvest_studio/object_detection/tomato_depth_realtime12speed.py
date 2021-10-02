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
import random


sys.path.append("..")

from utils import label_map_util

from utils import visualization_utils as vis_util

from utils import realsense_color as rs_color

from utils import realsense_depth_count as rs_depth

from utils import image_division as imdiv

from utils import realsense_depth_distance as rs_dis

from utils import focuspoint

from PIL import Image
from PIL.ImageQt import ImageQt
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QSlider, 
    QLabel, QApplication, QTableView, QHeaderView)
from PyQt5.QtCore import Qt, QThread, QObject, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage, QPalette, QStandardItemModel, QStandardItem
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from tomato_depth_realtime9out import tomato

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

class Walker(QObject):

    sig_status = QtCore.pyqtSignal(object)
    loop_finished = QtCore.pyqtSignal()
    fps_signal = QtCore.pyqtSignal(float)
    table_signal = QtCore.pyqtSignal(object)
    scale_signal = QtCore.pyqtSignal(float)

    def __init__(self, parent=None):
        QObject.__init__(self, parent=parent)
        self.path = ""
        self.stopped = True
        self.mutex = QtCore.QMutex()

    def setup(self, path):
        self.path = path
        self.stopped = False

    def stop(self):
        with QtCore.QMutexLocker(self.mutex):
            self.stopped = False
            print("end")

    def restart(self):
        with QtCore.QMutexLocker(self.mutex):
            self.stopped = False

    def stopping(self):
        print("stopping")

    def slider(self,value):
        self.value = value
        print("yes,slider %d" %value)

    def run(self):
        self.img = cv2.imread("golira.jpg")
        self.img = cv2.resize(self.img, dsize = (1280, 720))
        self.height ,self.width = self.img.shape[:2]
        self.value = 0
        self.auto = False
        self.automode = False
        first = False
        self.search = False
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
        TDbox = []
        quantity = []
        X_c = 0
        Y_c = 0
        t6 = 0
        t5 = 0

        with detection_graph.as_default():
            with tf.compat.v1.Session(graph=detection_graph) as sess:
                while self.stopped:
                    t1 = time.time()
                    if self.automode == True and length == 0:
                        self.auto = True
                    if self.auto == True:
                        if first == False:
                            t5 = time.time()
                            self.value = 0.5
                            first = True
                            quantity = []
                            valuezoom = []
                            random_line = []
                            self.search = True
                            value_max = False
                            X_c = -1
                            Y_c = -1

                        elif self.value != 1.0:
                            self.value += 0.5
                            value_max = False
                        elif self.value == 1.0:
                            self.value = 0.5
                            X_c += 1
                            value_max = True
                        if X_c == 2:
                            Y_c += 1
                            X_c = -1
                        #self.value = 0.5 elif self.value == 1.5

                        if X_c == 1 and Y_c == 1 and value_max == True:
                            self.auto = False
                        random_line.append([X_c, Y_c])
                        valuezoom.append(self.value)
                        if self.auto == False:
                            first = False
                            if max(quantity) != 0:
                                self.value = valuezoom[quantity.index(max(quantity))]
                                X_c, Y_c = random_line[quantity.index(max(quantity))]
                                self.search = False
                                t6 = time.time()
                            else:
                                self.auto = True
                    print("detect time:"+str(t6-t5))
                    # Wait for a coherent pair of frames: depth and color
                    frames = pipeline.wait_for_frames()
                    aligned_frames = align.process(frames)
                    depth_frame = aligned_frames.get_depth_frame()
                    color_frame = aligned_frames.get_color_frame()
                    if not depth_frame or not color_frame:
                        continue

                    # Convert images to numpy arrays
                    if self.search == False:
                        depth_image = np.asanyarray(depth_frame.get_data())
                        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                    color_image = np.asanyarray(color_frame.get_data())

                    #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)



                    depth_select = []
                    max_depth = []



                    #image = Image.fromarray(np.uint8(color_image))
                    # the array based representation of the image will be used later in order to prepare the
                    # result image with boxes and labels on it.




                    image_np , scale = self.ui_dialog.calculate_image(color_image, self.value, X_c, Y_c)
                    if self.search == False:
                        depth_image , scale = self.ui_dialog.calculate_image(depth_image, self.value, X_c, Y_c)


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


                    length = len(boxes)
                    if self.search == False:
                        #color画像内の検出されたトマトデータからdepth画像を切り出す
                        
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
                                        #max_depth.append(dist)
                                        pass
                                    else:
                                        max_depth.append(_y.index(max(_y)))
                            else:
                                continue

                        #x軸とy軸をピクセル単位からメートル単位へ変換
                        TDbox = []
                        i = 0
                        for k in range(length):
                            if names_show[k] == 'tomato':
                                if len(max_depth) > i:
                                    cv2.drawMarker(image_np, (int((boxes[k][1] + (boxes[k][3]-boxes[k][1])/2)*width_color), int((boxes[k][0] + (boxes[k][2]-boxes[k][0])/2)*height_color)), color=(0, 0, 0),markerType=cv2.MARKER_CROSS, thickness=2)
                                    pixel_x ,pixel_y, pixel_x_left, pixel_x_right = focuspoint.calculate_FocusPoint(k ,boxes ,X_c, Y_c ,scale)
                                    X_meter, Y_meter, Z_meter = rs_dis.convert_depth_pixel_to_metric_coordinate(max_depth[i], pixel_x, pixel_y, pixel_x_left, pixel_x_right , sum_depth[k])
                                    TDbox.append([names_show[k], str(X_meter), str(Y_meter), str(Z_meter)])
                                    cv2.putText(image_np, '(%d,%d,%d)'%(X_meter,Y_meter,Z_meter), (int((boxes[k][1] + (boxes[k][3]-boxes[k][1])/2)*width_color-120), int((boxes[k][0] + (boxes[k][2]-boxes[k][0])/2)*height_color)), cv2.FONT_ITALIC, 1.0, (255, 255, 255), thickness=2)
                                    i = i + 1
                                else:
                                    continue
                            else:
                                TDbox.append([names_show[k], 0, 0, 0])
                                continue

                    t2 = time.time()

                    #可視化

                    #cv2.putText(image_np, 'FPS : '+str(round(1/(t2-t1), 2)), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), thickness=2)
                    image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

                    if TDbox:
                        self.table_signal.emit(TDbox)

                    if boxes:
                        quantity.append(len(boxes))
                    else:
                        quantity.append(0)

                    cv2.imshow('depth',depth_colormap)
                    self.scale_signal.emit(scale)
                    self.sig_status.emit(image_np)
                    self.fps_signal.emit(1/(t2-t1))
                    print('time : '+str(t4-t3))
                    



            # Stop streaming
        pipeline.stop()
        self.loop_finished.emit()

    def automatic(self):
        self.auto = True
        self.automode = True

    def manual(self):
        self.automode = False



class Ui_Dialog(QWidget):
    stop_signal = pyqtSignal()
    slider_signal = pyqtSignal(int)

    def setupUi(self, Dialog):
        #######################################################
        """
        Just for test example
        
        if len(sys.argv) >= 2:
            ip = sys.argv[1]
        else:
            try:
                from configparser import ConfigParser
                parser = ConfigParser()
                parser.read('../robot.conf')
                ip = parser.get('xArm', 'ip')
            except:
                ip = input('Please input the xArm ip address:')
                if not ip:
                    print('input error, exit')
                    sys.exit(1)

       
        ########################################################
        self.arm = XArmAPI(ip, is_radian=True)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        
        self.arm.reset(wait=True)
        """
        self.value = 0
        self.restart = False
        self.img = cv2.imread("golira.jpg")
        self.img = cv2.resize(self.img, dsize = (1280, 720))
        self.height ,self.width = self.img.shape[:2]
        Dialog.setObjectName("Dialog")
        Dialog.resize(1820, 1080)
        self.zoom_label = QtWidgets.QLabel(Dialog)
        self.zoom_label.setGeometry(QtCore.QRect(1730, 460, 31, 71))
        self.zoom_label.setObjectName("zoom_label")
        self.zoom_label.setVisible(False)

        #slider
        self.horizontalSlider = QtWidgets.QSlider(Dialog)
        self.horizontalSlider.setGeometry(QtCore.QRect(1440, 480, 261, 41))
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.horizontalSlider.setEnabled(False)
        self.horizontalSlider.setVisible(False)
        self.horizontalSlider.setFocusPolicy(Qt.NoFocus)
        self.horizontalSlider.valueChanged[int].connect(self.changeValue)
        

        #image
        img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
        self.golira = QImage(img, self.width, self.height, QImage.Format_RGB888)
        self.for_image_label = QtWidgets.QLabel(Dialog)
        self.for_image_label.setGeometry(QtCore.QRect(20, 10, 1280, 720))
        self.for_image_label.setObjectName("for_image_label")
        self.for_image_label.clear()
        


        self.zoom = QtWidgets.QTextBrowser(Dialog)
        self.zoom.setGeometry(QtCore.QRect(1410, 400, 101, 41))
        self.zoom.setObjectName("zoom")

        #table tomato
        self.tomatolist = QtWidgets.QTableWidget(Dialog)
        self.tomatolist.setGeometry(QtCore.QRect(30, 760, 800, 280))
        self.tomatolist.setColumnCount(4)
        self.tomatolist.setRowCount(0)
        self.tomatolist.setObjectName("tomatolist")
        self.horheaders = ['name', 'X', 'Y', 'Z']
        self.tomatolist.setHorizontalHeaderLabels(self.horheaders)
        header = self.tomatolist.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)

        self.selectMode = QtWidgets.QTextBrowser(Dialog)
        self.selectMode.setGeometry(QtCore.QRect(1410, 40, 101, 41))
        self.selectMode.setObjectName("selectMode")

        #fps display
        self.lcdNumber = QtWidgets.QLCDNumber(Dialog)
        self.lcdNumber.setGeometry(QtCore.QRect(1480, 680, 181, 111))
        self.lcdNumber.setObjectName("lcdNumber")

        self.writeFPS = QtWidgets.QTextBrowser(Dialog)
        self.writeFPS.setGeometry(QtCore.QRect(1410, 610, 101, 41))
        self.writeFPS.setObjectName("writeFPS")

        #scale display
        self.scaleNumber = QtWidgets.QLCDNumber(Dialog)
        self.scaleNumber.setGeometry(QtCore.QRect(1000, 850, 181, 111))
        self.scaleNumber.setObjectName("current_scale")

        self.writeScale = QtWidgets.QTextBrowser(Dialog)
        self.writeScale.setGeometry(QtCore.QRect(950, 780, 101, 41))
        self.writeScale.setObjectName("writescale")

        #select mode
        self.zoommodeButton = QtWidgets.QToolButton(Dialog)
        self.zoommodeButton.setGeometry(QtCore.QRect(1430, 120, 111, 24))
        self.zoommodeButton.setObjectName("zoommodeButton")
        self.zoommodeButton.clicked.connect(self.zoommode)
        self.splitmodeButton = QtWidgets.QToolButton(Dialog)
        self.splitmodeButton.setGeometry(QtCore.QRect(1600, 120, 111, 24))
        self.splitmodeButton.setObjectName("zoommodeButton_2")

        #textbox run
        self.writerun = QtWidgets.QTextBrowser(Dialog)
        self.writerun.setGeometry(QtCore.QRect(1410, 220, 101, 41))
        self.writerun.setObjectName("writerun")

        #start
        self.startButton = QtWidgets.QToolButton(Dialog)
        self.startButton.setGeometry(QtCore.QRect(1430, 300, 222, 50))
        self.startButton.setObjectName("startButton")
        self.startButton.clicked.connect(self.start)
        self.startButton.setEnabled(False)
        self.startButton.setVisible(False)

        #stop
        self.stopButton = QtWidgets.QToolButton(Dialog)
        self.stopButton.setGeometry(QtCore.QRect(1430, 300, 222, 50))
        self.stopButton.setObjectName("stopButton")
        self.stopButton.clicked.connect(self.stop)
        self.stopButton.setEnabled(False)
        self.stopButton.setVisible(False)

        #automatic or manual
        self.combox = QtWidgets.QComboBox(Dialog)
        self.combox.setGeometry(1430, 920, 222, 24)
        self.combox.addItem("manual")
        self.combox.addItem("automatic")
        self.combox.setEnabled(False)
        self.combox.activated[str].connect(self.activation)
        self.writecombox = QtWidgets.QTextBrowser(Dialog)
        self.writecombox.setGeometry(QtCore.QRect(1410, 850, 150, 41))
        self.writecombox.setObjectName("writecombox")

        self.walker = Walker()
        self.thread = QThread()
        self.walker.moveToThread(self.thread)

        self.thread.started.connect(self.walker.run) # when thread starts, start worker
        self.thread.finished.connect(self.walker.stopping) # when thread finishes, stop worker

        self.startButton.clicked.connect(self.start)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

        self.walker.sig_status.connect(self.image_show)
        self.walker.fps_signal.connect(self.fps)
        #self.walker.loop_finished.connect(self.walker.stopping)
        self.walker.table_signal.connect(self.tomato_table)
        self.walker.scale_signal.connect(self.scaler)


    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.zoom_label.setText(_translate("Dialog", "0"))
        #self.for_image_label.setText(_translate("Dialog", "TextLabel"))
        self.zoom.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">ズーム</p></body></html>"))
        self.selectMode.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">モード選択</p></body></html>"))
        self.writeFPS.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">FPS</p></body></html>"))
        self.writeScale.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Scale</p></body></html>"))
        self.zoommodeButton.setText(_translate("Dialog", "zoom mode"))
        self.splitmodeButton.setText(_translate("Dialog", "split mode"))
        self.writerun.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">実行</p></body></html>"))
        self.startButton.setText(_translate("Dialog", "start"))
        self.stopButton.setText(_translate("Dialog", "stop"))
        self.writecombox.setText(_translate("Dialog", "automatically detect?"))

    def changeValue(self, value):
        self.zoom_label.setText(str(value))
        self.value = value
        self.walker.slider(value)


        


    def stop(self):
        self.stopButton.setEnabled(False)
        self.stopButton.setVisible(False)
        self.startButton.setEnabled(True)
        self.startButton.setVisible(True)
        self.horizontalSlider.setEnabled(False)
        self.restart = True
        self.walker.stop()
        self.thread.quit()
        self.for_image_label.clear()



    def start(self):
        self.for_image_label.setPixmap(QPixmap.fromImage(self.golira)) 
        if self.restart == True:
            print("connect")
            self.walker = Walker()
            self.thread = QThread()
            self.walker.moveToThread(self.thread)
            self.thread.started.connect(self.walker.run)
            self.walker.sig_status.connect(self.image_show)
            self.walker.fps_signal.connect(self.fps)
            #self.walker.loop_finished.connect(self.walker.stopping)
            self.walker.table_signal.connect(self.tomato_table)
            self.walker.scale_signal.connect(self.scaler)
    
        self.startButton.setEnabled(False)
        self.startButton.setVisible(False)
        self.stopButton.setEnabled(True)
        self.stopButton.setVisible(True)
        self.horizontalSlider.setEnabled(True)
        self.zoom_label.setVisible(True)
        self.combox.setEnabled(True)
        #self.arm.reset(wait=True)
        self.restart = False
        self.thread.start()


    def image_show(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.pixmap = QImage(image, self.width, self.height, QImage.Format_RGB888)
        self.for_image_label.clear()
        self.for_image_label.setPixmap(QPixmap.fromImage(self.pixmap))

    def ajust_image_center(self, image1, image2, value):
        height, width = image1.shape[:2]
        scale = 1 + value/20
        img_resize1 = cv2.resize(image1, dsize=None, fx=scale, fy=scale).copy()
        img_resize2 = cv2.resize(image2, dsize=None, fx=scale, fy=scale).copy()    
        reh, rew = img_resize1.shape[:2]
        changed_image1 = img_resize1[int((reh - height)/2) : int((reh + height)/2) , int((rew - width)/2) : int((rew + width)/2)].copy()
        changed_image2 = img_resize2[int((reh - height)/2) : int((reh + height)/2) , int((rew - width)/2) : int((rew + width)/2)].copy()
        return changed_image1, changed_image2, scale

    def ajust_image_leftup(self, image1, image2, value):
        height, width = image1.shape[:2]
        scale = 1 + value/20
        img_resize1 = cv2.resize(image1, dsize=None, fx=scale, fy=scale).copy()
        img_resize2 = cv2.resize(image2, dsize=None, fx=scale, fy=scale).copy()    
        reh, rew = img_resize1.shape[:2]
        changed_image1 = img_resize1[0 : height, 0 : width].copy()
        changed_image2 = img_resize2[0 : height, 0 : width].copy()
        return changed_image1, changed_image2, scale

    def calculate_image(self, image1, value, X_c, Y_c):
        height, width = image1.shape[:2]
        scale = 1 + value
        img_resize1 = cv2.resize(image1, dsize=None, fx=scale, fy=scale).copy()
        reh, rew = img_resize1.shape[:2]
        X_s = (X_c + 1)/2
        Y_s = (Y_c + 1)/2
        changed_image1 = img_resize1[int(height*(scale - 1)*Y_s) : int(height*(scale - 1)*Y_s + height), int(width*(scale - 1)*X_s) : int(width*(scale -1)*X_s + width)].copy()
        return changed_image1, scale



    def fps(self, data):
        self.lcdNumber.display(data)

    def zoommode(self):
        self.startButton.setEnabled(True)
        self.startButton.setVisible(True)
        self.horizontalSlider.setVisible(True)

    def tomato_table(self, data):
        col = len(data)
        row = 4
        self.tomatolist.setRowCount(col)
        self.tomatolist.clear()
        self.tomatolist.setHorizontalHeaderLabels(self.horheaders)
        for i in range(col):
            for j in range(row):
                item = QTableWidgetItem(data[i][j])
                self.tomatolist.setItem(i, j, item)

        #tomato, X ,Y ,Z = data[0]
        #self.arm.set_position(X, Z, Y, 180, 0, 0 ,radius=0, is_radian=False, wait=False, speed=300)

    

    def activation(self, text):
        if text == "manual":
            self.zoompoint = 0
            self.automode = False
            self.horizontalSlider.setEnabled(True)
            self.walker.manual()

        elif text == "automatic":
            self.automode = True

        if self.automode == True:
            self.horizontalSlider.setEnabled(False)
            self.walker.automatic()

    def scaler(self, num):
        self.scaleNumber.display(num)