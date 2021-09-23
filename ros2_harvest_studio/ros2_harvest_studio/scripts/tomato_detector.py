#! /usr/bin/env python3

"""
進捗状況
・花形の作成は完了
・DLの定義はグローバル定義を行い、main関数内でwithを用いてsessionを読み込ませる
・カラー画像とデプス画像を同期させる手法の検討が必要
・tfについての理解
"""


import rclpy
from rclpy.node import Node

from ros2_harvest_studio.msg import FruitDataList
from sensor_msgs.msg import Image

import numpy as np
import os
import sys
import tensorflow as tf
import cv2
import time

sys.path.append("..")
from ..object_detection.utils import label_map_util
from ..object_detection.utils import visualization_utils as vis_util
from ..object_detection.utils import realsense_depth_count as rs_depth
from ..object_detection.utils import realsense_depth_distance as rs_dis
from ..object_detection.utils import focuspoint
from PIL import Image




class TomatoDetector(Node):

    # 各パラメータの初期設定
    def __init__(self):
        super().__init__('tomato_detector')
        self.publisher_ = self.create_publisher(FruitDataList, 'fruit_detect_list', 10)
        self.rs_image_subscriber = self.create_subscription(
            Image, 
            'rs/camera/image_raw',
            self.rs_image_callback,
            10)
        self.rs_depth_subscriber = self.create_subscription(
            Image, 
            'rs/camera/image_raw',
            self.rs_depth_callback,
            10)
        self.azure_image_subscriber = self.create_subscription(
            Image, 
            'rs/camera/image_raw',
            self.azure_image_callback,
            10)
        self.azure_depth_subscriber = self.create_subscription(
            Image, 
            'rs/camera/image_raw',
            self.azure_depth_callback,
            10)
        # timer_period = 0.01
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.load_model()


    # 各画像トピックのcallback
    def rs_image_callback(self, msg):
        # 型変換が必要な気がする
        self.rs_image = msg
        
        self.detect_fruits()
        # tfにより位置関係を取得し座標変換
        # 得られた位置情報をpublish

    def rs_depth_callback(self, msg):
        # 型変換が必要な気がする
        self.rs_depth = msg
        

    def azure_image_callback(self, msg):
        # 型変換が必要な気がする
        self.azure_image = msg
        
        self.detect_fruits()

    def azure_depth_callback(self, msg):
        # 型変換が必要な気がする
        self.azure_depth = msg


    # 物体検出におけるモデル等読み込み
    # →グローバルで定義しといた方が楽かも
    def load_model(self):
        # What model to download.
        MODEL_NAME = '/home/ikedalab/catkin_ws/src/xarm_ros/xarm_planner/object_detection/tomato_graph'

        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

        # List of the strings that is used to add correct label for each box.
        PATH_TO_LABELS = os.path.join('/home/ikedalab/catkin_ws/src/xarm_ros/xarm_planner/object_detection/training', 'object-detection.pbtxt')

        NUM_CLASSES = 2

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)    

    # 物体検出のループ処理を実行
    def detect_fruits(self, color_image, depth_image):
        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                while True:
                    t1 = time.time()
                    if length == 0:
                        auto = True
                    if auto == True:
                        detect_count = 0
                        if first == False:
                            value = 0.5
                            first = True
                            quantity = []
                            valuezoom = []
                            random_line = []
                            value_max = False
                            X_c = -1
                            Y_c = -1
                        elif value != max_scale:
                            value += 0.5
                            value_max = False
                        elif value == max_scale:
                            value = 0.5
                            X_c += 1
                            value_max = True
                        if X_c == 2:
                            Y_c += 1
                            X_c = -1
                        if X_c == 1 and Y_c == 1 and value_max == True:
                            auto = False
                        random_line.append([X_c, Y_c])
                        valuezoom.append(value)
                        if auto == False:
                            first = False
                            if max(quantity) != 0:
                                value = valuezoom[quantity.index(max(quantity))]
                                X_c, Y_c = random_line[quantity.index(max(quantity))]
                                search = False
                                detect_count = 0
                                t6 = time.time()
                            else:
                                auto = True
                    if length != 0 and detect_count != 5:
                        detect_count += 1
                    elif length ==0 and detect_count != 5:
                        detect_count = 0
                    print(detect_count)    


                    depth_select = []
                    color_select = []
                    max_depth = []            

                    #image = Image.fromarray(np.uint8(color_image))
                    # the array based representation of the image will be used later in order to prepare the
                    # result image with boxes and labels on it.  
                    # 画像のズーム処理
                    image_np, scale = focuspoint.calculate_image(color_image, value, X_c, Y_c)
                    image_np_color, scale = focuspoint.calculate_image(color_image, value, X_c, Y_c)
                    depth_image, scale = focuspoint.calculate_image(depth_image, value, X_c, Y_c)

                    # 果実及びヘタ検出
                    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                    image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                    # Each box represents a part of the image where a particular object was detected.
                    boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                    # Each score represent how level of confidence for each of the objects.
                    # Score is shown on the result image, together with the class label.
                    scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                    classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                    num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
                    image_np_expanded = np.expand_dims(image_np, axis=0) 

                    # Actual detection
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
                        self.category_index, 
                        use_normalized_coordinates=True,
                        min_score_thresh=.6,
                        line_thickness=5)





import torch
import torch.nn.functional as F
import torch.nn as nn
from torchvision import models, transforms


class Net(nn.Module):
    # NNの各構成要素を定義
    def __init__(self):
        super(Net, self).__init__()

        # 畳み込み層とプーリング層の要素定義
        self.conv1 = nn.Conv2d(3, 6, 5)  # (入力, 出力, 畳み込みカーネル（5*5）)
        self.pool = nn.MaxPool2d(2, 2)  # (2*2)のプーリングカーネル
        self.conv2 = nn.Conv2d(6, 16, 5)
        # 全結合層の要素定義
        self.fc1 = nn.Linear(16 * 47 * 47, 120)  # (入力, 出力)
        self.fc2 = nn.Linear(120, 84)
        self.fc3 = nn.Linear(84, 5)# クラス数が１０なので最終出力数は10

    # この順番でNNを構成
    def forward(self, x):
        
        x = self.pool(F.relu(self.conv1(x)))  # conv1->relu->pool
        x = self.pool(F.relu(self.conv2(x)))  # conv2->relu->pool
        x = x.view(-1, 16*47*47)
        x = F.relu(self.fc1(x))  # fc1->relu
        x = F.relu(self.fc2(x))  # fc2->relu
        x = self.fc3(x)
        return x




def main(args=None):
    try:
        rclpy.init(args=args)
        detector = TomatoDetector()
        rclpy.spin(detector)

    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()