
"""
進捗状況
・花形の作成は完了
・DLの定義はグローバル定義を行い、main関数内でwithを用いてsessionを読み込ませる
・カラー画像とデプス画像を同期させる手法の検討が必要
・tfについての理解
"""


import imp
from re import T
from numpy.lib.function_base import append
import cv_bridge
from cv_bridge.core import CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy import qos

from rclpy.qos import qos_profile_sensor_data

from harvest_studio_msg.msg import FruitDataList
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

from tf2_ros import TransformException
from tf2_ros import buffer
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


import numpy as np
import os
import sys
import tensorflow as tf
import cv2
import time
import pyrealsense2 as rs2

sys.path.append("..")
from py_harvest_studio.object_detection.utils import label_map_util
from py_harvest_studio.object_detection.utils import visualization_utils as vis_util
from py_harvest_studio.object_detection.utils import realsense_depth_count as rs_depth
from py_harvest_studio.object_detection.utils import realsense_depth_distance as rs_dis
from py_harvest_studio.object_detection.utils import focuspoint
import message_filters



class TomatoDetector(Node):

    # 各パラメータの初期設定
    def __init__(self):
        super().__init__('tomato_detector')
        
        video_qos = qos.QoSProfile(depth=10)
        video_qos.reliability = qos.QoSReliabilityPolicy.BEST_EFFORT
        rs_color_subscriber = message_filters.Subscriber(self, Image, '/camera/color/image_raw', **{'qos_profile': video_qos})
        rs_depth_subscriber = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw', **{'qos_profile': video_qos})
        self.sub_rs_info = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.rs_depth_info_callback, qos_profile_sensor_data)
        self.rs_intrinsics = None
        self.publisher_ = self.create_publisher(FruitDataList, 'fruit_detect_list', 10)
        self.image_publisher = self.create_publisher(Image, '/fruit_detect_image', qos_profile_sensor_data)
        '''
        azure_color_subscriber = message_filters.Subscriber(self, Image, '/azure/camera/color/image_raw', **{'qos_profile': qos_profile_sensor_data})
        azure_depth_subscriber = message_filters.Subscriber(self, Image, '/azure/camera/depth/image_raw', **{'qos_profile': qos_profile_sensor_data})
        '''
        
        '''
        self.rs_image_subscriber = self.create_subscription(Image, '/camera/color/image_raw', self.rs_image_callback, 1)
        self.rs_depth_subscriber = self.create_subscription(Image, 'rs/depth/image_rect_raw', self.rs_depth_callback, 1)
        self.azure_image_subscriber = self.create_subscription(Image, 'rs/camera/image_raw', self.azure_image_callback, 1)
        self.azure_depth_subscriber = self.create_subscription(Image, 'rs/camera/image_raw', self.azure_depth_callback, 1)
        '''
        # timer_period = 0.01
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        # depth画像とcolor画像の同期
        queue_size = 10
        fps = 30.
        delay = 1/fps*0.5
        
        # rs_ts = message_filters.ApproximateTimeSynchronizer([rs_color_subscriber, rs_depth_subscriber], queue_size, delay)
        rs_ts = message_filters.TimeSynchronizer([rs_color_subscriber, rs_depth_subscriber], queue_size)
        rs_ts.registerCallback(self.rs_image_callback)
        '''
        azure_ts = message_filters.ApproximateTimeSynchronizer([azure_color_subscriber, azure_depth_subscriber], queue_size, delay)
        azure_ts.registerCallback(self.azure_image_callback)
        '''
        # TFの定義
        """
        self.tf_rs_buffer = Buffer()
        self.tf_rs_listener = TransformListener(self.tf_rs_buffer, self)
        self.tf_azure_buffer = Buffer()
        self.tf_azure_listener = TransformListener(self.tf_azure_buffer, self)
        self.rs_target_frame = self.get_parameter(
            'realsense_no_target').get_parameter_value().string_value
        self.fruit_target_frame = 'fruit_target_frame'

        self.br = TransformBroadcaster(self)
        """
        #realsenseの解像度指定
        self.width_color = 1280
        self.height_color = 720
        self.width_depth = 1280
        self.height_depth = 720

        # 画像用の変数定義
        # self.rs_color_image = np.zeros(720, 1280)
        # self.rs_depth_image = np.zeros(720, 1280)
        # self.azure_color_image = np.zeros(720, 1280)
        # self.azure_depth_image = np.zeros(720, 1280)

    def rs_depth_info_callback(self, cameraInfo):
        try:
            if self.rs_intrinsics:
                return

            self.rs_intrinsics = rs2.intrinsics()
            self.rs_intrinsics.width = cameraInfo.width
            self.rs_intrinsics.height = cameraInfo.height
            self.rs_intrinsics.ppx = cameraInfo.k[2]
            self.rs_intrinsics.ppy = cameraInfo.k[5]
            self.rs_intrinsics.fx = cameraInfo.k[0]
            self.rs_intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.rs_intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.rs_intrinsics.model = rs2.distortion.kannala_brandt4
            self.rs_intrinsics.coeffs = [i for i in cameraInfo.d]
        
        except CvBridgeError as e:
            print(e)
            return

    # ROS用のImage型msgをnp配列に変換
    def process_image(self, height, width, image, encode):
        try:
            bridge = CvBridge()
            img = bridge.imgmsg_to_cv2(image, encode)
            # print(type(img))
            h, w = img.shape[:2]
            resize_img = cv2.resize(img, dsize=(width, height), interpolation=cv2.INTER_NEAREST)
            return resize_img
        except Exception as err:
            pass

    # 各画像トピックのcallback#
    #  tfにより位置関係を取得し座標変換
    def rs_image_callback(self, color_msg, depth_msg):
        self.rs_color_image = self.process_image(self.height_color, self.width_color, color_msg, "bgr8")
        depth_image = self.process_image(self.height_depth, self.width_depth, depth_msg, "16UC1")
        self.rs_depth_image = depth_image.astype(np.uint16)
        # cv2.imshow("n_depth", self.rs_depth_image)
        # self.get_logger().info(str(max(self.rs_depth_image)))
        self.color_image_callback(mode="rs" ,child_frame="", camera_frame="", buffer="")
        

    def azure_image_callback(self, color_msg, depth_msg):
        self.azure_color_image = self.process_image(self.height_color, self.width_color, color_msg, "bgr8")
        self.azure_depth_image = self.process_image(self.height_depth, self.width_depth, depth_msg, "mono16")       
        self.color_image_callback(mode="azure", child_frame="", camera_frame="", buffer="")

    def azure_depth_callback(self, msg):
        self.azure_depth_image = self.process_image(self.height_depth, self.width_depth, msg)

    def color_image_callback(self, mode, child_frame, camera_frame, buffer):
        """
        from_frame_rel = self.rs_target_frame # world
        to_frame_rel = child_frame # fruit target
        """
        
        fruit_position_x = []
        fruit_position_y = []
        fruit_position_z = []

        # 果実の位置検出
        x, y, z, radius = self.detect_fruits(mode)
        if not x:
            return

        try:
            """
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = camera_frame
            t.child_frame_id = child_frame
            """
            for i in range(len(x)):
                """
                # 現在の果実座標とカメラの位置関係をTFに登録
                t.transform.translation.x = x[i]
                t.transform.translation.y = y[i]
                t.transform.translation.z = z[i]
                t.transform.rotation.x = 0
                t.transform.rotation.y = 0
                t.transform.rotation.z = 0
                t.transform.rotation.w = 1

                self.br.sendTransform(t)

                # アームと果実の座標の位置関係を取得
                now = rclpy.time.Time()
                trans = buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    now)
                fruit_position_x.append(trans.transform.translation.x)
                fruit_position_y.append(trans.transform.translation.y)
                fruit_position_z.append(trans.transform.translation.z)
                """
                fruit_position_x.append(x[i]*0.001)
                fruit_position_y.append(y[i]*0.001)
                fruit_position_z.append(z[i]*0.001)

            # 得られた位置情報をpublish
            pos_data = FruitDataList()
            pos_data.x = fruit_position_x
            pos_data.y = fruit_position_y
            pos_data.z = fruit_position_z
            pos_data.radius = radius

            self.publisher_.publish(pos_data)
            self.get_logger().info("Publish detect fruits")

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform to_frame_rel to from_frame_rel: {ex}')
            return      


    # 物体検出のループ処理を実行
    def detect_fruits(self, mode):
        if mode == "rs":
            color_image = self.rs_color_image
            depth_image = self.rs_depth_image
            intrinsics = self.rs_intrinsics
        
        elif mode == "azure":
            color_image = self.azure_color_image
            depth_image = self.azure_depth_image

        # 各変数の初期化
        length = 0
        quantity = []
        X_c = 0
        Y_c = 0
        value = 0
        auto = False
        first = False
        detect_count = 0
        max_scale = 3.0
        Position_X = []
        Position_Y = []
        Position_Z = []
        Radius = []
        Tomato_Class = []
        return_mode = False

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
                    X_c = -1
                    Y_c = -1
                elif value != max_scale:
                    value += 0.5

                elif value == max_scale:
                    value = 0.5
                    X_c += 1

                if X_c == 2:
                    Y_c += 1
                    X_c = -1
                if return_mode == True:
                    return Position_X, Position_Y, Position_Z, Radius

                if X_c == 1 and Y_c == 1 and value == max_scale:
                    return_mode = True

                random_line.append([X_c, Y_c])
                valuezoom.append(value)
                if auto == False:
                    first = False
                    if max(quantity) != 0:
                        value = valuezoom[quantity.index(max(quantity))]
                        X_c, Y_c = random_line[quantity.index(max(quantity))]
                        detect_count = 0
                    else:
                        auto = True
            if length != 0 and detect_count != 5:
                detect_count += 1
            elif length ==0 and detect_count != 5:
                detect_count = 0
            # print(detect_count)   

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
            image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, subscriptiontogether with the class label.
            scores = detection_graph.get_tensor_by_name('detection_scores:0')
            classes = detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = detection_graph.get_tensor_by_name('num_detections:0')
            image_np_expanded = np.expand_dims(image_np, axis=0) 

            # Actual detection
            (boxes, scores, classes, num_detections) = sess.run(
                [boxes, scores, classes, num_detections],
                feed_dict={image_tensor: image_np_expanded})                                     
            # Visualization of the results of a detection.
            # print('-----------------------------------------------------------------------')
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
                depth_select.append(depth_image[int(boxes[k][0]*self.height_depth):int(boxes[k][2]*self.height_depth), int(boxes[k][1]*self.width_depth):int(boxes[k][3]*self.width_depth)])
                color_tmp = image_np_color[int(boxes[k][0]*self.height_color):int(boxes[k][2]*self.height_color), int(boxes[k][1]*self.width_color):int(boxes[k][3]*self.width_color)]
                color_tmp = cv2.cvtColor(color_tmp, cv2.COLOR_BGR2RGB)
                color_tmp = cv2.resize(color_tmp, (200, 200)).astype(np.float32)
                color_tmp = color_tmp.astype(np.float32)
                color_tmp = color_tmp[:, :, ::-1].copy()
                color_tmp = torch.from_numpy(color_tmp).permute(2, 0, 1)
                color_select.append(torch.autograd.Variable(color_tmp.unsqueeze(0)))            


            #depth画像から切り出したトマトのデータから一番depthが小さい点を算出

            x = list(range(length))
            y = list(range(length))
            predicted = list(range(length))
            sum_depth = list(range(length))
            for k in range(length):
                if names_show[k] == 'tomato':
                    x[k] ,y[k] ,sum_depth[k] = rs_depth.rs_depth_counter(depth_select[k], k)
                    outputs = net(color_select[k])
                    _, predicted[k] = torch.max(outputs.data, 1)
                    # print(y[k])

                    if not y[k]:
                        # dist = depth_image[int((boxes[k][0] + (boxes[k][2]-boxes[k][0])/2)*self.height_color)][int((boxes[k][1] + (boxes[k][3]-boxes[k][1])/2)*self.width_color)]
                        # max_depth.append(dist)
                        pass

                    else:
                        y[k] = np.diff(y[k])
                        pixel_x, pixel_y = focuspoint.calculate_FocusPoint(k, boxes, X_c, Y_c, scale, 1)
                        dist = depth_image[int(pixel_y*self.height_depth)][int(pixel_x*self.width_depth)]
                        _y = list(y[k])
                        if dist >= _y.index(max(_y)):
                            max_depth.append(dist)
                        else:
                            max_depth.append(_y.index(max(_y)))

                else:
                    continue

            t2 = time.time()

            #x軸とy軸をピクセル単位からメートル単位へ変換
            i = 0

            for k in range(length):
                if names_show[k] == 'tomato':
                    if len(max_depth) > i:
                        pixel_x_left, pixel_x_right, pixel_y_up, pixel_y_low = focuspoint.calculate_FocusPoint(k ,boxes ,X_c, Y_c ,scale, 2)
                        X_meter, Y_meter, Z_meter, radius = rs_dis.convert_depth_pixel_to_metric_coordinate(max_depth[i], pixel_x_left, pixel_x_right ,pixel_y_up, pixel_y_low, sum_depth[k], intrinsics)
                        Position_X.append(float(X_meter))
                        Position_Y.append(float(Y_meter))
                        Position_Z.append(float(Z_meter))
                        Radius.append(float(radius))
                        Tomato_Class.append(predicted[k])
                        
                        cv2.putText(image_np, '(%d,%d,%d)'%(X_meter,Y_meter,Z_meter), (int((boxes[k][1] + (boxes[k][3]-boxes[k][1])/2)*self.width_color-120), int((boxes[k][0] + (boxes[k][2]-boxes[k][0])/2)*self.height_color)), cv2.FONT_ITALIC, 1.0, (255, 255, 255), thickness=2)
                        cv2.putText(image_np, '(%d,)'%(predicted[k]), (int((boxes[k][1] + (boxes[k][3]-boxes[k][1])/2)*self.width_color-120), int((boxes[k][0] + (boxes[k][2]-boxes[k][0])/2)*self.height_color-50)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
                        i += 1
                    else:
                        continue
                else:
                    continue

            cv2.putText(image_np, 'FPS : '+str(round(1/(t2-t1), 2)), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), thickness=2)
            cv2.imshow('tomato', image_np)
            # cv2.imshow('depth', depth_image)
            '''
            bridge = CvBridge()
            image_msg = bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera_color_frame"
            self.image_publisher.publish(image_msg)
            '''
            cv2.waitKey(1)
        




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




def run(args=None):
    try:
        rclpy.init(args=args)
        detector = TomatoDetector()
        rclpy.spin(detector)

    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main():
    PACKAGE_NAME = '/home/ikedalab/ros2_ws/src/ROS2-Automato-Robot-Software/py_harvest_studio/py_harvest_studio'
    MODEL_NAME = PACKAGE_NAME + '/object_detection/tomato_graph'

    # Path to frozen detection graph. This is the actual model that is used for the object detection.
    PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

    # List of the strings that is used to add correct label for each box.
    PATH_TO_LABELS = os.path.join(PACKAGE_NAME + '/object_detection/training', 'object-detection.pbtxt')

    NUM_CLASSES = 2

    global detection_graph, label_map, categories, category_index, net, sess
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

    net = Net()
    param = torch.load(PACKAGE_NAME + '/object_detection/train_model/weight.pth')
    net.load_state_dict(param)


    with detection_graph.as_default():
        sess = tf.compat.v1.Session(graph=detection_graph)
        run()    

if __name__=='__main__':
    main()
