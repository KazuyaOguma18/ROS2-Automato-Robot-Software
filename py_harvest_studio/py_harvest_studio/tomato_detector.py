from math import radians

from PIL.Image import NONE
from google.protobuf.text_format import Error
import cv_bridge
from cv_bridge.core import CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy import qos

from rclpy.qos import qos_profile_sensor_data

from harvest_studio_msg.msg import FruitDataList
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Empty
from cv_bridge import CvBridge

from tf2_ros import TransformException
from tf2_ros import buffer
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from ament_index_python import get_package_share_directory

from rclpy.duration import Duration


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

class Intrinsics:
    def __init__(self):
        self.width = None
        self.height = None
        self.ppx = None
        self.ppy = None
        self.fx = None
        self.fy = None


class TomatoDetector(Node):

    # 各パラメータの初期設定
    def __init__(self):
        super().__init__('tomato_detector')
        
        # 動作モードの取得 : rs or azure
        self.declare_parameter('camera_mode', 'azure')
        camera_mode = self.get_parameter('camera_mode')
        # depth画像とcolor画像の同期
        queue_size = 10
        fps = 30.
        delay = 1/fps*0.5        
            
        if str(camera_mode.value) == 'rs':
            video_qos = qos.QoSProfile(depth=10)
            video_qos.reliability = qos.QoSReliabilityPolicy.BEST_EFFORT
            rs_color_subscriber = message_filters.Subscriber(self, Image, '/rs_color/image_com', **{'qos_profile': video_qos})
            rs_depth_subscriber = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw', **{'qos_profile': video_qos})
            sub_rs_info = self.create_subscription(CameraInfo, '/camera/aligned_depth_to_color/camera_info', self.depth_info_callback, qos_profile_sensor_data)
            rs_ts = message_filters.TimeSynchronizer([rs_color_subscriber, rs_depth_subscriber], queue_size)
            rs_ts.registerCallback(self.rs_image_callback)         
            #realsenseの解像度指定
            self.width_color = 640
            self.height_color = 480
            self.width_depth = 640
            self.height_depth = 480 
            self.get_logger().info("camera_mode: "+str(camera_mode.value))   
            
            # realsenseのtf定義
            self.camera_frame = "camera_link"    
                   
        elif str(camera_mode.value) == 'azure':
            video_qos = qos.QoSProfile(depth=10)
            video_qos.reliability = qos.QoSReliabilityPolicy.BEST_EFFORT
            azure_color_subscriber = message_filters.Subscriber(self, Image, '/azure/rgb/image_raw', **{'qos_profile': video_qos})
            azure_depth_subscriber = message_filters.Subscriber(self, Image, '/azure/depth_to_rgb/image_raw', **{'qos_profile': video_qos})
            sub_azure_info = self.create_subscription(CameraInfo, '/azure/depth_to_rgb/camera_info', self.depth_info_callback, qos_profile_sensor_data)
            azure_ts = message_filters.ApproximateTimeSynchronizer([azure_color_subscriber, azure_depth_subscriber], queue_size, delay)
            azure_ts.registerCallback(self.azure_image_callback)
            #azure kinectの解像度指定
            self.width_color = 640
            self.height_color = 1080
            self.width_depth = 640
            self.height_depth = 1080

            # azureのtf定義
            self.camera_frame = "camera_base"

            
            self.get_logger().info("camera_mode: "+str(camera_mode.value))
               
        self.intrinsics = Intrinsics()    
        self.publisher_ = self.create_publisher(FruitDataList, '/fruit_detect_list', 10)
        pub_video_qos = qos.QoSProfile(depth=10)
        pub_video_qos = qos.QoSReliabilityPolicy.BEST_EFFORT
        self.image_publisher = self.create_publisher(Image, str(camera_mode.value) + '_fruit_detect_image', **{'qos_profile': pub_video_qos})
        self.depth_publisher = self.create_publisher(Image, str(camera_mode.value) + '_depth_image', **{'qos_profile': pub_video_qos})
        self.loop_complete_publisher = self.create_publisher(Empty, str(camera_mode.value) + '_loop_complete', 10)


        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  

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


        # 画像用の変数定義
        # self.rs_color_image = np.zeros(720, 1280)
        # self.rs_depth_image = np.zeros(720, 1280)
        # self.azure_color_image = np.zeros(720, 1280)
        # self.azure_depth_image = np.zeros(720, 1280)
        

    def depth_info_callback(self, cameraInfo):
        try:
            if self.intrinsics.fx != None:
                return

            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
        
        except Exception as e:
            print(e)
            return

    # ROS用のImage型msgをnp配列に変換
    def process_image(self, height, width, image, encode):
        try:
            bridge = CvBridge()
            img = bridge.imgmsg_to_cv2(image, encode)


        except Exception as err:
            print("image convert failed! :{}".format(err))
            return

        try:
            # print(type(img))
            h, w = img.shape[:2]
            resize_img = cv2.resize(img, dsize=(width, height), interpolation=cv2.INTER_NEAREST)
            return resize_img

        except Exception as err:
            print("image resize failed! :{}".format(err))
            return

    # 各画像トピックのcallback#
    #  tfにより位置関係を取得し座標変換
    def rs_image_callback(self, color_msg, depth_msg):
        self.rs_color_image = self.process_image(self.height_color, self.width_color, color_msg, "bgr8")
        depth_image = self.process_image(self.height_depth, self.width_depth, depth_msg, "16UC1")
        self.rs_depth_image = depth_image.astype(np.uint16)
        # cv2.imshow("n_depth", self.rs_depth_image)
        # self.get_logger().info(str(max(self.rs_depth_image)))
        self.color_image_callback(mode="rs" ,child_frame="rs_tomato", camera_frame="camera_link")
        

    def azure_image_callback(self, color_msg, depth_msg):
        color_tmp  = self.process_image(1080, 1920, color_msg, "bgr8")
        self.azure_color_image = color_tmp[0:1080, 640:1280]
        depth_tmp = self.process_image(1080, 1920, depth_msg, "16UC1")
        self.azure_depth_image = depth_tmp[0:1080, 640:1280]
        self.color_image_callback(mode="azure", child_frame="azure_tomato", camera_frame="camera_base")

    def azure_depth_callback(self, msg):
        self.azure_depth_image = self.process_image(self.height_depth, self.width_depth, msg)

    def color_image_callback(self, mode, child_frame, camera_frame):
        """
        from_frame_rel = self.rs_target_frame # world
        to_frame_rel = child_frame # fruit target
        """
        
        fruit_position_x = []
        fruit_position_y = []
        fruit_position_z = []
        fruit_radius = []
        

        # 果実の位置検出
        if self.intrinsics.fx == None:
            return
        
        x, y, z, radius = self.detect_fruits(mode)
        
        # loop処理完了時のデータを送信
        emp_data = Empty()
        self.loop_complete_publisher.publish(emp_data)
        if not x:
            return


        # for i in range(len(x)):    
        #     fruit_position_x.append(x[i])
        #     fruit_position_y.append(y[i])
        #     fruit_position_z.append(z[i])
        #     fruit_radius.append(radius[i]* 0.001)
        #     """
        #     fruit_position_x.append(x[i]*0.001)
        #     fruit_position_y.append(y[i]*0.001)
        #     fruit_position_z.append(z[i]*0.001)
        #     fruit_radius.append(radius[i]*0.001)
        #     """
        # # 得られた位置情報をpublish
        # pos_data = FruitDataList()
        # pos_data.x = fruit_position_x
        # pos_data.y = fruit_position_y
        # pos_data.z = fruit_position_z
        # pos_data.radius = fruit_radius
        # self.publisher_.publish(pos_data)
        # self.get_logger().info("Publish detect fruits")

 
        
    def transform_tomato_position(self, x, y, z, mode):
        if mode == 'rs':
            camera_frame = 'camera_link'
            child_frame = 'rs_tomato'
            
        elif mode == 'azure':
            camera_frame = 'camera_base'
            child_frame = 'azure_tomato'
            
        try:
            t = TransformStamped()
            t.header.frame_id = camera_frame
            t.child_frame_id = child_frame

            # 現在の果実座標とカメラの位置関係をTFに登録    
            now = self.get_clock().now()
            t.header.stamp = now.to_msg()
            t.transform.translation.x = z*0.001
            t.transform.translation.y = x*(-0.001)
            t.transform.translation.z = y*0.001
            t.transform.rotation.x = 0.
            t.transform.rotation.y = 0.
            t.transform.rotation.z = 0.
            t.transform.rotation.w = 1.
            self.br.sendTransform(t)
            
            # カメラと果実の座標の位置関係を取得
            trans = self.tf_buffer.lookup_transform(                
                "link_base",
                camera_frame,
                rclpy.time.Time(seconds=0), 
                timeout=Duration(seconds=0.5))
            
            # トマト座標をカメラ座標系からアーム座標系へ線形変換
            q0 = trans.transform.rotation.w
            q1 = trans.transform.rotation.x
            q2 = trans.transform.rotation.y
            q3 = trans.transform.rotation.z
            
            rotate_matrix = np.array([[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
                                      [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
                                      [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]])
            # カメラの位置
            camera_position_matrix = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            # カメラから見た果実の位置
            fruit_position_matrix = np.array([z*0.001, x*(-0.001), y*0.001])
            # 果実をカメラ座標系からアーム座標系に変換
            transformed_fruit_matrix = np.dot(rotate_matrix, fruit_position_matrix.T) + camera_position_matrix.T
            fruit_position = [transformed_fruit_matrix[0], transformed_fruit_matrix[1], transformed_fruit_matrix[2]]
            
            return fruit_position
        
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform link_base to camera_frame: {ex}')
            return False          
        
    # 物体検出のループ処理を実行
    def detect_fruits(self, mode):
        if mode == "rs":
            color_image = self.rs_color_image
            depth_image = self.rs_depth_image
            min_scale = 0.5
            max_scale = 1.5
        
        elif mode == "azure":
            color_image = self.azure_color_image
            depth_image = self.azure_depth_image
            min_scale = 2
            max_scale = 3.5
            
        # 各変数の初期化
        length = 0
        quantity = []
        X_c = 0
        Y_c = 0
        value = 0
        auto = False
        first = False
        detect_count = 0

        Tomato_Class = []
        return_mode = False

        while True:
            Position_X = []
            Position_Y = []
            Position_Z = []
            Radius = []
            
            t1 = time.time()

            if length == 0:
                auto = True
            if auto == True:
                detect_count = 0
                if first == False:
                    value = min_scale
                    first = True
                    quantity = []
                    valuezoom = []
                    random_line = []
                    X_c = -0.5
                    Y_c = -0.5
                elif value != max_scale:
                    value += 0.5

                elif value == max_scale:
                    value = min_scale
                    X_c += 0.5

                if X_c == 1.0:
                    Y_c += 0.5
                    X_c = -0.5
                if return_mode == True:
                    return Position_X, Position_Y, Position_Z, Radius

                if X_c == 0.5 and Y_c == 0.5 and value == max_scale:
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
            
            x_tmp = []
            y_tmp = []
            z_tmp = []          

            #image = Image.fromarray(np.uint8(color_image))
            # the array based representation of the image will be used later in order to prepare the
            # result image with boxes and labels on it.  
            # 画像のズーム処理
            image_np, scale = focuspoint.calculate_image(color_image, value, X_c, Y_c)
            image_np_color, scale = focuspoint.calculate_image(color_image, value, X_c, Y_c)
            depth_np, scale = focuspoint.calculate_image(depth_image, value, X_c, Y_c)

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
                depth_select.append(depth_np[int(boxes[k][0]*self.height_depth):int(boxes[k][2]*self.height_depth), int(boxes[k][1]*self.width_depth):int(boxes[k][3]*self.width_depth)])
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
                        #pixel_x, pixel_y = focuspoint.calculate_FocusPoint(k, boxes, X_c, Y_c, scale, 1)
                        #dist = depth_image[int(pixel_y*self.height_depth)][int(pixel_x*self.width_depth)]
                        _y = list(y[k])
                        #if dist >= _y.index(max(_y)):
                            #max_depth.append(dist)
                        #else:
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
                        X_meter, Y_meter, Z_meter, radius = rs_dis.convert_depth_pixel_to_metric_coordinate(max_depth[i], pixel_x_left, pixel_x_right ,pixel_y_up, pixel_y_low, mode, self.intrinsics)
                        x_tmp.append(float(X_meter))
                        y_tmp.append(float(Y_meter))
                        z_tmp.append(float(Z_meter))
                        Radius.append(float(radius) * 0.001)
                        Tomato_Class.append(predicted[k])
                        
                        cv2.putText(image_np, '(%d,%d,%d)'%(X_meter,Y_meter,Z_meter), (int((boxes[k][1] + (boxes[k][3]-boxes[k][1])/2)*self.width_color-120), int((boxes[k][0] + (boxes[k][2]-boxes[k][0])/2)*self.height_color)), cv2.FONT_ITALIC, 1.0, (255, 255, 255), thickness=2)
                        cv2.putText(image_np, '(%d,)'%(predicted[k]), (int((boxes[k][1] + (boxes[k][3]-boxes[k][1])/2)*self.width_color-120), int((boxes[k][0] + (boxes[k][2]-boxes[k][0])/2)*self.height_color-50)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
                        i += 1
                    else:
                        continue
                else:
                    continue

            # cv2.namedWindow("tomato", cv2.WINDOW_NORMAL)
            cv2.putText(image_np, 'FPS : '+str(round(1/(t2-t1), 2)), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), thickness=2)
            # cv2.imshow('tomato', image_np)
            # cv2.imshow('depth', depth_image)
            bridge = CvBridge()
            depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = "stand_base"   
            self.depth_publisher.publish(depth_msg)         
            image_msg = bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "stand_base"
            self.image_publisher.publish(image_msg)
            
            
            
            for i in range(len(x_tmp)):
                trans = self.transform_tomato_position(x_tmp[i], y_tmp[i], z_tmp[i], mode)
                if trans:
                    Position_X.append(trans[0])
                    Position_Y.append(trans[1])
                    Position_Z.append(trans[2])
                    # time.sleep(0.5)
            
            if len(x_tmp) > 0:
                pos_data = FruitDataList()
                pos_data.x = Position_X
                pos_data.y = Position_Y
                pos_data.z = Position_Z
                pos_data.radius = Radius
                self.publisher_.publish(pos_data)
                # self.get_logger().info("Publish detect fruits")
            
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
    PACKAGE_NAME = get_package_share_directory('py_harvest_studio')

    # Path to frozen detection graph. This is the actual model that is used for the object detection.
    PATH_TO_CKPT = PACKAGE_NAME + '/frozen_inference_graph.pb'

    # List of the strings that is used to add correct label for each box.
    PATH_TO_LABELS = os.path.join(PACKAGE_NAME, 'object-detection.pbtxt')

    NUM_CLASSES = 2

    global detection_graph, label_map, categories, category_index, net, sess
    physical_devices = tf.config.list_physical_devices('GPU')
    if len(physical_devices) > 0:
        for device in physical_devices:
            tf.config.experimental.set_memory_growth(device, True)
            print('{} memory growth: {}'.format(device, tf.config.experimental.get_memory_growth(device)))
    else:
        print("Not enough GPU hardware devices available")
        
    gpu_options = tf.compat.v1.GPUOptions(per_process_gpu_memory_fraction=0.333)
    
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
    param = torch.load(PACKAGE_NAME + '/weight.pth')
    net.load_state_dict(param)


    with detection_graph.as_default():
        sess = tf.compat.v1.Session(graph=detection_graph, config=tf.compat.v1.ConfigProto(gpu_options=gpu_options))
        run()    

if __name__=='__main__':
    main()
