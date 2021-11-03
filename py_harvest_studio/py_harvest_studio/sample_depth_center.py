import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.qos import qos_profile_sensor_data
import message_filters

from cv_bridge import CvBridge
from cv_bridge.core import CvBridgeError

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import numpy as np
import cv2

import pyrealsense2 as rs2

class SampleDepthCenter(Node):
    def __init__(self):
        super().__init__('sample_depth_counter')

        video_qos = qos.QoSProfile(depth=10)
        video_qos.reliability = qos.QoSReliabilityPolicy.BEST_EFFORT
        rs_color_subscriber = message_filters.Subscriber(self, Image, '/camera/color/image_raw', **{'qos_profile': video_qos})
        rs_depth_subscriber = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw', **{'qos_profile': video_qos})
        self.sub_rs_info = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.rs_depth_info_callback, qos_profile_sensor_data)
        self.rs_intrinsics = None

        queue_size = 10
        fps = 30.
        delay = 1/fps*0.5

        rs_ts = message_filters.ApproximateTimeSynchronizer([rs_color_subscriber, rs_depth_subscriber], queue_size, delay)
        rs_ts.registerCallback(self.rs_image_callback)

        self.height_color = 720
        self.width_color = 1280
        self.height_depth = 720
        self.width_depth = 1280

    def rs_image_callback(self, color_msg, depth_msg):
        self.rs_color_image = self.process_image(self.height_color, self.width_color, color_msg, "bgr8")
        self.rs_depth_image = self.process_image(self.height_depth, self.width_depth, depth_msg, depth_msg.encoding)
        color_img = self.rs_color_image
        cv2.drawMarker(color_img, (640, 360), (255, 255, 255))
        cv2.imshow("depth", self.rs_depth_image)
        cv2.imshow("color", color_img)
        cv2.waitKey(1)

        h, w = self.rs_depth_image.shape[:2]
        self.get_logger().info("Center depth : {0}".format(self.rs_depth_image[int(h/2), int(w/2)]))

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

def main():
    try:
        rclpy.init(args=None)
        detector = SampleDepthCenter()
        rclpy.spin(detector)

    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
