import imp
from numpy.lib.type_check import imag
import rclpy
from rclpy import qos
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2
import message_filters


class TrialSubscriber(Node):

    def __init__(self):
        super().__init__('trial_subscriber')
        color_subscription = message_filters.Subscriber(self, Image, '/camera/color/image_raw', **{'qos_profile': qos_profile_sensor_data})
        depth_subscription = message_filters.Subscriber(self, Image, '/camera/depth/image_rect_raw', **{'qos_profile': qos_profile_sensor_data})
        '''
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription
        '''

        queue_size = 10
        fps = 30.
        delay = 1/fps*0.5
        ts = message_filters.ApproximateTimeSynchronizer([color_subscription, depth_subscription], queue_size, delay)
        ts.registerCallback(self.listener_callback)
        self.br = CvBridge()
        self.get_logger().info('Subscribed')

    def listener_callback(self, msg1, msg2):
        try:
            self.get_logger().info('Subscribed')
            current_frame = self.br.imgmsg_to_cv2(msg1)

        except Exception as err:
            self.get_logger().info('Error now')

def main(args=None):
    try:
        rclpy.init(args=args)
        trial_subscriber = TrialSubscriber()
        rclpy.spin(trial_subscriber)

    except KeyboardInterrupt:
        pass

    finally:
        trial_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()