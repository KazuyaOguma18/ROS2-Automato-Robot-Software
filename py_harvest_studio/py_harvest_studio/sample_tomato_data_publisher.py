from harvest_studio_msg.msg import FruitDataList

import rclpy
from rclpy.node import Node
from rclpy import qos

class SampleTomatoDataPublisher(Node):
    def __init__(self):
        super().__init__('sample_tomato_data_publisher')

        self.pub = self.create_publisher(FruitDataList, 'fruit_detect_list', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        data = FruitDataList()
        
        x = [0.4, 0.4, 0.4]
        y = [0.5, 0.5, 0.5]
        z = [0.6, 0.5, 0.5]
        r = 0.06

        for i in range(len(x)):
            data.x.append(x[i])
            data.y.append(y[i])
            data.z.append(z[i])
            data.radius.append(r)

        self.pub.publish(data)

def main():
    try:
        rclpy.init(args=None)
        detector = SampleTomatoDataPublisher()
        rclpy.spin(detector)

    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

