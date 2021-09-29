#! /usr/bin/env python3

"""
進捗状況
・花形の作成は完了
・各要素の関数を埋めていく
・複数のpubやsubを行うことができるのかサンプルコードを作成して動作確認を行っていく
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from ros2_harvest_studio.msg import FruitDataList


class FruitDataProcessor(Node):

    def __init__(self):
        super().__init__('fruit_data_processor')
        self.position_publihser_ = self.create_publisher( , 'fruit_position_data', 10)
        self.fruit_status_publisher_ = self.create_publisher(Bool, 'fruit_detect_status', 10)
        self.list_subscriber_ = self.create_subscription(FruitDataList , 'fruit_detect_list', self.fruit_detect_list_callback, 10)
        self.command_subscriber_ = self.create_subscription(, 'fruit_send_command', self.fruit_send_command_callback, 10)

    # 検出された果実情報のコールバック
    # 重複果実のチェック、果実座標の平均取得、果実位置の線形変換＆平面距離取得＆ソート
    def fruit_detect_list_callback(self, msg):
        self.check_duplicate_fruits()
        self.get_fruit_position_average()
        self.fruit_position_linear_tf()

    def fruit_send_command_callback(self, msg):

    def check_duplicate_fruits(self):

    def get_fruit_position_average(self):

    def fruit_position_linear_tf(self):



def main(args=None):
    try:
        rclpy.init(args=args)
        processsor = FruitDataProcessor()
        rclpy.spin(processsor)

    except KeyboardInterrupt:
        pass
    finally:
        processsor.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()