#! /usr/bin/env python3

"""
進捗状況
・花形の作成は完了
・各要素の関数を埋めていく
・複数のpubやsubを行うことができるのかサンプルコードを作成して動作確認を行っていく
"""

from os import stat
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from harvest_studio_msg.msg import FruitDataList
from harvest_studio_msg.srv import FruitPositionData

import math


class FruitDataProcessor(Node):

    def __init__(self):
        super().__init__('fruit_data_processor')
        self.position_service_ = self.create_service(FruitPositionData, 'fruit_position_data', self.fruit_server_callback)
        self.fruit_status_publisher_ = self.create_publisher(Bool, 'fruit_detect_status', 10)
        self.harvest_list_publisher_ = self.create_publisher(FruitDataList, 'harvest_list', 10)
        self.list_subscriber_ = self.create_subscription(FruitDataList , 'fruit_detect_list', self.fruit_detect_list_callback, 10)

        self.timer = self.create_timer(0.5, self.timer_callback)

        # 果実情報を配列に代入
        self.x = []
        self.y = []
        self.z = []
        self.radius = []
        self.detect_number = []

    # 検出された果実情報のコールバック
    # 重複果実のチェック、果実座標の平均取得、果実位置の線形変換＆平面距離取得＆ソート
    def fruit_detect_list_callback(self, msg):

        self.duplicate_index = []

        self.msg_x = msg.x
        self.msg_y = msg.y
        self.msg_z = msg.z
        self.msg_radius = msg.radius

        self.check_duplicate_fruits(msg)
        # self.get_logger().info("check_duplicate_fruits")
        self.get_fruit_position_average()
        # self.get_logger().info("fruit_position_average")
        self.fruit_position_linear_tf()
        # self.get_logger().info("fruit_position_linear_tf")

        self.harvest_x = self.x
        self.harvest_y = self.y
        self.harvest_z = self.z
        self.harvest_radius = self.radius

        # 実際に収穫を行う果実の情報を出力
        harvest_list = FruitDataList()

        for i in range(len(self.x)):
            harvest_list.x.append(self.harvest_x[i])
            harvest_list.y.append(self.harvest_y[i])
            harvest_list.z.append(self.harvest_z[i])
            harvest_list.radius.append(self.harvest_radius[i])

        self.harvest_list_publisher_.publish(harvest_list)
        # self.get_logger().info("harvest_list_publish")



    # 検出された果実が過去に検出された果実とかぶっていないかチェック
    def check_duplicate_fruits(self, msg):

        msg_duplicate_index = []

        for i in range(len(self.x)):
            for j in range(len(msg.x)):
                # 新しく検出された果実がこれまでの果実の球の範囲内になかったら、新しく配列に追加
                if math.sqrt((self.x[i] - msg.x[j])**2 + (self.y[i] - msg.y[j])**2 + (self.z[i] - msg.z[j])**2) < self.radius[i]:
                    self.duplicate_index.append([i, j])
                    msg_duplicate_index.append(j)

        # かぶってなかったデータを追加
        # self.get_logger().info(str(msg_duplicate_index))
        for i in range(len(msg.x)):
            if not i in msg_duplicate_index:
                self.x.append(msg.x[i])
                self.y.append(msg.y[i])
                self.z.append(msg.z[i])
                self.radius.append(msg.radius[i])
                self.detect_number.append(1)

        # self.x内にデータにかぶりがないかどうか判定
        self_duplicate_index = []
        for i in range(len(self.x)-1):
            for j in range(i+1, len(self.x)):
                if math.sqrt((self.x[i] - self.x[j])**2 + (self.y[i] - self.y[j])**2 + (self.z[i] - self.z[j])**2) < self.radius[i]:
                    self_duplicate_index.append(i)
        
        for i in self_duplicate_index:
            self.delete_fruit_data(i)

    # 同じ果実と判定された果実の平均座標の取得
    def get_fruit_position_average(self):
        # 同じデータ番号を簡単に見つけるためにソート
        self.duplicate_index = sorted(self.duplicate_index)
        # ダミーデータ（エラー防止用）
        self.duplicate_index.append([0,0])
        # 最初のデータにこれまでの検出分を足す
        self.x[self.duplicate_index[0][0]] += (self.detect_number[self.duplicate_index[0][0]] - 1)*self.x[self.duplicate_index[0][0]] 
        self.y[self.duplicate_index[0][0]] += (self.detect_number[self.duplicate_index[0][0]] - 1)*self.y[self.duplicate_index[0][0]] 
        self.z[self.duplicate_index[0][0]] += (self.detect_number[self.duplicate_index[0][0]] - 1)*self.z[self.duplicate_index[0][0]] 

        # ダミー以外の回数分繰り返す
        for i in range(len(self.duplicate_index) - 1):

            if self.duplicate_index[i][0] != self.duplicate_index[i+1][0]:
                self.x[self.duplicate_index[i][0]] = self.x[self.duplicate_index[i][0]] / self.detect_number[self.duplicate_index[i][0]]
                self.y[self.duplicate_index[i][0]] = self.y[self.duplicate_index[i][0]] / self.detect_number[self.duplicate_index[i][0]]
                self.z[self.duplicate_index[i][0]] = self.z[self.duplicate_index[i][0]] / self.detect_number[self.duplicate_index[i][0]]
                self.radius[self.duplicate_index[i][0]] = self.radius[self.duplicate_index[i][0]] / self.detect_number[self.duplicate_index[i][0]]

                if i < len(self.duplicate_index) - 1:
                    self.x[self.duplicate_index[i+1][0]] += (self.detect_number[self.duplicate_index[i+1][0]] - 1)*self.x[self.duplicate_index[i+1][0]]
                    self.y[self.duplicate_index[i+1][0]] += (self.detect_number[self.duplicate_index[i+1][0]] - 1)*self.y[self.duplicate_index[i+1][0]]
                    self.z[self.duplicate_index[i+1][0]] += (self.detect_number[self.duplicate_index[i+1][0]] - 1)*self.z[self.duplicate_index[i+1][0]]
                    self.radius[self.duplicate_index[i+1][0]] += (self.detect_number[self.duplicate_index[i+1][0]] - 1)*self.radius[self.duplicate_index[i+1][0]]

            else:
                self.x[self.duplicate_index[i][0]] += self.msg_x[self.duplicate_index[i][1]]
                self.y[self.duplicate_index[i][0]] += self.msg_y[self.duplicate_index[i][1]]
                self.z[self.duplicate_index[i][0]] += self.msg_z[self.duplicate_index[i][1]]
                self.radius[self.duplicate_index[i][0]] += self.msg_radius[self.duplicate_index[i][1]]
                self.detect_number[self.duplicate_index[i][0]] += 1
            
                

    # 任意平面からの距離に基づく、果実位置のソート
    def fruit_position_linear_tf(self):
        # 平面の方程式の定義
        # 平面を(1,0,0), (0,c,a)のベクトルから平面を求める
        c = 2
        a = 1
        l = []
        for i in range(len(self.x)):
            l.append([abs(c*self.z[i] - a*self.y[i])/math.sqrt(a^2+c^2), int(i)])

        l = sorted(l)
        sorted_x = list(range(len(self.x)))
        sorted_y = list(range(len(self.x)))
        sorted_z = list(range(len(self.x)))
        sorted_radius = list(range(len(self.x)))
        sorted_detect_number = list(range(len(self.x)))

        for i in range(len(self.x)):
            sorted_x[i] = self.x[l[i][1]] 
            sorted_y[i] = self.y[l[i][1]] 
            sorted_z[i] = self.z[l[i][1]] 
            sorted_radius[i] = self.radius[l[i][1]] 
            sorted_detect_number[i] = self.detect_number[l[i][1]]

        self.x = sorted_x
        self.y = sorted_y
        self.z = sorted_z
        self.radius = sorted_radius
        self.detect_number = sorted_detect_number 

    # generate_motion_point から果実位置取得命令が出たときのコールバック
    def fruit_server_callback(self, request, responce):
        if request.order == True and len(self.x) > 0:
            i = 0

            # 五回以上検出された信頼性のある果実から収穫
            while self.detect_number[i] < 5:
                i += 1

            if len(self.x) >= i+1:
                responce.x = self.harvest_x[i]
                responce.y = self.y[i]
                responce.z = self.z[i]
                responce.radius = self.radius[i]
                responce.success = True
    
                # 収穫データとして送信されたデータは削除
                # 削除された直後に収穫完了する前に新たに検出されて、また配列に入りそうな予感
                self.delete_fruit_data(i)


        else:
            responce.x = 0.0
            responce.y = 0.0
            responce.z = 0.0
            responce.radius = 0.0
            responce.success = False
        
        return responce

    def delete_fruit_data(self, i):
        del self.x[i]
        del self.y[i]
        del self.z[i]
        del self.radius[i]
        del self.detect_number[i]
        
        for j in range(len(self.duplicate_index)):
            if i == self.duplicate_index[j][0]:
                del self.duplicate_index[j][0]
                del self.duplicate_index[j][1]

    def timer_callback(self):
        status = Bool()
        if len(self.x) > 0:
            status.data = True
            self.fruit_status_publisher_.publish(status)
        
        else:
            status.data = False
            self.fruit_status_publisher_.publish(status)



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