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
        self.harvest_target_publisher_ = self.create_publisher(FruitDataList, 'harvest_target', 10)
        self.list_subscriber_ = self.create_subscription(FruitDataList , 'fruit_detect_list', self.fruit_detect_list_callback, 10)

        self.timer = self.create_timer(0.5, self.timer_callback)

        # 果実情報を配列に代入
        self.x = []
        self.y = []
        self.z = []
        self.radius = []
        self.detect_number = []

        self.harvest_list = FruitDataList()

        # 収穫を行う果実
        self.harvest_x = []
        self.harvest_y = []
        self.harvest_z = []



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

        # self.get_logger().info("self duplicate number :" + str(len(self_duplicate_index)))
        
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
            l.append([abs(c*self.z[i] - a*self.y[i])/math.sqrt(a**2+c**2), int(i)])

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
        try:
            # 前回収穫した果実のデータを消去
            # このタイミングに消去することで、収穫された果実が収穫用データに入ってしまう問題を解決
            for i in range(len(self.x)):
                if math.sqrt(pow(self.harvested_x - self.x[i], 2)+pow(self.harvested_y - self.y[i], 2)+pow(self.harvested_z - self.z[i], 2)) < self.harvested_radius:
                    harvest_index = i
                    break

            self.delete_fruit_data(harvest_index)

        except:
            pass

        zero_data = False
        if request.order == True and len(self.harvest_x) > 0:
            harvest_index = 0

            # 五回以上検出された信頼性のある果実から収穫
            for j in range(len(self.detect_number)):
                if self.detect_number[j] > 5:
                    harvest_index = j
                    zero_data = True
                    break

            if len(self.harvest_x) >= harvest_index+1:
                responce.x = self.harvest_x[harvest_index]
                responce.y = self.harvest_y[harvest_index]
                responce.z = self.harvest_z[harvest_index]
                responce.radius = self.harvest_radius[harvest_index]
                responce.success = True

                harvest_target = FruitDataList()
                harvest_target.x.append(self.harvest_x[harvest_index])
                harvest_target.y.append(self.harvest_y[harvest_index])
                harvest_target.z.append(self.harvest_z[harvest_index])
                harvest_target.radius.append(self.harvest_radius[harvest_index])

                self.harvest_target_publisher_.publish(harvest_target)

    
                # 収穫データとして送信されたデータを保存
                self.harvested_x = harvest_target.x[0]
                self.harvested_y = harvest_target.y[0]
                self.harvested_z = harvest_target.z[0]
                self.harvested_radius = harvest_target.radius[0]
                
                self.get_logger().info("buffer x len :" + str(len(self.x)))


        elif request.order == False or zero_data == True:
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

        delete_index = []

        for j in range(len(self.duplicate_index)):
            try:
                if i == self.duplicate_index[j][0]:
                    delete_index.append(j)
            except:
                self.get_logger().info("nazo error")

        sorted_delete_index = sorted(delete_index, reverse=True)
        for j in sorted_delete_index:        
            self.get_logger().info("delete data :{0}".format(len(self.duplicate_index)))
            del self.duplicate_index[j][1]
            del self.duplicate_index[j][0]

    def timer_callback(self):
        status = Bool()
        if len(self.x) > 0:
            status.data = True
            self.fruit_status_publisher_.publish(status)
        
        else:
            status.data = False
            self.fruit_status_publisher_.publish(status)

        self.harvest_list = FruitDataList()

        # 実際に収穫を行う果実の情報を出力
        for i in range(len(self.harvest_x)):
            self.harvest_list.x.append(self.harvest_x[i])
            self.harvest_list.y.append(self.harvest_y[i])
            self.harvest_list.z.append(self.harvest_z[i])
            self.harvest_list.radius.append(self.harvest_radius[i])

        self.harvest_list_publisher_.publish(self.harvest_list)



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