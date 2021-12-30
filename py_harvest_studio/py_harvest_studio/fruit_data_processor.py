from os import stat
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Int32MultiArray
from harvest_studio_msg.msg import FruitDataList
from harvest_studio_msg.srv import FruitPositionData

import math


class FruitDataProcessor(Node):

    def __init__(self):
        super().__init__('fruit_data_processor')
        
        # 動作モードの取得 : with_grasp or without_grasp
        self.declare_parameter('grasp_mode', 'without_grasp')
        grasp_mode = self.get_parameter('grasp_mode')
        
        # 各種通信系の定義
        ## generate_motion_pointに果実位置を送信
        self.position_service_ = self.create_service(FruitPositionData, 'fruit_position_data', self.fruit_server_callback)
        ## 現在の果実のデータ保持状況を送信
        self.fruit_status_publisher_ = self.create_publisher(Int16, 'fruit_detect_status', 10)
        ## 現在の果実データ一覧を送信
        self.harvest_list_publisher_ = self.create_publisher(FruitDataList, 'harvest_list', 10)
        ## generate_motion_pointに送信した果実位置を公開
        self.harvest_target_publisher_ = self.create_publisher(FruitDataList, 'harvest_target', 10)
        ## tomato_detectorから検出された果実位置を受信
        self.list_subscriber_ = self.create_subscription(FruitDataList , 'fruit_detect_list', self.fruit_detect_list_callback, 10)
        ## 現在の把持回転機構のモードを受信
        self.studio_mode_subscriber_ = self.create_subscription(Int32MultiArray, 'studio_mode', self.studio_mode_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        # 果実情報を配列に代入
        self.x = []
        self.y = []
        self.z = []
        self.radius = []
        self.detect_number = []

        self.harvest_list = FruitDataList()

        # 収穫を行う果実
        self.harvest = FruitData()
        # self.harvest_x = []
        # self.harvest_y = []
        # self.harvest_z = []
        
        # 以前のポット回転モード
        self.previous_rotate_mode = 0
        
        # 収穫スタジオの動作モード
        '''
        0: 初期モード (把持角度0度、信号待機)
        1: 測距モード (把持角度0度、測距センサが閾値を超えるまで待機)
        2: 把持モード (アーム動作、接触判定が起こるまで)
        3: ポット回転モード (ポット回転、1/4回転ごとに制御)
        4: 解除モード (特に制御なし)
        '''
        self.studio_mode = 0
        if str(grasp_mode.value) == 'without_grasp':
            self.studio_mode = 3      
        
        # ポットが回転中かどうか取得
        self.is_rotation = 0
        
        # 収穫用データが更新されなくなったカウント
        self.not_harvest_count = 0
        self.previous_data_len = 0



    # 検出された果実情報のコールバック
    # 重複果実のチェック、果実座標の平均取得、果実位置の線形変換＆平面距離取得＆ソート
    def fruit_detect_list_callback(self, msg):

        if self.studio_mode == 3 and self.is_rotation == 0:
            self.duplicate_index = []

            self.msg = FruitData()
            # 新規果実が奥すぎないかチェック
            for i in range(len(msg.x)):
                if msg.x[i] < 0.7 and msg.x[i] > 0.3:
                    self.msg.x.append(msg.x[i])
                    self.msg.y.append(msg.y[i])
                    self.msg.z.append(msg.z[i])
                    self.msg.radius.append(msg.radius[i])         
                       
            # self.msg_x = msg.x
            # self.msg_y = msg.y
            # self.msg_z = msg.z
            # self.msg_radius = msg.radius
            try:
                self.check_duplicate_fruits(self.msg)
                # self.get_logger().info("check_duplicate_fruits")
                self.get_fruit_position_average()
                # self.get_logger().info("fruit_position_average")
                self.fruit_position_linear_tf()
                # self.get_logger().info("fruit_position_linear_tf")
            except Exception as err:
                self.get_logger().info("processor error : {}".format(err))      
                     
            self.harvest.x = self.x
            self.harvest.y = self.y
            self.harvest.z = self.z
            self.harvest.radius = self.radius
            
            # self.harvest_x = self.x
            # self.harvest_y = self.y
            # self.harvest_z = self.z
            # self.harvest_radius = self.radius
        
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
        # not_duplicate_x = []
        # not_duplicate_y = []
        # not_duplicate_z = []
        # not_duplicate_radius = []
        # not_duplicate_detect_number = []
        self_duplicate_index = []
        for i in range(len(self.x)):
            duplicate = False
            for j in range(i+1, len(self.x)):
                if math.sqrt((self.x[i] - self.x[j])**2 + (self.y[i] - self.y[j])**2 + (self.z[i] - self.z[j])**2) < self.radius[i]:
                    duplicate = True
            if  duplicate:
                # not_duplicate_x.append(self.x[i])
                # not_duplicate_y.append(self.y[i])
                # not_duplicate_z.append(self.z[i])
                # not_duplicate_radius.append(self.radius[i])
                # not_duplicate_detect_number.append(self.detect_number[i])
                
                self_duplicate_index.append(i)

        # self.get_logger().info("self duplicate number :" + str(len(self_duplicate_index)))
        

        for i in self_duplicate_index:
            try:
                self.delete_fruit_data(i)
                print("deleted")
            except Exception as err:
                print("error : {}".format(err))

    # 同じ果実と判定された果実の平均座標の取得
    def get_fruit_position_average(self):
        # 同じデータ番号を簡単に見つけるためにソート
        self.duplicate_index = sorted(self.duplicate_index)

        
        # 最初のデータにこれまでの検出分を足す
        if len(self.duplicate_index) > 0 and len(self.x) > 0:
            self.x[self.duplicate_index[0][0]] += (self.detect_number[self.duplicate_index[0][0]] - 1)*self.x[self.duplicate_index[0][0]] 
            self.y[self.duplicate_index[0][0]] += (self.detect_number[self.duplicate_index[0][0]] - 1)*self.y[self.duplicate_index[0][0]] 
            self.z[self.duplicate_index[0][0]] += (self.detect_number[self.duplicate_index[0][0]] - 1)*self.z[self.duplicate_index[0][0]] 
            self.radius[self.duplicate_index[0][0]] += (self.detect_number[self.duplicate_index[0][0]] - 1)*self.radius[self.duplicate_index[0][0]]
            
        
        # ダミーデータ（エラー防止用）
        self.duplicate_index.append([1000,0])
                
        # self.get_logger().info("self.detect_number[self.duplicate_index[0][0]] - 1: {}".format(self.detect_number[self.duplicate_index[0][0]] - 1))
        # self.get_logger().info("self.duplicate_index[0][0]: {}".format((self.duplicate_index[0][0])))
        # self.get_logger().info("self.x[self.duplicate_index[0][0]]: {}".format((self.x[self.duplicate_index[0][0]])))

        # ダミー以外の回数分繰り返す
        # self.get_logger().info("len(self.duplicate_index) - 1: {}".format(len((self.duplicate_index))-1))
        for i in range(len(self.duplicate_index) - 1):

            if self.duplicate_index[i][0] != self.duplicate_index[i+1][0]:
                self.x[self.duplicate_index[i][0]] = self.x[self.duplicate_index[i][0]] / self.detect_number[self.duplicate_index[i][0]]
                self.y[self.duplicate_index[i][0]] = self.y[self.duplicate_index[i][0]] / self.detect_number[self.duplicate_index[i][0]]
                self.z[self.duplicate_index[i][0]] = self.z[self.duplicate_index[i][0]] / self.detect_number[self.duplicate_index[i][0]]
                self.radius[self.duplicate_index[i][0]] = self.radius[self.duplicate_index[i][0]] / self.detect_number[self.duplicate_index[i][0]]
                self.detect_number[self.duplicate_index[i][0]] += 1

                if i < len(self.duplicate_index) - 2:
                    self.x[self.duplicate_index[i+1][0]] += (self.detect_number[self.duplicate_index[i+1][0]] - 1)*self.x[self.duplicate_index[i+1][0]]
                    self.y[self.duplicate_index[i+1][0]] += (self.detect_number[self.duplicate_index[i+1][0]] - 1)*self.y[self.duplicate_index[i+1][0]]
                    self.z[self.duplicate_index[i+1][0]] += (self.detect_number[self.duplicate_index[i+1][0]] - 1)*self.z[self.duplicate_index[i+1][0]]
                    self.radius[self.duplicate_index[i+1][0]] += (self.detect_number[self.duplicate_index[i+1][0]] - 1)*self.radius[self.duplicate_index[i+1][0]]
                    

            else:
                self.x[self.duplicate_index[i][0]] += self.msg.x[self.duplicate_index[i][1]]
                self.y[self.duplicate_index[i][0]] += self.msg.y[self.duplicate_index[i][1]]
                self.z[self.duplicate_index[i][0]] += self.msg.z[self.duplicate_index[i][1]]
                self.radius[self.duplicate_index[i][0]] += self.msg.radius[self.duplicate_index[i][1]]
                self.detect_number[self.duplicate_index[i][0]] += 1
                
        del self.duplicate_index[-1]
        # self.get_logger().info("self.x[self.duplicate_index[0][0]]: {}".format((self.x[self.duplicate_index[0][0]])))
        
            
                

    # 任意平面からの距離に基づく、果実位置のソート
    def fruit_position_linear_tf(self):
        # 平面の方程式の定義
        # 平面を(1,0,0), (0,c,a)のベクトルから平面を求める
        c = 2
        a = 1
        l = []
        for i in range(len(self.x)):
            l.append([abs(c*self.z[i] + a*self.x[i])/math.sqrt(a**2+c**2), int(i)])

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
        if request.order == True and len(self.harvest.x) > 0:
            harvest_index = 0

            # 10回以上検出された信頼性のある果実から収穫
            for j in range(len(self.detect_number)):
                if self.detect_number[j] > 10:
                    harvest_index = j
                    zero_data = True
                    break

            if len(self.harvest.x) >= harvest_index+1:
                responce.x = self.harvest.x[harvest_index]
                responce.y = self.harvest.y[harvest_index]
                responce.z = self.harvest.z[harvest_index]
                responce.radius = self.harvest.radius[harvest_index]
                responce.success = True

                harvest_target = FruitDataList()
                harvest_target.x.append(self.harvest.x[harvest_index])
                harvest_target.y.append(self.harvest.y[harvest_index])
                harvest_target.z.append(self.harvest.z[harvest_index])
                harvest_target.radius.append(self.harvest.radius[harvest_index])

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

        if self.previous_data_len == len(self.x):
            # 果実の情報が収穫データに移行しなくなったら5カウント後初期化
            if self.not_harvest_count > 5:
                self.not_harvest_count = 0
                self.initialize()
                
            self.not_harvest_count += 1
        
        self.previous_data_len = len(self.x)
        
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
            except Exception as err:
                self.get_logger().info("delete data error : {}".format(err))

        sorted_delete_index = sorted(delete_index, reverse=True)
        for j in sorted_delete_index:        
            self.get_logger().info("delete data index:{0}".format(len(self.duplicate_index)))
            del self.duplicate_index[j][1]
            del self.duplicate_index[j][0]

        # self.get_logger().info("delete fruit data")

    def timer_callback(self):
        status = Int16()
        status.data = len(self.x)

        self.fruit_status_publisher_.publish(status)

        self.harvest_list = FruitDataList()

        # 実際に収穫を行う果実の情報を出力
        for i in range(len(self.harvest.x)):
            self.harvest_list.x.append(self.harvest.x[i])
            self.harvest_list.y.append(self.harvest.y[i])
            self.harvest_list.z.append(self.harvest.z[i])
            self.harvest_list.radius.append(self.harvest.radius[i])
            # self.get_logger().info("detect_number[{}]: {}".format(i, self.detect_number[i]))

        self.harvest_list_publisher_.publish(self.harvest_list)
    
    # ポットの回転モードが変化したら配列を初期化   
    def studio_mode_callback(self, msg):
        if self.previous_rotate_mode != msg.data[1] or msg.data[2] != 0:
            self.initialize()
            
        else:
            pass
        
        self.previous_rotate_mode = msg.data[1]
        self.studio_mode = msg.data[0]
        self.is_rotation = msg.data[2]

    def initialize(self):
        # 果実情報を配列に代入
        self.x = []
        self.y = []
        self.z = []
        self.radius = []
        self.detect_number = []

        # 収穫を行う果実
        self.harvest.x = []
        self.harvest.y = []
        self.harvest.z = []   
        self.harvest.radius = []     

# 果実情報のコンストラクタ
class FruitData:
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []
        self.radius = []


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