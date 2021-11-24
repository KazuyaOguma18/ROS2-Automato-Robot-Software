#! /usr/bin/env python3

""" 
進捗状況
・おおよそのコーディングは完了
・動作確認が終わっていない
"""

import serial
import rclpy
from rclpy import handle
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from harvest_studio_msg.srv import EndEffectorControl

# subscribe from generate_motion_point
# substitute global list from ros message

class HandRos2Serial(Node):

    def __init__(self):
        super().__init__('hand_ros2serial')
        # 動作モードの取得 : demo or real
        self.declare_parameter('hand_control_mode', 'demo')
        self.control_mode = self.get_parameter('hand_control_mode')


        if str(self.control_mode.value) == 'demo':
            self.srv = self.create_service(EndEffectorControl, 'hand_data', self.hand_data_fake_callback)

        elif str(self.control_mode.value) == 'real':
            self.srv = self.create_service(EndEffectorControl, 'hand_data', self.hand_data_real_callback)

            self.timer_ = self.create_timer(0.1, self.timer_callback)
            self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)

        self.goal_array = [0,0,0]
        self.current_array = [0,0,0]

        # シリアル通信が問題なくできてるか確認用
        self.i = 0
        self.j = 0

        # 目標値との誤差計算用
        self.tolerance = 0



    # ハンドのゴール更新＆現在の状況を返す
    def hand_data_fake_callback(self, request, response):
        self.goal_array[0] = request.hand
        self.goal_array[1] = request.cup
        self.goal_array[2] = request.pump

        response.status = True

        return response
              
    def hand_data_real_callback(self, request, response):
        self.goal_array[0] = request.hand
        self.goal_array[1] = request.cup
        self.goal_array[2] = request.pump

        response.status = self.calc_tolerance()

        return response

    # 周期性の制御    
    def timer_callback(self):
        print("reading...")
        line = self.ser.readline().strip()
        line_s = line.split(",")

        if len(line_s) > 1:
            self.current_array = [float(s) for s in line_s]
            print(str(self.current_array) + "-" + str(self.i) + "-")
            self.ser.write("data get\r\n")
            self.i = 0
            self.j = self.j + 1

        else:
            print("No data recieved -{}-".format(self.i))
            print("--------------------")
            self.ser.write("No data\n")
            self.i = self.i+1
            self.j = 0   

        # 誤差が0.1以内に収まったらハンドに送信するゴール状態を更新
        if self.calc_tolerance() == True:
            self.ser.write(str(self.goal_array[0]) + "," + str(self.goal_array[1]) + "," + str(self.goal_array[2])) 



    def calc_tolerance(self):
        for i in range(2):
            self.tolerance = abs(self.current_array[i] - self.goal_array[i])
        
        return True if self.tolerance < 0.1 else False

def main(args=None):
    try:
        rclpy.init(args=args)
        hand_ros2serial = HandRos2Serial()
        rclpy.spin(hand_ros2serial)

    except KeyboardInterrupt:
        pass

    finally:
        hand_ros2serial.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()