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

from std_msgs.msg import Float32MultiArray

# subscribe from generate_motion_point
# substitute global list from ros message

class HandRos2Serial(Node):

    def __init__(self):
        super().__init__('hand_ros2serial')
        self.subscriber_ = self.create_subscription(
            Float32MultiArray,
            'hand_goal',
            self.hand_goal_callback,
            10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)
        self.goal_array = [0,0,0]
        self.current_array = [0,0,0]
        self.i = 0
        self.j = 0
        self.tolerance = 0

    # ハンドのゴール状態の取得
    def hand_goal_callback(self, array_msg):
        for i in range(3):
            self.goal_array[i] = array_msg.data[i]

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
            self.j = j + 1

        else:
            print("No data recieved -{}-".format(self.i))
            print("--------------------")
            self.ser.write("No data\n")
            self.i = self.i+1
            self.j = 0   

        if self.calc_tolerance() == True:
            self.ser.write(str(self.goal_array[0]) + "," + str(self.goal_array[1]) + "," + str(self.goal_array[2])) 



    def calc_tolerance(self):
        for i in range(2):
            self.tolerance = (self.current_array[i] - self.goal_array[i])**2
        
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