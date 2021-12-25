'''
ポット把持命令orポット回転命令を受信したらSTM32にデジタル信号を送信する
周期的に角度情報を送信する
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState

import RPi.GPIO as GPIO
import serial
import time
import math

class HarvestStudioRasp(Node):
    def __init__(self):
        super().__init__('harvest_studio_rasp')
        self.fruit_status_subscriber = self.create_subscription(Int16, 'fruit_detect_status', self.fruit_status_callback, 10)
        self.studio_control_signal_subscriber = self.create_subscription(Int16, 'studio_control_signal', self.studio_control_signal_callback, 10)
        self.jointstate_publisher = self.create_publisher(JointState, 'arm_joint_state', 10)
        self.mode_publisher = self.create_publisher(Int16MultiArray, 'studio_mode', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.serial_timer_ = self.create_timer(0.01, self.serial_timer_callback)
        self.studio_mode = 0
        self.rotate_mode = 0
        self.is_rotation = 0
        self.data_array = [0,0,0]
        self.right_joint_state = 0.
        self.left_joint_state = 0.
        self.zero_count = 0
        
        # Raspberry Pi のセットアップ
        self.rotate_pin = 23
        self.grasp_pin = 22
        self.reset_pin = 24
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.rotate_pin, GPIO.OUT)
        GPIO.setup(self.grasp_pin, GPIO.OUT)
        GPIO.setup(self.reset_pin, GPIO.OUT)

        # 把持・回転機構の制御信号
        self.studio_control_signal = 0
        self.studio_control_signal_recieved = False
        
        # シリアル通信の設定
        self.serial = serial.Serial("/dev/ttyACM0", 115200, timeout=0.1)

        # STM32のリセット
        self.reset_stm32()

    # 把持・回転機構の制御信号のコールバック
    def studio_control_signal_callback(self, msg):
        # 0: setup
        # 1: end
        # 2: rotate
        self.studio_control_signal_recieved = True
        self.studio_control_signal = msg.data

    
    # STM32の動作モードによって出力ピンを決定する
    def fruit_status_callback(self, data):
        if self.studio_mode == 0:
            if self.studio_control_signal == 0:
                GPIO.output(self.grasp_pin, True)
                GPIO.output(self.rotate_pin, False)
                
        elif self.studio_mode == 4:
            if self.studio_control_signal == 1:
                GPIO.output(self.grasp_pin, True)
                GPIO.output(self.rotate_pin, True)        
                

        elif self.studio_mode == 3:
            if self.studio_control_signal == 2:
                GPIO.output(self.grasp_pin, False)
                GPIO.output(self.rotate_pin, True)
                    
            else:
                GPIO.output(self.grasp_pin, False)
                GPIO.output(self.rotate_pin, False) 
                
        else:
            pass          
         
        self.get_logger().info("detect_status received") 
        
    def serial_timer_callback(self):
        try:
            line = self.serial.readline().strip().decode('utf-8')
            line_s = line.split(",")
            if len(line_s) > 3:
                self.current_data = [float(s) for s in line_s]
                # self.get_logger().info(str(self.current_data))   

                # print(str(self.current_data))
                self.right_joint_state = math.radians(self.current_data[0])
                self.left_joint_state = math.radians(self.current_data[1])
                self.studio_mode = int(self.current_data[2])
                self.rotate_mode = int(self.current_data[3])
                self.is_rotation = int(self.current_data[4])


            else:
                print("No data recieved")
        except Exception as err:
            print("[demo] exception has occured: {}".format(err))
            
    def timer_callback(self):

        # 角度情報を配信
        jointstate = JointState()
        jointstate.name.append('right_arm_joint')
        jointstate.name.append('left_arm_joint')

        jointstate.position.append(self.right_joint_state)
        jointstate.position.append(self.left_joint_state)

        self.jointstate_publisher.publish(jointstate)

        # 現在の把持回転機構の動作モードを配信
        mode_array = []
        mode_array.append(self.studio_mode)
        mode_array.append(self.rotate_mode)
        mode_array.append(self.is_rotation)
        
        mode_pub = Int16MultiArray(data = mode_array)
        self.mode_publisher.publish(mode_pub)
        
        print("now rotating: " + str(self.is_rotation))
        GPIO.output(self.grasp_pin, False)
        GPIO.output(self.rotate_pin, False)

    def reset_stm32(self):
        # STM32のリセット
        GPIO.output(self.reset_pin, False)
        time.sleep(0.1)
        GPIO.output(self.reset_pin, True) 
        GPIO.output(self.rotate_pin, False)
        GPIO.output(self.grasp_pin, False)

    def kill_stm32(self):
        GPIO.output(self.reset_pin, False) 
        GPIO.output(self.rotate_pin, False)
        GPIO.output(self.grasp_pin, False)

def main(args=None):
    global processor
    try:
        rclpy.init(args=args)
        processor = HarvestStudioRasp()
        rclpy.spin(processor)

    except KeyboardInterrupt:
        pass
    
    finally:
        processor.kill_stm32()
        processor.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
