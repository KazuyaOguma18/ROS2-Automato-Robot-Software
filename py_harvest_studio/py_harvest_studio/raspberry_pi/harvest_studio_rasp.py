'''
ポット把持命令orポット回転命令を受信したらSTM32にデジタル信号を送信する
周期的に角度情報を送信する
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from sensor_msgs.msg import JointState

import RPi.GPIO as GPIO
import serial

class HarvestStudioRasp(Node):
    def __init__(self):
        super().__init__('harvest_studio_rasp')
        self.fruit_status_subscriber = self.create_subscription(Int16, 'fruit_detect_status', self.fruit_status_callback)
        self.jointstate_publisher = self.create_publisher(JointState, 'arm_joint_state', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.mode = 0
        self.data_array = [0,0,0]
        
        # Raspberry Pi のセットアップ
        self.roatate_pin = 22
        self.grasp_pin = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.roatate_pin, GPIO.OUT)
        GPIO.setup(self.grasp_pin, GPIO.OUT)
        
        # シリアル通信の設定
        self.serial = serial.Serial("/dev/ttyACM0", 115200, timeout=0.1)
        
        
    # STM32の動作モードによって出力ピンを決定する
    def fruit_status_callback(self, data):
        if self.mode == 0 or self.mode == 4:
            if data.data == 0:
                GPIO.output(self.grasp_pin, True)
            
            else:
                GPIO.output(self.grasp_pin, False)
                
        elif self.mode == 3:
            if data.data == 0:
                GPIO.output(self.roatate_pin, True)
            
            else:
                GPIO.output(self.roatate_pin, False) 
                
        else:
            pass           

            
    def timer_callback(self):
        line = self.serial.readline().strip()
        line_s = line.split(",")
        
        try:
            if len(line_s) > 2:
                self.data_array = [float(s) for s in line_s]
                
                # 角度情報をpublish
                jointstate = JointState()
                jointstate.name.append('right_arm_joint')
                jointstate.name.append('left_arm_joint')
                
                jointstate.position.append(s[0])
                jointstate.position.append(s[1])
                
                self.jointstate_publisher.publish(jointstate)
                
                # 現在のモード情報を取得
                self.mode = int(s[2])
                
        except:
            pass
                  
            
def main(args=None):
    try:
        rclpy.init(args=args)
        processsor = HarvestStudioRasp()
        rclpy.spin(processsor)

    except KeyboardInterrupt:
        pass
    finally:
        processsor.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()