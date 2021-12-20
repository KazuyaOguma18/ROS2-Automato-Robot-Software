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

class HarvestStudioRasp(Node):
    def __init__(self):
        super().__init__('harvest_studio_rasp')
        self.fruit_status_subscriber = self.create_subscription(Int16, 'fruit_detect_status', self.fruit_status_callback)
        self.jointstate_publisher = self.create_publisher(JointState, 'arm_joint_state', 10)
        self.mode_publisher = self.create_publisher(Int16MultiArray, 'studio_mode', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.studio_mode = 0
        self.data_array = [0,0,0]
        self.right_joint_state = 0.
        self.left_joint_state = 0.
        
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
        if self.studio_mode == 0 or self.studio_mode == 4:
            if data.data == 0:
                GPIO.output(self.grasp_pin, True)
            
            else:
                GPIO.output(self.grasp_pin, False)
                
        elif self.studio_mode == 3:
            if data.data == 0:
                GPIO.output(self.roatate_pin, True)
            
            else:
                GPIO.output(self.roatate_pin, False) 
                
        else:
            pass           

    def serial_timer_callback(self):
        try:
            line = self.serial.readline().strip().decode('utf-8')
            line_s = line.split(",")
            if len(line_s) > 3:
                self.current_data = [float(s) for s in line_s]
                print(str(self.current_data))

                self.right_joint_state = s[0]
                self.left_joint_state = s[1]
                self.studio_mode = int(s[2])
                self.rotate_mode = int(s[3])


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
        mode = Int16MultiArray()
        mode.data.append(self.studio_mode)
        mode.data.append(self.rotate_mode)

        self.mode_publisher(mode)
                  
            
def main(args=None):
    try:
        rclpy.init(args=args)
        processsor = HarvestStudioRasp()
        rclpy.spin(processsor)

    except KeyboardInterrupt:
        pass
    finally:
        GPIO.output(22, False)
        GPIO.output(23, False)
        processsor.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()