import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import serial
import time

class HarvestStudioRasp(Node):
    def __init__(self):
        super().__init__('sample_rasp_stm32')
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.serial_timer = self.create_timer(0.01, self.serial_timer_callback)
        self.mode = True
        # Raspberry Pi のセットアップ
        self.roatate_pin = 22
        self.grasp_pin = 23
        self.reset_pin = 24
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.roatate_pin, GPIO.OUT)
        GPIO.setup(self.grasp_pin, GPIO.OUT)
        GPIO.setup(self.reset_pin, GPIO.OUT)
        # シリアル通信の設定
        self.serial = serial.Serial("/dev/ttyACM0", 115200, timeout=0.1)
        # シリアル通信用の配列の初期化
        self.current_data = [0,0]



    def serial_timer_callback(self):
        try:
            line = self.serial.readline().strip().decode('utf-8')
            line_s = line.split(",")
            if len(line_s) > 1:
                self.current_data = [int(s) for s in line_s]
                print(str(self.current_data))
            else:
                print("No data recieved")
        except Exception as err:
            print("[demo] exception has occured: {}".format(err))

    def timer_callback(self):
        if self.mode == True:
            GPIO.output(self.roatate_pin, True)
        else:
            GPIO.output(self.roatate_pin, False)
        if self.mode == False:
            GPIO.output(self.grasp_pin, True)
        else:
            GPIO.output(self.grasp_pin, False)
        self.mode = not self.mode
        # time.sleep(0.4)       

def main(args=None):
    try:
        rclpy.init(args=args)
        processor = HarvestStudioRasp()
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.output(22, False)
        GPIO.output(23, False)
        processor.destroy_node()
        rclpy.shutdown()
if __name__=='__main__':
    main()