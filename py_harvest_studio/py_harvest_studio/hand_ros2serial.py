# ハンドへの信号をシリアル化して行うノード
# ros2 service call /hand_data harvest_studio_msg/srv/EndEffectorControl "{hand: 10.0, cup: 10.0, pump: 0.0}"

import serial
import rclpy
from rclpy import handle
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from harvest_studio_msg.srv import EndEffectorControl
from std_msgs.msg import Int32MultiArray

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
            self.serial_publisher = self.create_publisher(Int32MultiArray, 'hand_current_status', 10)
            self.timer_ = self.create_timer(0.01, self.timer_callback)
            self.ser = serial.Serial("/dev/ttyUSB-XBee", 115200, timeout=0.1)
            

        self.goal_array = [0,0,0]
        self.current_array = [0,0]
        self.previous_array = [0,0]

        # シリアル通信が問題なくできてるか確認用
        self.hand_status = True
        self.i = 0
        self.j = 0

        # 目標値との誤差計算用
        self.tolerance = 0
        
        print("control mode: {}".format(self.control_mode.value))



    # ハンドのゴール更新＆現在の状況を返す
    def hand_data_fake_callback(self, request, response):
        self.goal_array[0] = request.cup
        self.goal_array[1] = request.hand
        self.goal_array[2] = request.pump
        
        print(self.goal_array)

        response.status = True

        return response
              
    def hand_data_real_callback(self, request: EndEffectorControl.Request, response: EndEffectorControl.Response):
        self.goal_array[0] = request.cup
        self.goal_array[1] = request.hand
        self.goal_array[2] = request.pump

        # 角度変化が0.1度未満になったらTrueを返す
        response.status = self.calc_tolerance(0.1)
        if self.hand_status == False:
            response.status = False
        
        for i in range(len(self.previous_array)):
            self.previous_array[i] = self.current_array[i]
            
        if response.status:
            try:
                senddata = bytes(str(self.goal_array[0]) + "," + str(self.goal_array[1]) + "," + str(self.goal_array[2]), encoding='utf-8')
                self.ser.write(senddata) 
            except Exception as err:
                pass
            
        return response

    # 周期性の制御
    def timer_callback(self):        
        pub_array = [int(s) for s in self.current_array]
        pub_data = Int32MultiArray(data = pub_array)
        self.serial_publisher.publish(pub_data)
        # print("reading...")
        
        try:
            line = self.ser.readline().strip().decode('utf-8')
            line_s = line.split(",")
            line_s.remove('')
            # print(line_s)

            if len(line_s) > 1:
                self.current_array = [float(s) for s in line_s]
                print(str(self.current_array) + "-" + str(self.j) + "-")
                # self.ser.write("data get\r\n")
                self.i = 0
                self.j = self.j + 1
                self.hand_status = True

            else:
                print("No data recieved -{}-".format(self.i))
                print("--------------------")
                # self.ser.write("No data\n")
                self.i = self.i+1
                self.j = 0     
            
        except Exception as err:
            print("exception has occured: {}".format(err))     
            
        if self.i > 100:
            self.hand_status = False
            
        if self.hand_status == False:
            self.get_logger().info("hand communication broken!") 
            



    # 二つのモータでどちらかが指定角度以上の変化があった場合True
    def calc_tolerance(self, tolerance):
        differ = 0
        for i in range(2):
            if differ < abs(self.current_array[i] - self.goal_array[i]):
                differ = abs(self.current_array[i] - self.goal_array[i])
        
        return True if differ < tolerance else False

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