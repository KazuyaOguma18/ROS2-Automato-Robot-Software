import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ArmSamplePublisher(Node):
    def __init__(self):
        super().__init__('arm_sample_publisher')
        self.publisher = self.create_publisher(JointState, 'arm_joint_state', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.angle = -1.0
        self.mode = 1

    def timer_callback(self):
        jointstate = JointState()
        jointstate.name.append('right_arm_joint')
        jointstate.name.append('left_arm_joint')

        if (self.mode == 1):
            self.angle += 0.02
        elif (self.mode == -1):
            self.angle -= 0.02

        if (self.angle > 1.0):
            self.mode = -1
        elif (self.angle < -1.0):
            self.mode = 1

        jointstate.position.append(self.angle)
        jointstate.position.append(self.angle)

        self.publisher.publish(jointstate)



def main(args=None):
    try:
        rclpy.init(args=args)
        processsor = ArmSamplePublisher()
        rclpy.spin(processsor)

    except KeyboardInterrupt:
        pass
    finally:
        processsor.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()