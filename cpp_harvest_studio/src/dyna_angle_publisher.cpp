#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include "cpp_harvest_studio/dyna_angle_publisher.hpp"
using namespace std::literals::chrono_literals;

void DynaAnglePublisher::timer_callback(){
    sensor_msgs::msg::JointState jointstate;
    jointstate.name.clear();
    jointstate.position.clear();
    jointstate.velocity.clear();
    jointstate.effort.clear();

    jointstate.header.stamp = rclcpp::Clock().now();
    jointstate.name.push_back("azure_camera_joint");

    if (angle_value > 0.5){
        angle_direction = -1;
    }
    else if (angle_value < -0.5){
        angle_direction = 1;
    }

    angle_value += 0.001*angle_direction;
    jointstate.position.push_back(angle_value);

    pub->publish(jointstate);
}

DynaAnglePublisher::DynaAnglePublisher(
    const rclcpp::NodeOptions& options
): DynaAnglePublisher("", options){}

DynaAnglePublisher::DynaAnglePublisher(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("dyna_angle_publisher", name_space, options){
    pub = this->create_publisher<sensor_msgs::msg::JointState>("dyna_joint_command", rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
        10ms,
        std::bind(&DynaAnglePublisher::timer_callback, this));
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynaAnglePublisher>());
    rclcpp::shutdown();
    return 0;
}