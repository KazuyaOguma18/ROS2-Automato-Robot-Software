#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int16.hpp>
#include "cpp_harvest_studio/dyna_angle_control.hpp"

using std::placeholders::_1;

void DynaAngleControl::detect_status_callback(const std_msgs::msg::Int16::SharedPtr status){
    jointstate.name.clear();
    jointstate.position.clear();
    jointstate.velocity.clear();
    jointstate.effort.clear();

    if (angle_value >= 0.0){
        angle_value = -0.5;
    }

    if (status->data < 2){
        jointstate.header.stamp = rclcpp::Clock().now();
        jointstate.name.push_back("azure_camera_joint");

        jointstate.position.push_back(angle_value);

        jointstate_pub->publish(jointstate);
        angle_value += 0.25;
    }
}

DynaAngleControl::DynaAngleControl(
    const rclcpp::NodeOptions& options
): DynaAngleControl("", options){}

DynaAngleControl::DynaAngleControl(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("harvest_studio_control", name_space, options){
    jointstate_pub = this->create_publisher<sensor_msgs::msg::JointState>("dyna_joint_command", rclcpp::QoS(10));

    detect_status_sub = this->create_subscription<std_msgs::msg::Int16>(
        "fruit_detect_status",
        rclcpp::QoS(10),
        std::bind(&DynaAngleControl::detect_status_callback, this, _1));

    
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynaAngleControl>());
    rclcpp::shutdown();
    return 0;
}