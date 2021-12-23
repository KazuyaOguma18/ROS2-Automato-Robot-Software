/*
回転把持機構とカメラ角の制御を主に行うノード
現在の収穫対象果実の保持数が０個
↓
カメラ角の制御　（下から上まで３段階）
↓
ポットの回転制御
*/
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include "cpp_harvest_studio/harvest_studio_control.hpp"

using std::placeholders::_1;

HarvestStudioControl::HarvestStudioControl(
    const rclcpp::NodeOptions& options
): HarvestStudioControl("", options){}

HarvestStudioControl::HarvestStudioControl(
    const std::string& name_space,
    const rclcpp::Nodeoptions& options
): Node("harvest_studio_control", name_space, options){
    jointstate_pub = this->create_publisher<sensor_msgs::msg::JointState>("dyna_joint_command", rclcpp::Qos(10));
    
}