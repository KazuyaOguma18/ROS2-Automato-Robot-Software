/*
回転把持機構とカメラ角の制御を主に行うノード
現在の収穫対象果実の保持数が０個が5回続く
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

#define STUDIO_SETUP 0
#define STUDIO_END 1
#define STUDIO_RORATE 2

using std::placeholders::_1;

double to_radian(double degree){
    return degree*M_PI/180;
}

void HarvestStudioControl::detect_status_callback(const std_msgs::msg::Int16::SharedPtr status){
    static int previous_rotate_mode = 0;

    jointstate.name.clear();
    jointstate.position.clear();
    jointstate.velocity.clear();
    jointstate.effort.clear();

    // ポットの回転モードに入ったとき、カメラ角度の動作を開始
    if (studio_mode == 0){
        std_msgs::msg::Int16 studio_signal;
        studio_signal.data = STUDIO_SETUP;
        
    }
    
    else if (studio_mode == 3){
        if (angle_value >= 0.0){
            angle_value = to_radian(-30.0);
        }

        // ポットの回転モードが変化したらカメラ位置を初期化する
        if (previous_rotate_mode != pot_rotate_mode){
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

    else{
        angle_value = -0.5;
        jointstate.header.stamp = rclcpp::Clock().now();
        jointstate.name.push_back("azure_camera_joint");
        jointstate.position.push_back(angle_value);
        jointstate_pub->publish(jointstate);
    }

    previous_rotate_mode = pot_rotate_mode;    
}

void HarvestStudioControl::studio_mode_callback(const std_msgs::msg::Int16MultiArray::SharedPtr mode_array){
    studio_mode = mode_array->data[0];
    pot_rotate_mode = mode_array->data[1];
    is_rotating = mode_array->data[2];
}

HarvestStudioControl::HarvestStudioControl(
    const rclcpp::NodeOptions& options
): HarvestStudioControl("", options){}

HarvestStudioControl::HarvestStudioControl(
    const std::string& name_space,
    const rclcpp::Nodeoptions& options
): Node("harvest_studio_control", name_space, options){
    jointstate_pub = this->create_publisher<sensor_msgs::msg::JointState>("dyna_joint_command", rclcpp::Qos(10));
    
    studio_unit_pub = this->create_publisher<std_msgs::msg::Int16>("studio_control_signal");

    detect_status_sub = this->create_subscription<std_msgs::msg::Int16>(
        "fruit_detect_status",
        rclcpp::QoS(10),
        std::bind(&DynaAngleControl::detect_status_callback, this, _1));

    studio_mode_sub = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "studio_mode",
        rclcpp::QoS(10),
        std::bind(&DynaAngleControl::studio_mode_callback, this, _1));    
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HarvestStudioControl>());
    rclcpp::shutdown();
    return 0;
}