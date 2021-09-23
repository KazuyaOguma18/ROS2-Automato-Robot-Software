/*
進捗状況
・コードの花形の作成は完了
・publishの型をdynamixel sdk仕様に変更する必要あり
・型によってpublish関連のコードを作成していく
*/



#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include "camera2dynamixel.hpp"


void Camera2Dynamixel::monitor_jointstate_callback(const sensor_msgs::msg::JointState::ConstPtr& jointstates){
    for (int i=0; i<jointstates->position.size(); i++){
        if(jointstates->name[i] == "camera_joint"){
            if(jointstates->position.size() > 0){
                joint_pos.data = jointstates->position[i];
            }
            if(jointstates->velocity.size() > 0){
                joint_vel.data = jointstates->velocity[i];
            }
            if(jointstates->effort.size() > 0){
                joint_eff.data = jointstates->effort[i];
            }
        }
    }    
}

Camera2Dynamixel::Camera2Dynamixel(
    const rclcpp::NodeOptions& options
): Camera2Dynamixel("", options){}

Camera2Dynamixel::Camera2Dynamixel(
    const std::string& name_space="",
    const rclcpp::NodeOptions& options
): Node("camera2dynamixel", name_space, options){
    sub_joints = this->create_subscription<sensor_msgs::msg::JointState::ConstPtr>(
        "/dyna_joint_command",
        rclcpp::QoS(10),
        std::bind(&Camera2Dynamixel::monitor_jointstate_callback, this, std::placeholders::_1)
    );
    arm_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pupupu", rclcpp::QoS(10));
    timer_ = this->create_wall_timer(
        100ms,
        [this](){
            auto msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
            /*msgに代入する処理系を入力していく*/


            arm_pub->publish(*msg);
        }
    );
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Camera2Dynamixel>());
    rclcpp::shutdown();
    return 0;
}