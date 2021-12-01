/*
進捗状況
・一部関数除くほとんどのコーディング完了
・動作確認まだ
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include "harvest_studio_msg/srv/fruit_position_data.hpp"
#include "harvest_studio_msg/srv/end_effector_control.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

float x, y, z, radius;
bool success=false;

bool hand_status = false;

// エンドエフェクタの角度の生成を行う
void create_eef_motion(float hand_data[], float radius, float pump){
    for (int i = 0; i < 2; i++){
        hand_data[i] = 0.0*radius;
    }
    hand_data[2] = pump;
}

double to_radians(const double deg_angle){
  return deg_angle * M_PI / 180.0;
}

double calc_yaw(double x, double y){
    double yaw;
    if (x==0.0){
        if (y > 0.0){
            yaw = M_PI_2;
        }
        else if (y < 0.0){
            yaw = -M_PI_2;
        }
        else{
            yaw = 0.0;
        }
    }
    else{
        yaw = atan(y/x);
    }

    return yaw;
}

void arm_callback_response(rclcpp::Client<harvest_studio_msg::srv::FruitPositionData>::SharedFuture result){
    x = result.get()->x;
    y = result.get()->y;
    z = result.get()->z;
    radius = result.get()->radius;
    success = result.get()->success;
}

void hand_callback_response(rclcpp::Client<harvest_studio_msg::srv::EndEffectorControl>::SharedFuture result){
    hand_status = result.get()->status;
}

void hand_service(rclcpp::Client<harvest_studio_msg::srv::EndEffectorControl>::SharedPtr hand_client,
                  std::shared_ptr<harvest_studio_msg::srv::EndEffectorControl::Request> hand_request, float eef_data[]){
    while (!hand_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("GMP"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("GMP"), "hand service not available, waiting again...");
    }

    hand_request->hand = eef_data[0];
    hand_request->cup = eef_data[1];
    hand_request->pump = eef_data[2];
    
    while (!hand_status){
        auto result = hand_client->async_send_request(hand_request, std::bind(&hand_callback_response, std::placeholders::_1));
        hand_status = result.get()->status;
    }
    

}


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> move_node = rclcpp::Node::make_shared("generate_motion_point_arm", node_options);
    std::shared_ptr<rclcpp::Node> service_node = rclcpp::Node::make_shared("generate_motion_point_service", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_node);
    executor.add_node(service_node);
    std::thread([&executor]() {executor.spin(); }).detach();

    RCLCPP_INFO(rclcpp::get_logger("GMP"), "initialize MoveGroupInterface");
    MoveGroupInterface move_group(move_node,"xarm5");
    RCLCPP_INFO(rclcpp::get_logger("GMP"), "complete MoveGroupInterface");
    
    // アーム位置制御用サービス
    rclcpp::Client<harvest_studio_msg::srv::FruitPositionData>::SharedPtr arm_client = 
        service_node->create_client<harvest_studio_msg::srv::FruitPositionData>("fruit_position_data");

    auto arm_request = std::make_shared<harvest_studio_msg::srv::FruitPositionData::Request>();
    arm_request->order = true;

    // ハンド制御用サービス
    rclcpp::Client<harvest_studio_msg::srv::EndEffectorControl>::SharedPtr hand_client =
        service_node->create_client<harvest_studio_msg::srv::EndEffectorControl>("hand_data");

    auto hand_request = std::make_shared<harvest_studio_msg::srv::EndEffectorControl::Request>();

    /*
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_eef =
        service_node->create_publisher<std_msgs::msg::Float32MultiArray>("/hand_goal", rclcpp::QoS(10));
    */

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr fruit_target_command_pub =
        service_node->create_publisher<std_msgs::msg::Empty>("fruit_target_send_command", rclcpp::QoS(10));
    
    std_msgs::msg::Empty fruit_target_command;

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    move_group.setPlanningTime(0.5);
    auto joint_values = move_group.getCurrentJointValues();
    double GRIPPER_STATE1 = to_radians(-360);
    double GRIPPER_STATE2 = to_radians(360);
    geometry_msgs::msg::Pose target_pose;
    geometry_msgs::msg::PoseStamped target_pose_stamped;
    tf2::Quaternion q;

    std_msgs::msg::Float32MultiArray eef_data;

    // 初期姿勢
    RCLCPP_INFO(rclcpp::get_logger("GMP"), "Pose Initialize");  
    while (!move_group.setNamedTarget("hold-up")){
        continue;
    }
    while (!move_group.move()){
        continue;
    }
    

    
    double yaw;
    float hand_data[] = {0.0, 0.0, 0.0};


    while (rclcpp::ok()){

        // 果実位置情報の呼び出し
        fruit_target_command_pub->publish(fruit_target_command);
        while (!arm_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("GMP"), "Interrupted while waiting for the service. Exiting.");
              return 0;
            }
            RCLCPP_INFO(rclcpp::get_logger("GMP"), "service not available, waiting again...");
        }

        auto result = arm_client->async_send_request(arm_request, std::bind(&arm_callback_response, std::placeholders::_1));
        rclcpp::sleep_for(100ms);
        // Wait for the result.

        // RCLCPP_INFO(rclcpp::get_logger("GMP"), "C");

        // false(正しい値じゃない)場合、この先のアーム動作生成にはデータを送らない
        if (!success){
            RCLCPP_INFO(rclcpp::get_logger("GMP"), "recieved data is false");
            continue;
        }
        
        else{
                    // アームの動作生成
            RCLCPP_INFO(rclcpp::get_logger("GMP"), "Generating robot arm motion");
            // 果実位置まで移動
            target_pose.position.x = x;
            target_pose.position.y = y;
            target_pose.position.z = z;
            
            yaw = calc_yaw(x,y);
            q.setRPY(0.0, to_radians(100), yaw);
            target_pose.orientation.x = q.w();
            target_pose.orientation.y = q.z();
            target_pose.orientation.z = q.y();
            target_pose.orientation.w = q.x();

            move_group.clearPoseTarget();
            if (move_group.setPoseTarget(target_pose)){
                while (!move_group.move()){
                    move_group.clearPoseTarget();
                    move_group.setPoseTarget(target_pose);            
                    RCLCPP_INFO(rclcpp::get_logger("GMP"), "x: %f", target_pose.position.x);
                    RCLCPP_INFO(rclcpp::get_logger("GMP"), "y: %f", target_pose.position.y);
                    RCLCPP_INFO(rclcpp::get_logger("GMP"), "z: %f", target_pose.position.z);
                    // RCLCPP_INFO(rclcpp::get_logger("GMP"), "w: %f", target_pose.orientation.w);
                    continue;
                }


                // ハンドの機動→キャッチまで
                // 吸引軸伸ばす＆ポンプ駆動
                hand_data[0] = 0.0;
                hand_data[1] = 100.0;
                hand_data[2] = 1.0;
                hand_service(hand_client, hand_request, hand_data);

                // キャッチ
                create_eef_motion(hand_data, radius, 1.0);
                hand_service(hand_client, hand_request, hand_data);


                // エフェクタ軸回転
                joint_values = move_group.getCurrentJointValues();
                joint_values[4] = GRIPPER_STATE1;
                if (move_group.setJointValueTarget(joint_values)){
                    move_group.move();
                }

                joint_values = move_group.getCurrentJointValues();
                joint_values[4] = GRIPPER_STATE2;
                if (move_group.setJointValueTarget(joint_values)){
                    move_group.move();
                }

                // キャッチの完了→果実を置くところへ移動
                joint_values = move_group.getCurrentJointValues();
                joint_values[0] = to_radians(-98.03);
                joint_values[1] = to_radians(-72.31);
                joint_values[2] = to_radians(-1.40);
                joint_values[3] = to_radians(73.71);
                joint_values[4] = to_radians(-98.03);
                if (move_group.setJointValueTarget(joint_values)){
                    move_group.move();    
                }

                //　ハンドの把持を解除
                hand_data[0] = 0.0;
                hand_data[1] = 0.0;
                hand_data[2] = 0.0;
                hand_service(hand_client, hand_request, hand_data);

            }

            else{
                RCLCPP_ERROR(rclcpp::get_logger("generate motion point"), "Failed to create arm motion");
            }
        }


    }
    

    rclcpp::shutdown();
    return 0;
}