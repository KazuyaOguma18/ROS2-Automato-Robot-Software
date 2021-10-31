/*
進捗状況
・一部関数除くほとんどのコーディング完了
・動作確認まだ
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "harvest_studio_msg/srv/fruit_position_data.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

// #include "cpp_harvest_studio/generate_motion_point.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

float *create_eef_motion(float radius){

}

double to_radians(const double deg_angle)
{
  return deg_angle * M_PI / 180.0;
}

/*
GenerateMotionPoint::GenerateMotionPoint(
    const rclcpp::NodeOptions& options
): GenerateMotionPoint("", options){}

GenerateMotionPoint::GenerateMotionPoint(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("generate_motion_point", name_space, options){
    fruit_service = this->create_service<harvest_studio_msg::srv::FruitPositionData>(
        "fruit_position_data", 
        &GenerateMotionPoint::fruit_position_data_callback);

    pub_eef = this->create_publisher<std_msgs::msg::Float32MultiArray>("/hand_goal", rclcpp::Qos(10));

}
*/

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("generate_motion_point");

    rclcpp::Client<harvest_studio_msg::srv::FruitPositionData>::SharedPtr client = 
        node->create_client<harvest_studio_msg::srv::FruitPositionData>("fruit_position_data");

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_eef =
        node->create_publisher<std_msgs::msg::Float32MultiArray>("/hand_goal", rclcpp::QoS(10));

    auto request = std::make_shared<harvest_studio_msg::srv::FruitPositionData::Request>();
    request.order = true;

    MoveGroupInterface move_group(node, "xarm5");
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    auto joint_values = move_group.getCurrentJointValues();
    double GRIPPER_STATE1 = to_radians(-180);
    double GRIPPER_STATE2 = to_radians(180);
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;

    // 初期姿勢
    if (move_group.setNamedTarget("home")){
        move_group.move();
    }
        

    float x, y, z, radius;
    bool success;
    float eef_angle;


    while (rclcpp::ok()){

        // 果実位置情報の呼び出し
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
              return 0;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) ==
          rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Recieved Fruit Position Data!");
          x = result.get()->x;
          y = result.get()->y;
          z = result.get()->z;
          radius = result.get()->radius;
          success = result.get()->success;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service fruit_position_data");
        }

        // アームの動作生成
        // 果実位置まで移動
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        q.setRPY(to_radians(0), to_radians(0), to_radians(0));
        target_pose.orientation = tf2::toMsg(q);
        if (move_group.setPoseTarget(target_pose)){
            move_group.move()        
            // ハンドの機動→キャッチまで
            // 吸引軸伸ばす＆ポンプ駆動
            pub_eef->publish([0,0,0]);
            
            // キャッチ
            *eef_angle = create_eef_motion(radius);
            pub_eef->publish(eef_angle);

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
            target_pose.position(0,0,0);
            q.setRPY(to_radians(0), to_radians(0), to_radians(0));
            target_pose.orientation = tf2::toMsg(q);
            if (move_group.setPoseTarget(target_pose)){
                move_group.move()    
            }

            //　ハンドの把持を解除
            pub_eef->publish([0,0,0]);
        }

        else{
            RCLCPP_ERROR(rclcpp::get_logger("generate motion point"), "Failed to create arm motion");
        }

    }
    

    rclcpp::shutdown();
    return 0;
}