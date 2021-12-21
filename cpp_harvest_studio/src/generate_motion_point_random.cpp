/*
進捗状況
・一部関数除くほとんどのコーディング完了
・動作確認まだ
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <harvest_studio_msg/srv/fruit_position_data.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// エンドエフェクタの角度の生成を行う
void create_eef_motion(float angle[], float radius){
    for (int i = 0; i < 3; i++){
        angle[i] = 0.0*radius;
    }
}

double to_radians(const double deg_angle)
{
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


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("generate_motion_point", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() {executor.spin(); }).detach();

    RCLCPP_INFO(rclcpp::get_logger("GMP"), "initialize MoveGroupInterface");

    MoveGroupInterface move_group(node,"xarm5");

    RCLCPP_INFO(rclcpp::get_logger("GMP"), "complete MoveGroupInterface");
    
    rclcpp::Client<harvest_studio_msg::srv::FruitPositionData>::SharedPtr client = 
        node->create_client<harvest_studio_msg::srv::FruitPositionData>("fruit_position_data");

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_eef =
        node->create_publisher<std_msgs::msg::Float32MultiArray>("/hand_goal", rclcpp::QoS(10));

    auto request = std::make_shared<harvest_studio_msg::srv::FruitPositionData::Request>();
    request->order = true;


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
    while (!move_group.setNamedTarget("home")){
        continue;
    }
    while (!move_group.move()){
        continue;
    }
    

    float x, y, z, radius;
    float eef_angle[] = {0.0, 0.0, 0.0};


    while (rclcpp::ok()){
        // 初期姿勢
        RCLCPP_INFO(rclcpp::get_logger("GMP"), "Pose Initialize");  
        while (!move_group.setNamedTarget("hold-up")){
            continue;
        }
        while (!move_group.move()){
            continue;
        }


        // Wait for the result.
        /*
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS){
          RCLCPP_INFO(rclcpp::get_logger("GMP"), "Recieved Fruit Position Data!");
          x = result.get()->x;
          y = result.get()->y;
          z = result.get()->z;
          radius = result.get()->radius;
          success = result.get()->success;
        } 
        else {
          RCLCPP_ERROR(rclcpp::get_logger("GMP"), "Failed to call service fruit_position_data");
        }
        RCLCPP_INFO(rclcpp::get_logger("GMP"), "C");

        // false(正しい値じゃない)場合、この先のアーム動作生成にはデータを送らない
        if (!success){
            continue;
        }
        */
        
        // アームの動作生成
        x = 0.5;
        y = 0.5;
        z = 0.3;
        radius = 0.06;
        RCLCPP_INFO(rclcpp::get_logger("GMP"), "Generating robot arm motion");
        // 果実位置まで移動
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        q.setRPY(0.0, to_radians(0.0), to_radians(0.0));
        target_pose.orientation = tf2::toMsg(q);

        target_pose_stamped = move_group.getRandomPose();
        move_group.clearPoseTarget();
        if (move_group.setPoseTarget(target_pose_stamped)){
            while (!move_group.move()){
                move_group.clearPoseTarget();
                target_pose_stamped = move_group.getRandomPose();
                move_group.setPoseTarget(target_pose_stamped);
                continue;
            }     
            // ハンドの機動→キャッチまで
            // 吸引軸伸ばす＆ポンプ駆動
            for (int i=0; i<3; i++){
                eef_data.data.push_back(0.0);
            }
            pub_eef->publish(eef_data);
            eef_data.data.clear();

            // キャッチ
            create_eef_motion(eef_angle, radius);
            for (int i=0; i<3; i++){
                eef_data.data.push_back(eef_angle[0]);
            }
            
            pub_eef->publish(eef_data);
            eef_data.data.clear();

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
            joint_values[0] = -98.03;
            joint_values[1] = -72.31;
            joint_values[2] = -1.40;
            joint_values[3] = 73.71;
            joint_values[4] = -98.03;
            if (move_group.setJointValueTarget(joint_values)){
                move_group.move();    
            }

            //　ハンドの把持を解除
            for (int i=0; i<3; i++){
                eef_data.data.push_back(0.0);
            }
            pub_eef->publish(eef_data);
            eef_data.data.clear();
        }

        else{
            RCLCPP_ERROR(rclcpp::get_logger("generate motion point"), "Failed to create arm motion");
        }

    }
    

    rclcpp::shutdown();
    return 0;
}