/*
アームやハンドの動作生成を行うノード
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include "harvest_studio_msg/srv/fruit_position_data.hpp"
#include "harvest_studio_msg/srv/end_effector_control.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>


#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>

#define TRY_MOTION_GENERATE_COUNT 5
#define IS_HARVESTING true
#define NOT_HARVESTING false

using namespace std::chrono_literals;



double x, y, z, radius;
bool success=false;

bool hand_status = false;

// エンドエフェクタの角度の生成を行う
void create_eef_motion(float hand_data[], float radius, float pump, std::string finger){
    // cup(hand_data[0]): 10~230
    // finger(hand_data[1]): 10~210
    // pump: 0~1

    float threshold_radius = 0.035;
    float max_cup = 150.0;
    float min_cup = 10.0;

    // 半径が0.035m以上の場合、10
    // それ以下の場合、10~150の範囲を線形的に制御
    if (radius > threshold_radius){
        hand_data[0] = min_cup;
    }
    else{
        hand_data[0] = (min_cup - max_cup)/threshold_radius*radius + max_cup;
    }
    
    // finger制御
    if (finger == "open"){
        hand_data[1] = 210.0;
    }
    else if (finger == "close"){
        hand_data[1] = 10.0;
    }
    
    
    hand_data[2] = pump;
}

double to_radians(const double deg_angle){
  return deg_angle * M_PI / 180.0;
}

double calc_yaw(double x, double y){
    double yaw;
    if (x==0.0){
        if (y > 0.0) {yaw = M_PI_2;}
        else if (y < 0.0) { yaw = -M_PI_2;}
        else { yaw = 0.0;}
    }
    else {yaw = atan(y/x);}

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
        // 1秒ごとに制御信号を送信してハンド側の状態を監視する
        auto result = hand_client->async_send_request(hand_request, std::bind(&hand_callback_response, std::placeholders::_1));
        rclcpp::sleep_for(1s);
        // RCLCPP_INFO(rclcpp::get_logger("EndEffector Control"), "result : false");
        // hand_status = result.get()->status;
    }

    // 次の呼び出し時にtrueのままだとループが回らないので，falseに
    hand_status = false;
}

// トマトのモデルをオブジェクトとしてrviz上に反映
moveit_msgs::msg::CollisionObject addTomato_object(std::string planning_frame,
                                                    double x, double y, double z, double radius){
    
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame;

    // The id of the object is used to identify it.
    collision_object.id = "tomato";

    // Define tomato(sphere) to add to the world
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(1);
    primitive.dimensions[0] = radius/2;

    // Define the pose for a tomato(sphere)
    geometry_msgs::msg::Pose tomato_pose;
    tomato_pose.position.x = x;
    tomato_pose.position.y = y;
    tomato_pose.position.z = z;
    tomato_pose.orientation.x = 0.0;
    tomato_pose.orientation.y = 0.0;
    tomato_pose.orientation.z = 0.0;
    tomato_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(tomato_pose);
    collision_object.operation = collision_object.ADD;

    // Now, let's add the collision object into the world
    return collision_object;
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
    // Move group interface for the robot
    moveit::planning_interface::MoveGroupInterface move_group(move_node, "xarm5");
    RCLCPP_INFO(rclcpp::get_logger("GMP"), "complete MoveGroupInterface");

    // Planning scene interface
    moveit::planning_interface::PlanningSceneInterface plannig_scene_interface;
    
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

    // Octomap更新用Publisher
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr fruit_target_command_pub =
        service_node->create_publisher<std_msgs::msg::Empty>("fruit_target_send_command", rclcpp::QoS(10));
    
    std_msgs::msg::Empty fruit_target_command;

    // 現在のロボットアームの動作状況の送信
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robotarm_hand_status_pub = 
        service_node->create_publisher<std_msgs::msg::Bool>("robotarm_hand_status", rclcpp::QoS(10));

    std_msgs::msg::Bool robotarm_hand_status;


    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setPlanningTime(0.5);
    auto joint_values = move_group.getCurrentJointValues();
    double GRIPPER_STATE1 = to_radians(-360);
    double GRIPPER_STATE2 = to_radians(360);
    geometry_msgs::msg::Pose target_pose;
    geometry_msgs::msg::PoseStamped target_pose_stamped;
    tf2::Quaternion q;

    std_msgs::msg::Float32MultiArray eef_data;

    // -----------------------initial pose--------------------------
    robotarm_hand_status.data = IS_HARVESTING;
    robotarm_hand_status_pub->publish(robotarm_hand_status);
    RCLCPP_INFO(rclcpp::get_logger("GMP"), "Pose Initialize");
    while (!move_group.setNamedTarget("hold-up")){
        continue;
    }
    while (!move_group.move()){
        continue;
    }
    robotarm_hand_status.data = NOT_HARVESTING;
    robotarm_hand_status_pub->publish(robotarm_hand_status);
    // -----------------------initial pose--------------------------

    double yaw;
    float hand_data[] = {0.0, 0.0, 0.0};
    int try_count = 0;
    moveit_msgs::msg::CollisionObject collision_object;
    std::vector<std::string> objects;
    objects.push_back("tomato");


    while (rclcpp::ok()){

        try_count = 0;

        // 初期姿勢
        robotarm_hand_status.data = IS_HARVESTING;
        robotarm_hand_status_pub->publish(robotarm_hand_status);
        joint_values = move_group.getCurrentJointValues();
        joint_values[0] = to_radians(-38.54);
        joint_values[1] = to_radians(-43.16);
        joint_values[2] = to_radians(-22.65);
        joint_values[3] = to_radians(65.80);
        joint_values[4] = to_radians(-38.53);
        if (move_group.setJointValueTarget(joint_values)){
            while(!move_group.move()){
                continue;
            }
        }
        robotarm_hand_status.data = NOT_HARVESTING;
        robotarm_hand_status_pub->publish(robotarm_hand_status);        


        // 果実位置情報の呼び出し
        
        while (!arm_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("GMP"), "Interrupted while waiting for the service. Exiting.");
              return 0;
            }
            RCLCPP_INFO(rclcpp::get_logger("GMP"), "fruit service not available, waiting again...");
        }

        auto result = arm_client->async_send_request(arm_request, std::bind(&arm_callback_response, std::placeholders::_1));

        // トマトオブジェクトの追加
        // RCLCPP_INFO(rclcpp::get_logger("GMP"), "adding tomato object");
        //plannig_scene_interface.applyCollisionObject(addTomato_object(move_group.getPlanningFrame(), x, y, z, radius));
        rclcpp::sleep_for(500ms);
        plannig_scene_interface.applyCollisionObject(addTomato_object(move_group.getPlanningFrame(), x, y, z, radius));

        // planning_scene_pub->publish(plannig_scene_interface);


        rclcpp::sleep_for(500ms);
        fruit_target_command_pub->publish(fruit_target_command);
        rclcpp::sleep_for(500ms);
        // Wait for the result.

        // RCLCPP_INFO(rclcpp::get_logger("GMP"), "C");

        // false(正しい値じゃない)場合、この先のアーム動作生成にはデータを送らない
        if (!success){
            RCLCPP_INFO(rclcpp::get_logger("GMP"), "recieved data is false");
            continue;
        }
        
        else{
            robotarm_hand_status.data = IS_HARVESTING;
            robotarm_hand_status_pub->publish(robotarm_hand_status);

            do  {
                // アームの動作生成
                RCLCPP_INFO(rclcpp::get_logger("GMP"), "Generating robot arm motion");
                // 果実位置まで移動
                target_pose.position.x = x;
                target_pose.position.y = y;
                target_pose.position.z = z;
                
                yaw = calc_yaw(x,y);
                switch (try_count)
                {
                case 0:
                    q.setRPY(0.0, to_radians(150), yaw);
                    RCLCPP_INFO(rclcpp::get_logger("GMP"), "degree: 150");
                    break;

                case 1:
                    q.setRPY(0.0, to_radians(140), yaw);
                    RCLCPP_INFO(rclcpp::get_logger("GMP"), "degree: 140");
                    break;

                case 2:
                    q.setRPY(0.0, to_radians(130), yaw);
                    RCLCPP_INFO(rclcpp::get_logger("GMP"), "degree: 130");
                    break;

                case 3:
                    q.setRPY(0.0, to_radians(120), yaw);
                    RCLCPP_INFO(rclcpp::get_logger("GMP"), "degree: 120");
                    break;

                case 4:
                    q.setRPY(0.0, to_radians(100), yaw);
                    RCLCPP_INFO(rclcpp::get_logger("GMP"), "degree: 100");
                    break;

                default:
                    q.setRPY(0.0, to_radians(100), yaw);
                    RCLCPP_INFO(rclcpp::get_logger("GMP"), "degree: 100");
                    break;
                }
                
                target_pose.orientation.x = q.w();
                target_pose.orientation.y = q.z();
                target_pose.orientation.z = q.y();
                target_pose.orientation.w = q.x();

                move_group.clearPoseTarget();
                if (move_group.setPoseTarget(target_pose)){
                    RCLCPP_INFO(rclcpp::get_logger("GMP"), "x: %f", target_pose.position.x);
                    RCLCPP_INFO(rclcpp::get_logger("GMP"), "y: %f", target_pose.position.y);
                    RCLCPP_INFO(rclcpp::get_logger("GMP"), "z: %f", target_pose.position.z);
                    // RCLCPP_INFO(rclcpp::get_logger("GMP"), "w: %f", target_pose.orientation.w);
                    try_count++;
                    continue;
                }
            } while (!move_group.move() && try_count <= TRY_MOTION_GENERATE_COUNT);
            
            // 5回以上動作生成失敗したら新しい果実のデータを取りに行く
            if(try_count > TRY_MOTION_GENERATE_COUNT){
                continue;
            }


            // ハンドの機動→キャッチまで
            // ハンドを開く
            // 吸引軸伸ばす
            hand_data[0] = 230.0;
            hand_data[1] = 210.0;
            hand_data[2] = 0.0;
            hand_service(hand_client, hand_request, hand_data);

            // ポンプ駆動
            hand_data[2] = 1.0;
            hand_service(hand_client, hand_request, hand_data);

            // 吸引軸の動作量生成
            create_eef_motion(hand_data, radius, 1.0, "open");
            hand_service(hand_client, hand_request, hand_data);
            rclcpp::sleep_for(1s);

            // キャッチ
            create_eef_motion(hand_data, radius, 1.0, "close");
            hand_service(hand_client, hand_request, hand_data);
            rclcpp::sleep_for(1s);

            // ポンプ停止
            create_eef_motion(hand_data, radius, 0.0, "close");
            hand_service(hand_client, hand_request, hand_data);

            // オブジェクトの把持
            move_group.attachObject("tomato");

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
            joint_values[0] = to_radians(-38.54);
            joint_values[1] = to_radians(-43.16);
            joint_values[2] = to_radians(-22.65);
            joint_values[3] = to_radians(65.80);
            joint_values[4] = to_radians(-38.53);
            if (move_group.setJointValueTarget(joint_values)){
                while(!move_group.move()){
                    continue;
                }
            }

            //　ハンドの把持を解除
            hand_data[0] = 10.0;
            hand_data[1] = 210.0;
            hand_data[2] = 0.0;
            hand_service(hand_client, hand_request, hand_data);
            rclcpp::sleep_for(1s);

            // 把持の終了
            hand_data[1] = 10.0;
            hand_service(hand_client, hand_request, hand_data);
            rclcpp::sleep_for(1s);

            // オブジェクトの把持を解除
            move_group.detachObject("tomato");


        }
    }
    

    rclcpp::shutdown();
    return 0;
}