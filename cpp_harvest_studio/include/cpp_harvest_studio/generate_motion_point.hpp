#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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

// エンドエフェクタの角度の生成を行う
void create_eef_motion(float hand_data[], float radius, float pump, std::string finger);

double to_radians(const double deg_angle);

double calc_yaw(double x, double y);

void arm_callback_response(rclcpp::Client<harvest_studio_msg::srv::FruitPositionData>::SharedFuture result);

void hand_callback_response(rclcpp::Client<harvest_studio_msg::srv::EndEffectorControl>::SharedFuture result);

void hand_service(rclcpp::Client<harvest_studio_msg::srv::EndEffectorControl>::SharedPtr hand_client,
                  std::shared_ptr<harvest_studio_msg::srv::EndEffectorControl::Request> hand_request, float eef_data[]);

// トマトのモデルをオブジェクトとしてrviz上に反映
moveit_msgs::msg::CollisionObject addTomato_object(std::string planning_frame,
                                                    double x, double y, double z, double radius);