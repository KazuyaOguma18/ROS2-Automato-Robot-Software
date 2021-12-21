#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_xarm_state", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() {executor.spin(); }).detach();

    RCLCPP_INFO(rclcpp::get_logger("GMP"), "initialize MoveGroupInterface");

    MoveGroupInterface move_group(node,"xarm5");


    auto joint_values = move_group.getCurrentJointValues();
    auto current_pose = move_group.getCurrentPose();


    while (rclcpp::ok()){

        RCLCPP_INFO(rclcpp::get_logger("GMP"), "----------------------------------");
        joint_values = move_group.getCurrentJointValues();

        for (long unsigned int i = 0; i < joint_values.size(); i++){
            RCLCPP_INFO(rclcpp::get_logger("GXS"), "joint%d : %f",i, joint_values[i]);
        }

        current_pose = move_group.getCurrentPose();
        RCLCPP_INFO(rclcpp::get_logger("GMP"), "position x: %f", current_pose.pose.position.x);
        RCLCPP_INFO(rclcpp::get_logger("GMP"), "position y: %f", current_pose.pose.position.y);
        RCLCPP_INFO(rclcpp::get_logger("GMP"), "position z: %f", current_pose.pose.position.z);
        RCLCPP_INFO(rclcpp::get_logger("GMP"), "orientation x: %f", current_pose.pose.orientation.x);
        RCLCPP_INFO(rclcpp::get_logger("GMP"), "orientation y: %f", current_pose.pose.orientation.y);
        RCLCPP_INFO(rclcpp::get_logger("GMP"), "orientation z: %f", current_pose.pose.orientation.z);
        RCLCPP_INFO(rclcpp::get_logger("GMP"), "orientation w: %f", current_pose.pose.orientation.w);


        rclcpp::sleep_for(1s);

    }
    

    rclcpp::shutdown();
    return 0;
}