#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include "harvest_studio_msg/srv/fruit_position_data.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class GenerateMotionPoint : public rclcpp::Node{
protected:
    float fruit_data[4]; // (x,y,z,radius)

private:
    rclcpp::Service<harvest_studio_msg::srv::FruitPositionData>::SharedPtr fruit_service;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_eef;
    
    void fruit_position_data_callback(const std::shared_ptr<harvest_studio_msg::srv::FruitPositionData::Request> request,
                                        std::shared_ptr<harvest_studio_msg::srv::FruitPositionData::Response> response);
public:
    GenerateMotionPoint(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    GenerateMotionPoint(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
};
