#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include "ros2_harvest_studio/msg/fruit_data_list.hpp"
/*自作msgの作成方法チェック*/
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class GenerateMotionPoint : public rclcpp::Node{
private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_pub;
    
    rclcpp::Subscription<ros2_harvest_studio/msg/FruitFataList>::SharedPtr fruit_data_sub;
    /* data */
public:
    GenerateMotionPoint
(/* args */);
    ~GenerateMotionPoint
();
};
