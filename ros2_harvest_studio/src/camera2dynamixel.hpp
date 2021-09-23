#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

std_msgs::msg::Float32 joint_pos;
std_msgs::msg::Float32 joint_vel;
std_msgs::msg::Float32 joint_acc;
std_msgs::msg::Float32 joint_eff;


class Camera2Dynamixel : public rclcpp::Node{
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState::ConstPtr>::SharedPtr sub_joints;
    rclcpp::TimerBase::SharedPtr timer_;
    void monitor_jointstate_callback(const sensor_msgs::msg::JointState::ConstPtr& jointstates);
public:
    Camera2Dynamixel(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    Camera2Dynamixel(
        const std::string& name_space="",
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );

};

