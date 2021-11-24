#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


class DynaAngleControl : public rclcpp::Node{
protected:
    sensor_msgs::msg::JointState jointstate;
    float angle_value = -0.5;

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_pub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr detect_status_sub;

    void detect_status_callback(const std_msgs::msg::Int16::SharedPtr status);

public:
    DynaAngleControl(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    DynaAngleControl(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
};