#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

class DynaAnglePublisher : public rclcpp::Node{
protected:
    sensor_msgs::msg::JointState jointstate;
    float angle_value = -0.5;
    int angle_direction = 1;
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();

public:
    DynaAnglePublisher(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    DynaAnglePublisher(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
};
