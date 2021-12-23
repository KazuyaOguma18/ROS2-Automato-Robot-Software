#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


class HarvestStudioControl : public rclcpp::Node{
protected:
    sensor_msgs::msg::JointState jointstate;
    float angle_value = -0.5;
    int studio_mode = 0;
    int pot_rotate_mode = 0;

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr grasp_unit_pub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr detect_status_sub;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr studio_mode_sub;


    void detect_status_callback(const std_msgs::msg::Int16::SharedPtr status);
    void studio_mode_callback(const std_msgs::msg::Int16MultiArray::SharedPtr mode_array);

public:
    HarvestStudioControl(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    HarvestStudioControl(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
};