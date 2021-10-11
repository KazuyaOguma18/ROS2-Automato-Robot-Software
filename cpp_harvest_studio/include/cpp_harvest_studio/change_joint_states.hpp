#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>



class ChangeJointStates : public rclcpp::Node
{
protected:
    sensor_msgs::msg::JointState rejoint_state;
    sensor_msgs::msg::JointState dyna_joint_state;
    std::string joint_name[8];

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_dyna_joint_state;
    void jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr jointstates);
    void dyna_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr dyna_jointsates);

public:
    ChangeJointStates(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    ChangeJointStates(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
};