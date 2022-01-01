#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>



class ChangeJointStates : public rclcpp::Node
{
protected:
    sensor_msgs::msg::JointState rejoint_state;
    sensor_msgs::msg::JointState dyna_joint_state;
    sensor_msgs::msg::JointState arm_joint_state;
    sensor_msgs::msg::JointState xarm_joint_state;
    std::string joint_name[8];

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_xarm_joint_state;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_dyna_joint_state;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_arm_joint_state;
    rclcpp::TimerBase::SharedPtr timer;

    void xarm_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr jointstates);
    void dyna_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr dyna_jointsates);
    void arm_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr arm_jointsates);
    void timer_callback();

public:
    ChangeJointStates(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    ChangeJointStates(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
};