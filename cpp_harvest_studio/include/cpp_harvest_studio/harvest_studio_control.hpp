#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define IS_HARVESTING true
#define NOT_HARVESTING false

double to_radian(double degree);

class HarvestStudioControl : public rclcpp::Node{
protected:
    sensor_msgs::msg::JointState jointstate;
    double angle_value;
    int studio_mode = 0;
    int pot_rotate_mode = 0;
    int is_rotating = 0;
    int rs_loop_complete_count = 0;
    int azure_loop_complete_count = 0;
    bool robotarm_hand_status = IS_HARVESTING;


private:
    // アーム角の配信
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_pub;
    // 把持回転機構の制御信号配信
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr studio_control_signal_pub;
    // 検出ループ完了信号受信
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr rs_loop_complete_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr azure_loop_complete_sub;
    // 果実データ処理ノードが保持している果実のデータ個数を受信
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr detect_status_sub;
    // 把持回転機構の現在の状態を受信
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr studio_mode_sub;
    // ロボットアーム等の動作状況を取得
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robotarm_hand_status_sub;


    void detect_status_callback(const std_msgs::msg::Int16::SharedPtr status);
    void studio_mode_callback(const std_msgs::msg::Int16MultiArray::SharedPtr mode_array);
    void rs_loop_complete_callback(const std_msgs::msg::Empty::SharedPtr empty);
    void azure_loop_complete_callback(const std_msgs::msg::Empty::SharedPtr empty);
    int harvest_completed(int rs_loop_comlete_count,
                          int azure_loop_complete_count,
                          int detect_status);
    void robotarm_hand_status_callback(const std_msgs::msg::Bool::SharedPtr status);

public:
    HarvestStudioControl(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    HarvestStudioControl(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
};