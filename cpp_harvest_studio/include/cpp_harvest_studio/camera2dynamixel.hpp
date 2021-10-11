#include <dynamixel_sdk/dynamixel_sdk.h>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <string>
#include <vector>

std::vector<double> cmd_joint_pos;
std::vector<double> cmd_joint_vel;
std::vector<double> cmd_joint_eff;


class Camera2Dynamixel : public rclcpp::Node{
protected:
    std::vector<double> joint_positions_dxl;
    std::vector<double> joint_positions_rad;    
    sensor_msgs::msg::JointState jointstate;
    std::string joint_name[1];
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_arm;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints;
    rclcpp::TimerBase::SharedPtr timer_;
    void monitor_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr jointstates);
    void timer_callback();

    // define dynamixel private parameters
    std::shared_ptr<dynamixel::PortHandler> dxl_port_handler_;
    std::shared_ptr<dynamixel::PacketHandler> dxl_packet_handler_;
    int baudrate_;
    std::vector<uint8_t> id_list_;
    std::string last_error_log_;

    bool read_byte_data_from_each_joints(const uint16_t address, std::vector<uint8_t> & buffer);
    bool read_word_data_from_each_joints(const uint16_t address, std::vector<uint16_t> & buffer);
    bool parse_dxl_error(
      const std::string func_name, const uint8_t dxl_id,
      const int dxl_comm_result, const uint8_t dxl_packet_error);
    double dxl_pos_to_radian(const uint16_t position);
    uint16_t radian_to_dxl_pos(const double position);
    double dxl_speed_to_rps(const uint16_t speed);
    double dxl_load_to_percent(const uint16_t load);
    double dxl_voltage_to_actual_voltage(const uint8_t voltage);
public:
    Camera2Dynamixel(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    Camera2Dynamixel(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );

    // define dynamixel public parameters
    bool open_port(void);
    void close_port(void);
    std::string get_last_error_log(void);

    bool torque_enable(const bool enable);
    bool write_goal_joint_positions(const std::vector<double> & goal_positions);
    bool write_moving_speed_rpm(const uint8_t dxl_id, const double speed_rpm);
    bool write_moving_speed_rpm_all(const double speed_rpm);
    bool read_present_joint_positions(std::vector<double> & joint_positions);
    bool read_present_joint_speeds(std::vector<double> & joint_speeds);
    bool read_present_joint_loads(std::vector<double> & joint_loads);
    bool read_present_joint_voltages(std::vector<double> & joint_voltages);
    bool read_present_joint_temperatures(std::vector<double> & joint_temperatures);    

};

