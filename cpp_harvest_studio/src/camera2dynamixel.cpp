/*
進捗状況
・コードの花形の作成は完了
・publishの型をdynamixel sdk仕様に変更する必要あり
・型によってpublish関連のコードを作成していく
*/
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <memory>
#include <algorithm>
#include <cmath>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include "cpp_harvest_studio/camera2dynamixel.hpp"
using namespace std::literals::chrono_literals;

constexpr double PROTOCOL_VERSION = 1.0;
constexpr int DXL_HOME_POSITION = 511;  // value range:0 ~ 1023
constexpr double DXL_MAX_POSITION = 1023.0;
constexpr double DXL_MAX_POSITION_DEGREES = 300.0;
constexpr double TO_RADIANS = (DXL_MAX_POSITION_DEGREES / DXL_MAX_POSITION) * M_PI / 180.0;
constexpr double TO_DXL_POS = 1.0 / TO_RADIANS;
constexpr double TO_SPEED_REV_PER_MIN = 0.111;
constexpr double TO_SPEED_RAD_PER_MIN = TO_SPEED_REV_PER_MIN * 2.0 * M_PI;
constexpr double TO_SPEED_RAD_PER_SEC = TO_SPEED_RAD_PER_MIN / 60.0;
constexpr double TO_LOAD_PERCENT = 0.1;
constexpr double TO_VOLTAGE = 0.1;

// Dynamixel AX-12A address table
// Ref: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
constexpr uint16_t ADDR_TORQUE_ENABLE = 24;
constexpr uint16_t ADDR_GOAL_POSITION = 30;
constexpr uint16_t ADDR_MOVING_SPEED = 32;
constexpr uint16_t ADDR_PRESENT_POSITION = 36;
constexpr uint16_t ADDR_PRESENT_SPEED = 38;
constexpr uint16_t ADDR_PRESENT_LOAD = 40;
constexpr uint16_t ADDR_PRESENT_VOLTAGE = 42;
constexpr uint16_t ADDR_PRESENT_TEMPERATURE = 43;

void Camera2Dynamixel::monitor_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr jointstates){
    cmd_joint_pos.clear();
    cmd_joint_vel.clear();
    cmd_joint_eff.clear();

    for (long unsigned int i=0; i<jointstates->position.size(); i++){
        if(jointstates->name[i] == "azure_camera_joint"){
            if(jointstates->position.size() > 0){
                cmd_joint_pos.push_back(jointstates->position[i]);
            }
            if(jointstates->velocity.size() > 0){
                cmd_joint_vel.push_back(jointstates->velocity[i]);
            }
            if(jointstates->effort.size() > 0){
                cmd_joint_eff.push_back(jointstates->effort[i]);
            }
        }
    }    
}

Camera2Dynamixel::Camera2Dynamixel(
    const rclcpp::NodeOptions& options
): Camera2Dynamixel("", options){}

Camera2Dynamixel::Camera2Dynamixel(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("camera2dynamixel", name_space, options){

    joint_name[0] = "azure_camera_joint";
    // https://ittechnicalmemos.blogspot.com/2019/09/usblinux.htmlを参考に修正
    std::string port_name="/dev/ttyUSB-Dyna";
    baudrate_=1000000;
    id_list_={1};

    dxl_port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
        dynamixel::PortHandler::getPortHandler(port_name.c_str()));
    dxl_packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
      dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));

    if (!open_port()){
        RCLCPP_INFO(this->get_logger(), "failed to open port");
        RCLCPP_INFO(this->get_logger(), last_error_log_);
        exit(1);
    }
    else{
      sub_joints = this->create_subscription<sensor_msgs::msg::JointState>(
        "/dyna_joint_command",
        rclcpp::QoS(10),
        std::bind(&Camera2Dynamixel::monitor_jointstate_callback, this, std::placeholders::_1)
      );
      pub_arm = this->create_publisher<sensor_msgs::msg::JointState>("/dyna_joint_state", rclcpp::QoS(10));
      timer_ = this->create_wall_timer(
        10ms,
        std::bind(&Camera2Dynamixel::timer_callback, this));
    }


}

// Dynamixel function
bool Camera2Dynamixel::open_port(void){
    if (!dxl_port_handler_->openPort()){
        last_error_log_ = std::string(__func__) + ": unable to open dynamixel port: " +
            dxl_port_handler_->getPortName();
        return false;
    }

    if (!dxl_port_handler_->setBaudRate(baudrate_)) {
        last_error_log_ = std::string(__func__) + ": unable to set baudrate" +
            std::to_string(dxl_port_handler_->getBaudRate());
        return false;
    }

  return true;      
}

void Camera2Dynamixel::close_port(void){
    dxl_port_handler_->closePort();
}

std::string Camera2Dynamixel::get_last_error_log(void){
    return last_error_log_;
}

bool Camera2Dynamixel::torque_enable(const bool enable){
    bool retval = true;
    for (auto dxl_id : id_list_){
        uint8_t dxl_error = 0;
        int dxl_result = dxl_packet_handler_->write1ByteTxRx(
            dxl_port_handler_.get(),
            dxl_id, ADDR_TORQUE_ENABLE, enable, &dxl_error);

        if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
            retval = false;
        }
    } 
    return retval;   
}

bool Camera2Dynamixel::write_goal_joint_positions(const std::vector<double> & goal_positions){
    if (goal_positions.size() != id_list_.size()) {
        last_error_log_ = std::string(__func__) + ": vectors size does not match: " +
          " goal_positions:" + std::to_string(goal_positions.size()) +
          ", id_list:" + std::to_string(id_list_.size());
        return false;
    }

    bool retval = true;

    for (size_t i = 0; i < goal_positions.size(); i++) {
        uint8_t dxl_error = 0;
        uint16_t goal_position = radian_to_dxl_pos(goal_positions[i]);
        auto dxl_id = id_list_[i];
        int dxl_result = dxl_packet_handler_->write2ByteTxRx(
            dxl_port_handler_.get(),
            dxl_id, ADDR_GOAL_POSITION, goal_position, &dxl_error);

        if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
            retval = false;
        }
    }

  return retval;    
}

bool Camera2Dynamixel::write_moving_speed_rpm(const uint8_t dxl_id, const double speed_rpm){
  const int DXL_MAX_MOVING_SPEED = 1023;
  const double SPEED_UNIT = 0.111;  // rpm
  if (std::find(id_list_.begin(), id_list_.end(), dxl_id) == id_list_.end()) {
    last_error_log_ = std::string(__func__) + ": dxl_id: " + std::to_string(dxl_id) +
      "not found.";
    return false;
  }

  int dxl_moving_speed = speed_rpm / SPEED_UNIT;
  if (dxl_moving_speed > DXL_MAX_MOVING_SPEED) {
    dxl_moving_speed = DXL_MAX_MOVING_SPEED;
  } else if (dxl_moving_speed == 0) {
    // If moving_speed is set to 0, it means the maximum rpm of the motor is used
    // without controlling the speed.
    dxl_moving_speed = 1;
  }

  bool retval = true;

  uint8_t dxl_error = 0;
  int dxl_result = dxl_packet_handler_->write2ByteTxRx(
    dxl_port_handler_.get(),
    dxl_id, ADDR_MOVING_SPEED, dxl_moving_speed, &dxl_error);

  retval = parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error);

  return retval;    
}

bool Camera2Dynamixel::write_moving_speed_rpm_all(const double speed_rpm){
  bool retval = true;

  for (auto dxl_id : id_list_) {
    if (!write_moving_speed_rpm(dxl_id, speed_rpm)) {
      retval = false;
    }
  }

  return retval;    
}

bool Camera2Dynamixel::read_present_joint_positions(std::vector<double> & joint_positions){
  std::vector<uint16_t> buffer;
  bool retval = read_word_data_from_each_joints(ADDR_PRESENT_POSITION, buffer);

  for (auto data : buffer) {
    joint_positions.push_back(dxl_pos_to_radian(data));
  }

  return retval;    
}

bool Camera2Dynamixel::read_present_joint_speeds(std::vector<double> & joint_speeds){
  std::vector<uint16_t> buffer;
  bool retval = read_word_data_from_each_joints(ADDR_PRESENT_SPEED, buffer);

  for (auto data : buffer) {
    joint_speeds.push_back(dxl_speed_to_rps(data));
  }

  return retval;    
}

bool Camera2Dynamixel::read_present_joint_loads(std::vector<double> & joint_loads){
    std::vector<uint16_t> buffer;
    bool retval = read_word_data_from_each_joints(ADDR_PRESENT_LOAD, buffer);

    for (auto data : buffer) {
      joint_loads.push_back(dxl_load_to_percent(data));
    }

    return retval;
}

bool Camera2Dynamixel::read_present_joint_voltages(std::vector<double> & joint_voltages){
  std::vector<uint8_t> buffer;
  bool retval = read_byte_data_from_each_joints(ADDR_PRESENT_VOLTAGE, buffer);

  for (auto data : buffer) {
    joint_voltages.push_back(dxl_voltage_to_actual_voltage(data));
  }

  return retval;
}

bool Camera2Dynamixel::read_present_joint_temperatures(std::vector<double> & joint_temperatures){
  std::vector<uint8_t> buffer;
  bool retval = read_byte_data_from_each_joints(ADDR_PRESENT_TEMPERATURE, buffer);

  for (auto data : buffer) {
    joint_temperatures.push_back(data);
  }

  return retval;
}

bool Camera2Dynamixel::read_byte_data_from_each_joints(
  const uint16_t address,
  std::vector<uint8_t> & buffer){
  bool retval = true;
  for (auto dxl_id : id_list_) {
    uint8_t dxl_error = 0;
    uint8_t data = 0;
    int dxl_result = dxl_packet_handler_->read1ByteTxRx(
      dxl_port_handler_.get(),
      dxl_id, address, &data, &dxl_error);

    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }

    buffer.push_back(data);
  }

  return retval;
}

bool Camera2Dynamixel::read_word_data_from_each_joints(
  const uint16_t address,
  std::vector<uint16_t> & buffer){
  bool retval = true;
  for (auto dxl_id : id_list_) {
    uint8_t dxl_error = 0;
    uint16_t data = 0;
    int dxl_result = dxl_packet_handler_->read2ByteTxRx(
      dxl_port_handler_.get(),
      dxl_id, address, &data, &dxl_error);

    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }
    buffer.push_back(data);
  }

  return retval;
}

bool Camera2Dynamixel::parse_dxl_error(
  const std::string func_name, const uint8_t dxl_id,
  const int dxl_comm_result, const uint8_t dxl_packet_error){
  bool retval = true;

  if (dxl_comm_result != COMM_SUCCESS) {
    last_error_log_ = func_name + ": dxl_id: " + std::to_string(dxl_id) + " :" +
      std::string(dxl_packet_handler_->getTxRxResult(dxl_comm_result));
    retval = false;
  }

  if (dxl_packet_error != 0) {
    last_error_log_ = func_name + ": dxl_id: " + std::to_string(dxl_id) + " :" +
      std::string(dxl_packet_handler_->getRxPacketError(dxl_packet_error));
    retval = false;
  }

  return retval;
}

double Camera2Dynamixel::dxl_pos_to_radian(const uint16_t position){
  return (position - DXL_HOME_POSITION) * TO_RADIANS;
}

uint16_t Camera2Dynamixel::radian_to_dxl_pos(const double position){
  return position * TO_DXL_POS + DXL_HOME_POSITION;
}

double Camera2Dynamixel::dxl_speed_to_rps(const uint16_t speed){
  if (speed < 1024) {  // CCW, positive rotation
    return speed * TO_SPEED_RAD_PER_SEC;
  } else {  // CW, negative rotation
    return -(speed - 1024) * TO_SPEED_RAD_PER_SEC;
  }
}

double Camera2Dynamixel::dxl_load_to_percent(const uint16_t load){
  if (load < 1024) {  // CCW, positive rotation
    return load * TO_LOAD_PERCENT;
  } else {  // CW, negative rotation
    return -(load - 1024) * TO_LOAD_PERCENT;
  }
}

double Camera2Dynamixel::dxl_voltage_to_actual_voltage(const uint8_t voltage){
  return voltage * TO_VOLTAGE;
}


void Camera2Dynamixel::timer_callback(){

    // RCLCPP_INFO(this->get_logger(), "timer_callback");

    // サーボ角度の受信
    joint_positions_dxl.clear();
    if(!read_present_joint_positions(joint_positions_dxl)){
        RCLCPP_INFO(this->get_logger(), "failed to get dynamixel position");
        return;
    }
    else if(joint_positions_dxl.size() > 0){
        for (size_t i = 0; i < joint_positions_dxl.size(); i++){
            joint_positions_rad.push_back(joint_positions_dxl[i]);
            
        }
        // RCLCPP_INFO(this->get_logger(), "%.4f", joint_positions_dxl[0]);
        // サーボ角度の配信
        jointstate.name.clear();
        jointstate.position.clear();
        jointstate.velocity.clear();
        jointstate.effort.clear();
        jointstate.header.stamp = rclcpp::Clock().now();
        jointstate.name.push_back(joint_name[0]);
        jointstate.position.push_back(joint_positions_dxl[0]);
        jointstate.velocity.push_back(0.0);
        jointstate.effort.push_back(0.0);

        pub_arm->publish(jointstate);  
          
    }
    else{
        // RCLCPP_INFO(this->get_logger(), "joint_pos not found");
    }

    
    // 現状のサーボ角度に基づく角度制御
    if (cmd_joint_pos.size() > 0){
        if (!write_goal_joint_positions(cmd_joint_pos)){
            RCLCPP_INFO(this->get_logger(), "failed to write dynamixel position");
        }    
    }

    // RCLCPP_INFO(this->get_logger(), "timer_callback3");
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Camera2Dynamixel>());
    rclcpp::shutdown();
    return 0;
}