/*
回転把持機構とカメラ角の制御を主に行うノード
現在の収穫対象果実の保持数が０個が5回続く
↓
カメラ角の制御　（下から上まで３段階）
↓
ポットの回転制御
*/
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include "cpp_harvest_studio/harvest_studio_control.hpp"

#define STUDIO_SETUP 0
#define STUDIO_END 1
#define STUDIO_RORATE 2

#define HARVEST_COMPLETED 0
#define HARVEST_RUNNNING 1

using std::placeholders::_1;

double to_radian(double degree){
    return degree*M_PI/180;
}

void HarvestStudioControl::detect_status_callback(const std_msgs::msg::Int16::SharedPtr status){
    static int previous_rotate_mode = 0;
    static int rotate_trigger = 0;
    std_msgs::msg::Int16 studio_signal;

    jointstate.name.clear();
    jointstate.position.clear();
    jointstate.velocity.clear();
    jointstate.effort.clear();

    // 初期モードならセットアップ信号を送信
    if (studio_mode == 0){
        studio_signal.data = STUDIO_SETUP;
        studio_control_signal_pub->publish(studio_signal);
    }

    // ポットの回転モードに入ったとき、カメラ角度の動作を開始    
    else if (studio_mode == 3){

        // ポットの回転モードが変化したらカメラ位置を初期化する
        if (previous_rotate_mode != pot_rotate_mode){
            angle_value = to_radian(-30.0);
        }

        if (harvest_completed(rs_loop_complete_count, azure_loop_complete_count, status->data) == HARVEST_COMPLETED){

            jointstate.header.stamp = rclcpp::Clock().now();
            jointstate.name.push_back("azure_camera_joint");
            jointstate.position.push_back(angle_value);
            jointstate_pub->publish(jointstate);

            // カメラ角を15度づつ回転
            if (angle_value < 0.0){
                angle_value += to_radian(15.0);
            }
            else{
                angle_value = to_radian(-30.0);
                rotate_trigger = 1;
            }
            
            // カメラ角の制御が完了したら、ポット回転信号を送信する
            if (rotate_trigger == 1){
                rotate_trigger = 0;
                studio_signal.data = STUDIO_RORATE;
                studio_control_signal_pub->publish(studio_signal);
            }
        }
    }

    else if (studio_mode == 4){
        studio_signal.data = STUDIO_END;
        studio_control_signal_pub->publish(studio_signal);
    }

    else{
        angle_value = to_radian(-30.0);
        jointstate.header.stamp = rclcpp::Clock().now();
        jointstate.name.push_back("azure_camera_joint");
        jointstate.position.push_back(angle_value);
        jointstate_pub->publish(jointstate);
    }

    previous_rotate_mode = pot_rotate_mode;    
}

// 把持回転機構の現在の動作状況を取得
void HarvestStudioControl::studio_mode_callback(const std_msgs::msg::Int16MultiArray::SharedPtr mode_array){
    static int previous_rotate_mode = 0;
    static int start_rotating = 0;
    // 動作モード
    studio_mode = mode_array->data[0];
    // ポットの回転モード
    pot_rotate_mode = mode_array->data[1];

    // ポットが回転中かどうか->しかしラグのためすぐに回転中かどうかわからない
    // ポットの回転モードの変化->回転開始
    // 回転開始後、再度ポットの回転状況の信号が0となったら回転終了
    if (pot_rotate_mode != previous_rotate_mode){
        start_rotating = 1;
    }
    if (start_rotating == 1){
        if (mode_array->data[2] != 0){
            is_rotating = 1;
        }
        if (is_rotating == 1 && mode_array->data[2] == 0){
            is_rotating = 0;
            start_rotating = 0;
        }
    }
}

// realsenseが１ループ分物体検出を完了した信号を受信
void HarvestStudioControl::rs_loop_complete_callback(const std_msgs::msg::Empty::SharedPtr empty){
    rs_loop_complete_count++;
}

// kinectが１ループ分物体検出を完了した信号を受信
void HarvestStudioControl::azure_loop_complete_callback(const std_msgs::msg::Empty::SharedPtr empty){
    azure_loop_complete_count++;
}

// 収穫動作が完了したかどうか判定
// 両方の物体検出を２ループ以上行った上で果実の個数が０個だったら収穫完了と判定
int HarvestStudioControl::harvest_completed(int rs_loop_comlete_count,
                                            int azure_loop_complete_count,
                                            int detect_status){
    if (rs_loop_comlete_count > 1 && azure_loop_complete_count > 1){
        if (detect_status == 0){
            return HARVEST_COMPLETED;
        }
    }
    return HARVEST_RUNNNING;
}

HarvestStudioControl::HarvestStudioControl(
    const rclcpp::NodeOptions& options
): HarvestStudioControl("", options){}

HarvestStudioControl::HarvestStudioControl(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("harvest_studio_control", name_space, options){
    jointstate_pub = this->create_publisher<sensor_msgs::msg::JointState>("dyna_joint_command", rclcpp::QoS(10));
    
    studio_control_signal_pub = this->create_publisher<std_msgs::msg::Int16>("studio_control_signal", rclcpp::QoS(10));

    detect_status_sub = this->create_subscription<std_msgs::msg::Int16>(
        "fruit_detect_status",
        rclcpp::QoS(10),
        std::bind(&HarvestStudioControl::detect_status_callback, this, _1));

    studio_mode_sub = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "studio_mode",
        rclcpp::QoS(10),
        std::bind(&HarvestStudioControl::studio_mode_callback, this, _1));

    rs_loop_complete_sub = this->create_subscription<std_msgs::msg::Empty>(
        "rs_loop_comlete",
        rclcpp::QoS(10),
        std::bind(&HarvestStudioControl::rs_loop_complete_callback, this , _1));

    azure_loop_complete_sub = this->create_subscription<std_msgs::msg::Empty>(
        "azure_loop_comlete",
        rclcpp::QoS(10),
        std::bind(&HarvestStudioControl::azure_loop_complete_callback, this, _1));

    angle_value = to_radian(-30.0);    
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HarvestStudioControl>());
    rclcpp::shutdown();
    return 0;
}