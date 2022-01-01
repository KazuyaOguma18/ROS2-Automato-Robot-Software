#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <memory>

#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "cpp_harvest_studio/change_joint_states.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

void ChangeJointStates::timer_callback(){
    rejoint_state.header.stamp = rclcpp::Clock().now();
    for(long unsigned int i=0; i < xarm_joint_state.name.size(); i++){
        if(rejoint_state.name[i] == xarm_joint_state.name[i]){
            if(rejoint_state.position.size() > 0){
                rejoint_state.position[i] = xarm_joint_state.position[i];
            }
            if(rejoint_state.velocity.size() > 0){
                rejoint_state.velocity[i] = xarm_joint_state.velocity[i];
            }
            if(rejoint_state.effort.size() > 0){
                rejoint_state.effort[i] = xarm_joint_state.effort[i];
            }
        }

    }
        
    for (long unsigned int i=0; i < rejoint_state.position.size(); i++){
        if(rejoint_state.name[i] == "azure_camera_joint"){
            if(rejoint_state.position.size() > 0){
                rejoint_state.position[i] = dyna_joint_state.position[0];
            }
            if(rejoint_state.velocity.size() > 0){
                rejoint_state.velocity[i] = dyna_joint_state.velocity[0];
            }
            if(rejoint_state.effort.size() > 0){
                rejoint_state.effort[i] = dyna_joint_state.effort[0];
            }
        }
        else if(rejoint_state.name[i] == "right_arm_joint"){
            if(rejoint_state.position.size() > 0){
                rejoint_state.position[i] = arm_joint_state.position[0];
            }
            if(rejoint_state.velocity.size() > 0){
                rejoint_state.velocity[i] = arm_joint_state.velocity[0];
            }
            if(rejoint_state.effort.size() > 0){
                rejoint_state.effort[i] = arm_joint_state.effort[0];
            }
        }
        else if(rejoint_state.name[i] == "left_arm_joint"){
            if(rejoint_state.position.size() > 0){
                rejoint_state.position[i] = arm_joint_state.position[1];
            }
            if(rejoint_state.velocity.size() > 0){
                rejoint_state.velocity[i] = arm_joint_state.velocity[1];
            }
            if(rejoint_state.effort.size() > 0){
                rejoint_state.effort[i] = arm_joint_state.effort[1];
            }
        }
    }

    pub_joint_state->publish(rejoint_state);
}

void ChangeJointStates::xarm_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr jointstates){
    for(long unsigned int i=0; i < jointstates->name.size(); i++){
        if(xarm_joint_state.name[i] == jointstates->name[i]){
            if(jointstates->position.size() > 0){
                xarm_joint_state.position[i] = jointstates->position[i];
            }
            if(jointstates->velocity.size() > 0){
                xarm_joint_state.velocity[i] = jointstates->velocity[i];
            }
            if(jointstates->effort.size() > 0){
                xarm_joint_state.effort[i] = jointstates->effort[i];
            }
        }

    }

}

void ChangeJointStates::dyna_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr jointstates){
    if(jointstates->name[0] == "azure_camera_joint"){
        if(jointstates->position.size() > 0){
            dyna_joint_state.position[0] = jointstates->position[0];
        }
        if(jointstates->velocity.size() > 0){
            dyna_joint_state.velocity[0] = jointstates->velocity[0];
        }
        if(jointstates->effort.size() > 0){
            dyna_joint_state.effort[0] = jointstates->effort[0];
        }
    }
}

void ChangeJointStates::arm_jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr jointstates){
    for (long unsigned int i=0; i<jointstates->position.size(); i++){
        if(jointstates->name[i] == "right_arm_joint"){
            if(jointstates->position.size() > 0){
                arm_joint_state.position[0] = jointstates->position[i];
            }
            if(jointstates->velocity.size() > 0){
                arm_joint_state.velocity[0] = jointstates->velocity[i];
            }
            if(jointstates->effort.size() > 0){
                arm_joint_state.effort[0] = jointstates->effort[i];
            }
        }
        else if(jointstates->name[i] == "left_arm_joint"){
            if(jointstates->position.size() > 0){
                arm_joint_state.position[1] = jointstates->position[i];
            }
            if(jointstates->velocity.size() > 0){
                arm_joint_state.velocity[1] = jointstates->velocity[i];
            }
            if(jointstates->effort.size() > 0){
                arm_joint_state.effort[1] = jointstates->effort[i];
            }
        }
    }
}

ChangeJointStates::ChangeJointStates(
    const rclcpp::NodeOptions& options
): ChangeJointStates("", options){}

ChangeJointStates::ChangeJointStates(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("change_joint_states", name_space, options){
    sub_xarm_joint_state = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states_remap",
        rclcpp::QoS(10),
        std::bind(&ChangeJointStates::xarm_jointstate_callback, this, _1));

    sub_dyna_joint_state = this->create_subscription<sensor_msgs::msg::JointState>(
        "/dyna_joint_state",
        rclcpp::QoS(10),
        std::bind(&ChangeJointStates::dyna_jointstate_callback, this, _1));

    sub_arm_joint_state = this->create_subscription<sensor_msgs::msg::JointState>(
        "/arm_joint_state",
        rclcpp::QoS(10),
        std::bind(&ChangeJointStates::arm_jointstate_callback, this, _1));        
    
    timer = this->create_wall_timer(
        10ms,
        [this]() -> void{ ChangeJointStates::timer_callback(); });

    pub_joint_state = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));

    rejoint_state.name.clear();
    rejoint_state.position.clear();
    rejoint_state.velocity.clear();
    rejoint_state.effort.clear();
    rejoint_state.header.stamp = rclcpp::Clock().now();
    joint_name[0] = "joint1";
    joint_name[1] = "joint2";
    joint_name[2] = "joint3";
    joint_name[3] = "joint4";
    joint_name[4] = "joint5";
    joint_name[5] = "azure_camera_joint";
    joint_name[6] = "right_arm_joint";
    joint_name[7] = "left_arm_joint";

    for (int i = 0; i < 8; i++){
        rejoint_state.name.push_back(joint_name[i]);
        rejoint_state.position.push_back(0.0);
        rejoint_state.velocity.push_back(0.0);
        rejoint_state.effort.push_back(0.0);
    }

    // dynamixelの角度情報の初期化
    dyna_joint_state.name.clear();
    dyna_joint_state.position.clear();
    dyna_joint_state.velocity.clear();
    dyna_joint_state.effort.clear();
    dyna_joint_state.name.push_back("azure_camera_joint");
    dyna_joint_state.position.push_back(0.0);
    dyna_joint_state.velocity.push_back(0.0);
    dyna_joint_state.effort.push_back(0.0);  

    // 把持機構の角度情報の初期化
    arm_joint_state.name.clear();
    arm_joint_state.position.clear();
    arm_joint_state.velocity.clear();
    arm_joint_state.effort.clear();
    arm_joint_state.name.push_back("right_arm_joint");
    arm_joint_state.name.push_back("left_arm_joint");
    arm_joint_state.position.push_back(0.0);
    arm_joint_state.velocity.push_back(0.0);
    arm_joint_state.effort.push_back(0.0);      
    arm_joint_state.position.push_back(0.0);
    arm_joint_state.velocity.push_back(0.0);
    arm_joint_state.effort.push_back(0.0);

    // xarmの角度情報の初期化
    xarm_joint_state.name.clear();
    xarm_joint_state.position.clear();
    xarm_joint_state.velocity.clear();
    xarm_joint_state.effort.clear();
    xarm_joint_state.name.push_back("joint1");
    xarm_joint_state.name.push_back("joint2");
    xarm_joint_state.name.push_back("joint3");
    xarm_joint_state.name.push_back("joint4");
    xarm_joint_state.name.push_back("joint5");
    for (size_t i = 0; i < 5; i++){
        xarm_joint_state.position.push_back(0.0);
        xarm_joint_state.velocity.push_back(0.0);
        xarm_joint_state.effort.push_back(0.0);
    }
    
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChangeJointStates>());
    rclcpp::shutdown();
    return 0;
}

