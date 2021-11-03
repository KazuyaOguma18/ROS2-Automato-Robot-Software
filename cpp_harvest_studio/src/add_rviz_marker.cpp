#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <memory>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <harvest_studio_msg/msg/fruit_data_list.hpp>

#include "cpp_harvest_studio/add_rviz_marker.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

void AddRvizMarker::detect_list_callback(const harvest_studio_msg::msg::FruitDataList::SharedPtr detect_list){
    long unsigned int data_len = detect_list->x.size(); 

    visualization_msgs::msg::MarkerArray marker;
    marker.markers.resize(data_len);

    for (long unsigned int i = 0; i < data_len; i++){
        marker.markers[i].header.frame_id = "stand_base";
        marker.markers[i].header.stamp = rclcpp::Clock().now();

        marker.markers[i].ns = "detect_list";
        marker.markers[i].id = i;

        marker.markers[i].type = visualization_msgs::msg::Marker::SPHERE;
        marker.markers[i].action = visualization_msgs::msg::Marker::ADD;
        marker.markers[i].lifetime = rclcpp::Duration(1s);

        marker.markers[i].scale.x = detect_list->radius[i]*0.001;
        marker.markers[i].scale.y = detect_list->radius[i]*0.001;
        marker.markers[i].scale.z = detect_list->radius[i]*0.001;

        marker.markers[i].pose.position.x = detect_list->x[i]*0.001;
        marker.markers[i].pose.position.y = detect_list->y[i]*0.001;
        marker.markers[i].pose.position.z = detect_list->z[i]*0.001;
        marker.markers[i].pose.orientation.x = 0;
        marker.markers[i].pose.orientation.y = 0;
        marker.markers[i].pose.orientation.z = 0;
        marker.markers[i].pose.orientation.w = 1;
        marker.markers[i].color.r = 1.0f;
        marker.markers[i].color.g = 0.0f;
        marker.markers[i].color.b = 0.0f;
        marker.markers[i].color.a = 0.5f;

    pub_detect_list->publish(marker);
    }
    

}
void AddRvizMarker::harvest_list_callback(const harvest_studio_msg::msg::FruitDataList::SharedPtr harvest_list){
    long unsigned int data_len = harvest_list->x.size(); 

    visualization_msgs::msg::MarkerArray marker;
    marker.markers.resize(data_len);

    for (long unsigned int i = 0; i < data_len; i++){
        marker.markers[i].header.frame_id = "stand_base";
        marker.markers[i].header.stamp = rclcpp::Clock().now();

        marker.markers[i].ns = "harvest_list";
        marker.markers[i].id = i;

        marker.markers[i].type = visualization_msgs::msg::Marker::SPHERE;
        marker.markers[i].action = visualization_msgs::msg::Marker::ADD;
        marker.markers[i].lifetime = rclcpp::Duration(1s);

        marker.markers[i].scale.x = harvest_list->radius[i]*0.001;
        marker.markers[i].scale.y = harvest_list->radius[i]*0.001;
        marker.markers[i].scale.z = harvest_list->radius[i]*0.001;

        marker.markers[i].pose.position.x = harvest_list->x[i]*0.001;
        marker.markers[i].pose.position.y = harvest_list->y[i]*0.001;
        marker.markers[i].pose.position.z = harvest_list->z[i]*0.001;
        marker.markers[i].pose.orientation.x = 0;
        marker.markers[i].pose.orientation.y = 0;
        marker.markers[i].pose.orientation.z = 0;
        marker.markers[i].pose.orientation.w = 1;
        marker.markers[i].color.r = 0.0f;
        marker.markers[i].color.g = 0.0f;
        marker.markers[i].color.b = 1.0f;
        marker.markers[i].color.a = 0.8f;

    pub_harvest_list->publish(marker);
    }
}


void AddRvizMarker::harvest_target_callback(const harvest_studio_msg::msg::FruitDataList::SharedPtr harvest_target){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "stand_base";
    marker.header.stamp = rclcpp::Clock().now();

    marker.ns = "harvest_target";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration(1s);

    marker.scale.x = harvest_target->radius[0]*0.001;
    marker.scale.y = harvest_target->radius[0]*0.001;
    marker.scale.z = harvest_target->radius[0]*0.001;

    marker.pose.position.x = harvest_target->x[0]*0.001;
    marker.pose.position.y = harvest_target->y[0]*0.001;
    marker.pose.position.z = harvest_target->z[0]*0.001;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    pub_harvest_target->publish(marker);
}

AddRvizMarker::AddRvizMarker(
    const rclcpp::NodeOptions& options
): AddRvizMarker("", options){}

AddRvizMarker::AddRvizMarker(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("harvest_studio_add_rviz_marker", name_space, options){
    sub_detect_list = this->create_subscription<harvest_studio_msg::msg::FruitDataList>(
        "fruit_detect_list",
        rclcpp::QoS(10),
        std::bind(&AddRvizMarker::detect_list_callback, this, _1));

    sub_harvest_list = this->create_subscription<harvest_studio_msg::msg::FruitDataList>(
        "harvest_list",
        rclcpp::QoS(10),
        std::bind(&AddRvizMarker::harvest_list_callback, this, _1));

    sub_harvest_target = this->create_subscription<harvest_studio_msg::msg::FruitDataList>(
        "harvest_target",
        rclcpp::QoS(10),
        std::bind(&AddRvizMarker::harvest_list_callback, this, _1));

    pub_detect_list = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_detect_list", rclcpp::QoS(10));
    pub_harvest_list = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_harvest_list", rclcpp::QoS(10));
    pub_harvest_target = this->create_publisher<visualization_msgs::msg::Marker>("/marker_harvest_target", rclcpp::QoS(10));

}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddRvizMarker>());
    rclcpp::shutdown();
    return 0;
}