#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "cpp_harvest_studio/point_cloud_updater.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// 最新のPointCloudを取得
void PointCloudUpdater::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg){
    pointcloud.header = pointcloud_msg->header;

    pointcloud.height = pointcloud_msg->height;
    pointcloud.width = pointcloud_msg->width;

    pointcloud.fields = pointcloud_msg->fields;

    pointcloud.is_bigendian = pointcloud_msg->is_bigendian;
    pointcloud.point_step = pointcloud_msg->point_step;
    pointcloud.row_step = pointcloud_msg->row_step;
    pointcloud.data = pointcloud_msg->data;

    pointcloud.is_dense = pointcloud_msg->is_dense;

    pointcloud_subscribed = true;
}

// 収穫用果実の取得信号を受信した際に、Octomapを更新する
void PointCloudUpdater::signal_callback(const std_msgs::msg::Empty::SharedPtr signal_msg){
    sensor_msgs::msg::PointCloud2 pointcloud_data;

    // 最新の点群が取得できてたら更新する
    if (pointcloud_subscribed){
        octomap_service();
        pointcloud_data = pointcloud;
        pointcloud_pub->publish(pointcloud_data);
    }

    pointcloud_subscribed = false;
}

// Octomapを初期化
void PointCloudUpdater::octomap_service(){
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    while (!octomap_client->wait_for_service(1s)){
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("PCU"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("PCU"), "pointcloud service not available, waiting again...");
    }

    auto result = octomap_client->async_send_request(request);
}

PointCloudUpdater::PointCloudUpdater(
    const rclcpp::NodeOptions& options
): PointCloudUpdater("", options){}

PointCloudUpdater::PointCloudUpdater(
    const std::string& name_space,
    const rclcpp::NodeOptions& options    
): Node("point_cloud_updater", name_space, options){
    pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("update_points2", rclcpp::QoS(10));

    pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/azure/points2",
        rclcpp::QoS(10),
        std::bind(&PointCloudUpdater::pointcloud_callback, this, _1));
    
    signal_sub = this->create_subscription<std_msgs::msg::Empty>(
        "fruit_target_send_command",
        rclcpp::QoS(10),
        std::bind(&PointCloudUpdater::signal_callback, this, _1));

    octomap_client = this->create_client<std_srvs::srv::Empty>("clear_octomap");

}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudUpdater>());
    rclcpp::shutdown();
    return 0;    
}