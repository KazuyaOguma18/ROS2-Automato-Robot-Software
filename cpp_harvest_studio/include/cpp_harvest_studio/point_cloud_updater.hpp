#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <std_srvs/srv/empty.hpp>

class PointCloudUpdater : public rclcpp::Node{
protected:
    sensor_msgs::msg::PointCloud2 pointcloud;
    bool pointcloud_subscribed = false;
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr signal_sub;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_client;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg);
    void signal_callback(const std_msgs::msg::Empty::SharedPtr signal_msg);
    void octomap_service();

public:
    PointCloudUpdater(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    PointCloudUpdater(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
};
