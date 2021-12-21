#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <harvest_studio_msg/msg/fruit_data_list.hpp>

class AddRvizMarker : public rclcpp::Node{
private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_detect_list;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_harvest_list;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_harvest_target;

    rclcpp::Subscription<harvest_studio_msg::msg::FruitDataList>::SharedPtr sub_detect_list;
    rclcpp::Subscription<harvest_studio_msg::msg::FruitDataList>::SharedPtr sub_harvest_list;
    rclcpp::Subscription<harvest_studio_msg::msg::FruitDataList>::SharedPtr sub_harvest_target;

    void detect_list_callback(const harvest_studio_msg::msg::FruitDataList::SharedPtr detect_list);
    void harvest_list_callback(const harvest_studio_msg::msg::FruitDataList::SharedPtr harvest_list);
    void harvest_target_callback(const harvest_studio_msg::msg::FruitDataList::SharedPtr harvest_target);

public:
    AddRvizMarker(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    AddRvizMarker(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()        
    );
};