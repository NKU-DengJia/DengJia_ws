#include "rclcpp/rclcpp.hpp"
#include "self_interface/msg/ego_status.hpp" // 自定义消息类型
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class MarkerPublisher : public rclcpp::Node
{
public:
    MarkerPublisher() : Node("NodeEgoMarkerPublish")
    {
        // 订阅 ego_status_msg 消息
        subscription_ = this->create_subscription<self_interface::msg::EgoStatus>(
            "ego_status_msg", 10,
            std::bind(&MarkerPublisher::egoStatusCallback, this, std::placeholders::_1));

        // 创建 marker array 发布者
        marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "ego_marker_array", 10);
    }

private:
    void egoStatusCallback(const self_interface::msg::EgoStatus::SharedPtr msg)
    {
        // 创建车身 marker 消息
        auto body_marker_msg = std::make_shared<visualization_msgs::msg::Marker>();
        body_marker_msg->header = msg->header;
        body_marker_msg->header.frame_id = "/odom"; 
        body_marker_msg->type = visualization_msgs::msg::Marker::LINE_STRIP;
        body_marker_msg->id = 0;
        body_marker_msg->scale.x = 0.1; // 根据实际情况调整大小
        body_marker_msg->color.r = 0.0;
        body_marker_msg->color.g = 0.0;
        body_marker_msg->color.b = 1.0;
        body_marker_msg->color.a = 1.0;

        // 设置车身顶点
        geometry_msgs::msg::Point p1, p2, p3, p4;
        double length = 6.0; // 车身长度
        double width = 3.8; // 车身宽度
        double ego_theta = msg->ego_theta; // 车辆方向
        double half_length = length / 2;
        double half_width = width / 2;
        double cos_theta = cos(ego_theta);
        double sin_theta = sin(ego_theta);

        p1.x = msg->ego_x - half_length * cos_theta - half_width * sin_theta;
        p1.y = msg->ego_y - half_length * sin_theta + half_width * cos_theta;
        p1.z = 0.0;

        p2.x = msg->ego_x + half_length * cos_theta - half_width * sin_theta;
        p2.y = msg->ego_y + half_length * sin_theta + half_width * cos_theta;
        p2.z = 0.0;

        p3.x = msg->ego_x + half_length * cos_theta + half_width * sin_theta;
        p3.y = msg->ego_y + half_length * sin_theta - half_width * cos_theta;
        p3.z = 0.0;

        p4.x = msg->ego_x - half_length * cos_theta + half_width * sin_theta;
        p4.y = msg->ego_y - half_length * sin_theta - half_width * cos_theta;
        p4.z = 0.0;

        body_marker_msg->points.push_back(p1);
        body_marker_msg->points.push_back(p2);
        body_marker_msg->points.push_back(p3);
        body_marker_msg->points.push_back(p4);
        body_marker_msg->points.push_back(p1);

        // 创建方向箭头 marker 消息
        auto arrow_marker_msg = std::make_shared<visualization_msgs::msg::Marker>();
        arrow_marker_msg->header = msg->header;
        arrow_marker_msg->header.frame_id = "/odom"; 
        arrow_marker_msg->type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker_msg->id = 1;
        arrow_marker_msg->scale.x = 1.5; // 根据实际情况调整大小
        arrow_marker_msg->scale.y = 0.3;
        arrow_marker_msg->scale.z = 0.3;
        arrow_marker_msg->color.r = 0.0;
        arrow_marker_msg->color.g = 0.0;
        arrow_marker_msg->color.b = 1.0;
        arrow_marker_msg->color.a = 1.0;

        // 设置方向箭头位置和朝向
        arrow_marker_msg->pose.position.x = msg->ego_x + cos(ego_theta) * length / 2;
        arrow_marker_msg->pose.position.y = msg->ego_y + sin(ego_theta) * length / 2;
        arrow_marker_msg->pose.position.z = 0.0;
        arrow_marker_msg->pose.orientation.x = 0.0;
        arrow_marker_msg->pose.orientation.y = 0.0;
        arrow_marker_msg->pose.orientation.z = sin(ego_theta / 2.0);
        arrow_marker_msg->pose.orientation.w = cos(ego_theta / 2.0);

        // 创建 marker array 消息
        auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();

        // 添加车身 marker 消息
        marker_array_msg->markers.push_back(*body_marker_msg);

        // 添加方向箭头 marker 消息
        marker_array_msg->markers.push_back(*arrow_marker_msg);

        // 发布 marker array 消息
        marker_array_publisher_->publish(*marker_array_msg);
    }

    rclcpp::Subscription<self_interface::msg::EgoStatus>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
