#include "rclcpp/rclcpp.hpp"
#include "self_interface/msg/planner_traj.hpp"       // 自定义消息类型
#include "self_interface/msg/planner_traj_point.hpp" // 自定义消息类型
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>

class TrajMarkerPublisher : public rclcpp::Node
{
public:
    TrajMarkerPublisher() : Node("NodeTrajMarkerPublisher")
    {
        // 订阅 planner_traj 消息
        subscription_ = this->create_subscription<self_interface::msg::PlannerTraj>(
            "planner_traj", 10,
            std::bind(&TrajMarkerPublisher::plannerTrajCallback, this, std::placeholders::_1));

        // 创建 marker array 发布者
        marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "traj_marker_array", 10);
    }

private:
    void plannerTrajCallback(const self_interface::msg::PlannerTraj::SharedPtr msg)
    {
        // 创建 marker array 消息
        auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();

        // 遍历规划轨迹点，创建对应的 marker 消息
        int i = 0;
        for (const auto &traj_point : msg->traj_points)
        {

            // 创建箭头 marker 表示方向
            auto arrow_marker_msg = std::make_shared<visualization_msgs::msg::Marker>();
            arrow_marker_msg->header = msg->header;
            arrow_marker_msg->header.frame_id = "/odom";
            arrow_marker_msg->type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker_msg->id = i * 2 + 1;
            i = i + 1;
            // 设置标记尺寸
            arrow_marker_msg->scale.x = 1.5; // 根据实际情况调整大小
            arrow_marker_msg->scale.y = 0.2;
            arrow_marker_msg->scale.z = 0.2;
            arrow_marker_msg->color.r = 0.0;
            arrow_marker_msg->color.g = 1.0;
            arrow_marker_msg->color.b = 0.0;
            arrow_marker_msg->color.a = 1.0;
            // 设置方向箭头位置和朝向
            arrow_marker_msg->pose.position.x = traj_point.x;
            arrow_marker_msg->pose.position.y = traj_point.y;
            arrow_marker_msg->pose.position.z = 0.0;
            arrow_marker_msg->pose.orientation.x = 0.0;
            arrow_marker_msg->pose.orientation.y = 0.0;
            arrow_marker_msg->pose.orientation.z = sin(traj_point.theta / 2.0);
            arrow_marker_msg->pose.orientation.w = cos(traj_point.theta / 2.0);

            // 添加轨迹点 marker 消息
            // marker_array_msg->markers.push_back(*marker_msg);
            
            // 添加方向箭头 marker 消息
            marker_array_msg->markers.push_back(*arrow_marker_msg);
        }
        // 发布 marker array 消息
        marker_array_publisher_->publish(*marker_array_msg);
    }

    rclcpp::Subscription<self_interface::msg::PlannerTraj>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajMarkerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
