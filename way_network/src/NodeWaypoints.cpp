#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <fstream> // 添加文件操作的头文件
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class NodeWaypoints: public rclcpp::Node{
public:
    NodeWaypoints(): Node("NodeWaypoints"), pre_x(0), pre_y(0) { 
        // 创建订阅者对象（消息类型、话题名、回调函数、队列长度）
        subscription1_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&NodeWaypoints::odom_callback, this, std::placeholders::_1)); 
    };
private:
     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        if ((x-pre_x)*(x-pre_x) + (y-pre_y)*(y-pre_y) < 0.25) {
            return;
        }
        pre_x = x;
        pre_y = y;
        // 打开文件
        std::ofstream file("/home/dengjia/DengJia_ws/src/way_network/WayPoints/testrviz.txt", std::ios_base::app); // 打开文件，以追加模式写入数据
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file");
            return;
        }
        // 写入 x 和 y 坐标到文件
        file << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << std::endl;
        // 关闭文件
        file.close();
    }

    // 订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription1_;

    double pre_x = 0.0;
    double pre_y = 0.0;
}; 

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeWaypoints>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
