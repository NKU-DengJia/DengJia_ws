#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <fstream>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <filesystem>

class NodeReadtxtAndPublish : public rclcpp::Node
{
public:
    NodeReadtxtAndPublish() : Node("NodeReadtxtAndPublish")
    {
        read_modoule_planner_ini();
        publisher_waypoints_ = this->create_publisher<visualization_msgs::msg::Marker>("way_network", 1);
        publisher_obstacles_ = this->create_publisher<visualization_msgs::msg::Marker>("static_obstacles", 1);
        // 读取路网，并在路网附近随机生成障碍物
        readWaypointsAndGenerateObstacles();
        // 定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&NodeReadtxtAndPublish::publishMarkers, this));
    }

private:
    int num_obstacles = 0;  // 障碍物数量
    std::string file_path_; // 路网文件路径
    void read_modoule_planner_ini()
    {
        try
        {
            boost::property_tree::ptree pt;
            boost::property_tree::ini_parser::read_ini("/home/dengjia/DengJia_ws/src/module_planner/src/module_planner.ini", pt);

            num_obstacles = pt.get<int>("module_planner.num_static_obstacles");
            file_path_ = pt.get<std::string>("module_planner.way_file_path_");
        }
        catch (const boost::property_tree::ini_parser_error &e)
        {
            // 读取失败，输出错误信息
            std::cerr << "Error reading module_planner.ini: " << e.what() << std::endl;
        }
    }

    void readWaypointsAndGenerateObstacles()
    {
        // 打开.txt路网文件
        std::ifstream file(file_path_);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path_.c_str());
            return;
        }
        // 读取.txt路网文件
        float x, y;
        while (file >> x >> y)
        {
            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = 0.0;
            waypoints_.push_back(point);
        }

        // Random number generator
        std::random_device rd;
        std::mt19937 gen(rd());

        for (int i = 0; i < num_obstacles; ++i)
        {
            // 随机选择一个路点
            int waypoint_index = std::uniform_int_distribution<>(30, waypoints_.size() - 1)(gen);
            geometry_msgs::msg::Point waypoint = waypoints_[waypoint_index];

            // 随机生成障碍物长宽
            std::uniform_real_distribution<> dis_width(1.0, 4.0);
            std::uniform_real_distribution<> dis_length(1.0, 6.0);
            double width = dis_width(gen);
            double length = dis_length(gen);

            // 随机生成障碍物在路点附近的偏移量
            std::uniform_real_distribution<> dis_offset(-5.0, 5.0);
            double offset_x = dis_offset(gen);
            double offset_y = dis_offset(gen);

            // 随机生成朝向
            std::uniform_real_distribution<> dis_angle(0.0, M_PI);
            double angle = dis_angle(gen);

            // 在路点附近，随机生成障碍物位置
            geometry_msgs::msg::Point obstacle_point;
            obstacle_point.x = waypoint.x + offset_x;
            obstacle_point.y = waypoint.y + offset_y;
            obstacle_point.z = 0.5;

            // 检查生成的障碍物位置与已有障碍物的距离
            bool valid_position = true;
            for (const auto &obstacle : obstacles_)
            {
                double distance = std::sqrt(std::pow(obstacle.pose.position.x - obstacle_point.x, 2) +
                                            std::pow(obstacle.pose.position.y - obstacle_point.y, 2));
                if (distance < 15.0)
                {
                    valid_position = false;
                    break;
                }
            }
            // 如果位置无效，则重新生成
            if (!valid_position)
            {
                --i; // 重新生成当前障碍物
                continue;
            }

            // 设置障碍物marker
            visualization_msgs::msg::Marker obstacle_marker;
            obstacle_marker.header.frame_id = "/odom";
            obstacle_marker.header.stamp = this->now();
            obstacle_marker.ns = "static_obstacles";
            obstacle_marker.id = i;
            obstacle_marker.type = visualization_msgs::msg::Marker::CUBE;
            obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
            obstacle_marker.pose.position = obstacle_point;
            obstacle_marker.scale.x = length;
            obstacle_marker.scale.y = width;
            obstacle_marker.scale.z = 1.0;

            tf2::Quaternion quat;
            quat.setRPY(0, 0, angle);
            obstacle_marker.pose.orientation = tf2::toMsg(quat);

            obstacle_marker.color.r = 0.2;
            obstacle_marker.color.g = 0.2;
            obstacle_marker.color.b = 0.2;
            obstacle_marker.color.a = 1.0;

            obstacles_.push_back(obstacle_marker);
        }
    }

    void publishMarkers()
    {
        // 广播障碍物
        for (const auto &obstacle : obstacles_)
        {
            publisher_obstacles_->publish(obstacle);
        }
        // 广播路点
        visualization_msgs::msg::Marker waypoint_marker;
        waypoint_marker.header.frame_id = "/odom";
        waypoint_marker.header.stamp = this->now();
        waypoint_marker.ns = "way_points";
        waypoint_marker.id = 0;
        waypoint_marker.type = visualization_msgs::msg::Marker::POINTS;
        waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
        waypoint_marker.pose.orientation.w = 1.0;
        waypoint_marker.scale.x = 0.8;
        waypoint_marker.scale.y = 0.8;
        waypoint_marker.color.r = 0.5;
        waypoint_marker.color.g = 0.5;
        waypoint_marker.color.b = 0.5;
        waypoint_marker.color.a = 1.0;
        waypoint_marker.points = waypoints_;
        publisher_waypoints_->publish(waypoint_marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_waypoints_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_obstacles_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::Point> waypoints_;
    std::vector<visualization_msgs::msg::Marker> obstacles_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeReadtxtAndPublish>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
