/*
订阅：
    self_interface::msg::ego_status     ego_status_msg     (100Hz）
发布：
    nav_msgs/msg/occupancy_grid         local_grid_map_msg     (10Hz）频率大于等于规划就行
*/
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "self_interface/msg/ego_status.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
class LocalGridMapPublisher : public rclcpp::Node
{
public:
    LocalGridMapPublisher() : Node("local_grid_map_publisher")
    {
        read_module_planner_ini();
        ego_status_subscription_ = this->create_subscription<self_interface::msg::EgoStatus>(
            "ego_status_msg", 10, std::bind(&LocalGridMapPublisher::egoStatusCallback, this, std::placeholders::_1));

        local_grid_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "local_grid_map_msg", 10);
    }

private:
    double grid_map_height = 0.0;
    double grid_map_width = 0.0;
    double grid_map_resolution = 0.0;
    int grid_map_data_initialize = 0;
    void read_module_planner_ini()
    {
        try
        {
            boost::property_tree::ptree pt;
            boost::property_tree::ini_parser::read_ini("/home/dengjia/DengJia_ws/src/module_planner/src/module_planner.ini", pt);
            grid_map_height = pt.get<double>("grid_map.height");
            grid_map_width = pt.get<double>("grid_map.width");
            grid_map_resolution = pt.get<double>("grid_map.resolution");
            grid_map_data_initialize = pt.get<int>("grid_map.data_initialize");
        }
        catch (const boost::property_tree::ini_parser_error &e)
        {
            // 读取失败，输出错误信息
            std::cerr << "Error reading module_planner.ini: " << e.what() << std::endl;
        }
    }

    void egoStatusCallback(const self_interface::msg::EgoStatus::SharedPtr msg)
    {
        nav_msgs::msg::OccupancyGrid local_grid_map_msg;
        local_grid_map_msg.header.stamp = this->now();
        local_grid_map_msg.header.frame_id = "/odom";
        if (std::fabs(grid_map_resolution) > 0.001)
        {
            local_grid_map_msg.info.width = grid_map_width / grid_map_resolution;
            local_grid_map_msg.info.height = grid_map_height / grid_map_resolution;
        }
        local_grid_map_msg.info.resolution = grid_map_resolution;
        local_grid_map_msg.info.resolution = grid_map_resolution;
        local_grid_map_msg.info.resolution = grid_map_resolution;
        double cos_theta = cos(msg->ego_theta);
        double sin_theta = sin(msg->ego_theta);
        double half_width = local_grid_map_msg.info.width / 2.0 * local_grid_map_msg.info.resolution;
        double half_height = local_grid_map_msg.info.height / 2.0 * local_grid_map_msg.info.resolution;
        local_grid_map_msg.info.origin.position.x = msg->ego_x -
                                                    (half_width * cos_theta - half_height * sin_theta);
        local_grid_map_msg.info.origin.position.y = msg->ego_y -
                                                    (half_width * sin_theta + half_height * cos_theta);
        local_grid_map_msg.info.origin.position.z = 0.0;
        local_grid_map_msg.info.origin.orientation.x = 0.0;
        local_grid_map_msg.info.origin.orientation.y = 0.0;
        local_grid_map_msg.info.origin.orientation.z = sin_theta / 2.0;
        local_grid_map_msg.info.origin.orientation.w = cos_theta / 2.0;

        local_grid_map_msg.info.origin.position.z = 0.0;
        local_grid_map_msg.info.origin.orientation.x = 0.0;
        local_grid_map_msg.info.origin.orientation.y = 0.0;
        local_grid_map_msg.info.origin.orientation.z = sin(msg->ego_theta / 2.0);
        local_grid_map_msg.info.origin.orientation.w = cos(msg->ego_theta / 2.0);
        local_grid_map_msg.data.resize(local_grid_map_msg.info.width * local_grid_map_msg.info.height);
        for (size_t i = 0; i < local_grid_map_msg.data.size(); ++i)
        {
            local_grid_map_msg.data[i] = grid_map_data_initialize;
        }
        local_grid_map_publisher_->publish(local_grid_map_msg);
    }

    rclcpp::Subscription<self_interface::msg::EgoStatus>::SharedPtr ego_status_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_grid_map_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalGridMapPublisher>());
    rclcpp::shutdown();
    return 0;
}
