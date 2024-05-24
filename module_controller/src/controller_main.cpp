#include <memory>
#include <chrono>
#include <functional>
#include <string>
using namespace std::chrono_literals;
#include "rclcpp/rclcpp.hpp"
#include "self_interface/msg/chassis_cmd.hpp"        // 自定义消息类型
#include "self_interface/msg/ego_status.hpp"         // 自定义消息类型
#include "self_interface/msg/planner_traj.hpp"       // 自定义消息类型
#include "self_interface/msg/planner_traj_point.hpp" // 自定义消息类型
#include <iostream>
#include "math/controller_base.hpp"
#include "controller_lat_lqr.hpp"
#include "controller_lat_ESO_lqr.hpp"
#include "self_interface/msg/control_error.hpp" // 自定义消息类型

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

// #include <iostream>

/*
订阅：
    self_interface::msg::PlannerTraj  planner_traj (5Hz发布)
    self_interface::msg::EgoStatus    ego_status_msg (与odom发布频率相同，具体多少Hz暂时未知)
发布：
    self_interface::msg::ChassisCmd   chassis_cmd  (50Hz)
    self_interface::msg::ControlError control_error  (50Hz)
*/
class ChassisCmdPublisherNode : public rclcpp::Node
{
public:
    ChassisCmdPublisherNode() : Node("ChassisCmd_publisher")
    {
        read_module_controller_ini();
        // 创建发布者对象（消息类型、话题名、队列长度）
        publisher1_ = this->create_publisher<self_interface::msg::ChassisCmd>("chassis_cmd", 10);
        publisher2_ = this->create_publisher<self_interface::msg::ControlError>("control_error", 10);
        // 创建一个定时器,定时执行回调函数
        timer_ = this->create_wall_timer(
            20ms, std::bind(&ChassisCmdPublisherNode::timer_callback, this));

        // 创建订阅者对象（消息类型、话题名、回调函数、队列长度）
        subscription1_ = this->create_subscription<self_interface::msg::PlannerTraj>(
            "planner_traj", 10, std::bind(&ChassisCmdPublisherNode::planner_traj_callback, this, std::placeholders::_1));

        // 创建订阅者对象（消息类型、话题名、回调函数、队列长度）
        subscription2_ = this->create_subscription<self_interface::msg::EgoStatus>(
            "ego_status_msg", 10, std::bind(&ChassisCmdPublisherNode::ego_status_callback, this, std::placeholders::_1));
    }

private:
    int lat_controller_type = 0;
    double expected_speed = 0.0; // km/h
    void read_module_controller_ini()
    {
        try
        {
            boost::property_tree::ptree pt;
            boost::property_tree::ini_parser::read_ini("/home/dengjia/DengJia_ws/src/module_controller/src/module_controller.ini", pt);
            lat_controller_type = pt.get<int>("module_controller.lat_controller_type");
            expected_speed = pt.get<int>("module_controller.expected_speed");
        }
        catch (const boost::property_tree::ini_parser_error &e)
        {
            // 读取失败，输出错误信息
            std::cerr << "Error reading module_controller.ini: " << e.what() << std::endl;
        }
    }

    void planner_traj_callback(const self_interface::msg::PlannerTraj::SharedPtr msg)
    {
        PlannerTraj_.traj_points.clear(); // 先清空traj_points
        for (const auto &traj_point_msg : msg->traj_points)
        {
            TrajPoint traj_point(traj_point_msg.x, traj_point_msg.y, traj_point_msg.s,
                                 traj_point_msg.l, traj_point_msg.kappa, traj_point_msg.dkappa, traj_point_msg.theta,
                                 traj_point_msg.v, traj_point_msg.t);

            // std::cout << "x: " << traj_point_msg.x << ", "
            //           << "y: " << traj_point_msg.y << ", "
            //           << "s: " << traj_point_msg.s << ", "
            //           << "l: " << traj_point_msg.l << ", "
            //           << "kappa: " << traj_point_msg.kappa << ", "
            //           << "dkappa: " << traj_point_msg.dkappa << ", "
            //           << "theta: " << traj_point_msg.theta << ", "
            //           << "v: " << traj_point_msg.v << ", "
            //           << "t: " << traj_point_msg.t << std::endl;

            PlannerTraj_.traj_points.push_back(traj_point);
        }
    }

    void ego_status_callback(const self_interface::msg::EgoStatus msg)
    {
        EgoStatus_.ego_x = msg.ego_x;
        EgoStatus_.ego_y = msg.ego_y;
        EgoStatus_.ego_theta = msg.ego_theta;
        EgoStatus_.ego_velocity = msg.ego_velocity;
        EgoStatus_.ego_w = msg.ego_w;
        EgoStatus_.ego_curvature = msg.ego_curvature;
    }

    // 创建定时器周期执行的回调函数: 给底盘发布控制消息; 发布控制误差
    void timer_callback()
    {
        if (!PlannerTraj_.traj_points.empty())
        {
            switch (lat_controller_type)
            {
            case 0:
                lat_lqr_control_.calculateControlCmd(EgoStatus_, PlannerTraj_, ChassisCmd_);
                lat_lqr_control_.getControlError(ControlError_);
                break;
            case 1:
                lat_lqr_ESO_control_.calculateControlCmd(EgoStatus_, PlannerTraj_, ChassisCmd_);
                lat_lqr_ESO_control_.getControlError(ControlError_);
                break;
            default:
                std::cerr << "Unknown lateral controller type: " << lat_controller_type << std::endl;
                break;
            }
            ChassisCmd_.vel_req = expected_speed / 3.6; // 将来的纵向控制
        }
        else
        {
            ChassisCmd_.vel_req = 0.0;
            ChassisCmd_.turn_radius_req = 500.0;
        }

        auto msg_ChassisCmd = self_interface::msg::ChassisCmd();
        msg_ChassisCmd.header.stamp = this->now(); // 设置消息的时间戳为当前时间
        msg_ChassisCmd.vel_req = ChassisCmd_.vel_req;
        msg_ChassisCmd.turn_radius_req = ChassisCmd_.turn_radius_req;
        msg_ChassisCmd.steer_mod_req = ChassisCmd_.SteerModReq;
        msg_ChassisCmd.center_steer_req = ChassisCmd_.CenterSteerReq;
        publisher1_->publish(msg_ChassisCmd);

        auto msg_ControlError = self_interface::msg::ControlError();
        msg_ControlError.header.stamp = this->now(); // 设置消息的时间戳为当前时间
        msg_ControlError.lateral_err = ControlError_.lateral_err;
        msg_ControlError.yaw_err = ControlError_.yaw_err;
        msg_ControlError.dot_yaw_err = ControlError_.dot_yaw_err;

        ControlError_.vel_err = EgoStatus_.ego_velocity - ChassisCmd_.vel_req;
        msg_ControlError.vel_err = ControlError_.vel_err;
        publisher2_->publish(msg_ControlError);
    }

    rclcpp::TimerBase::SharedPtr timer_;                                       // 定时器指针
    rclcpp::Publisher<self_interface::msg::ChassisCmd>::SharedPtr publisher1_; // 发布者指针
    rclcpp::Publisher<self_interface::msg::ControlError>::SharedPtr publisher2_;

    rclcpp::Subscription<self_interface::msg::PlannerTraj>::SharedPtr subscription1_; // 订阅者指针
    rclcpp::Subscription<self_interface::msg::EgoStatus>::SharedPtr subscription2_;   // 订阅者指针

    ChassisCmd ChassisCmd_;
    PlannerTraj PlannerTraj_;
    EgoStatus EgoStatus_;

    lqr_control lat_lqr_control_;
    lqr_ESO_control lat_lqr_ESO_control_;
    ControlError ControlError_;
};

// ROS2节点主入口main函数
int main(int argc, char *argv[])
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);

    // 创建ROS2节点对象并进行初始化
    rclcpp::spin(std::make_shared<ChassisCmdPublisherNode>());

    // 关闭ROS2 C++接口
    rclcpp::shutdown();

    return 0;
}