#include "rclcpp/rclcpp.hpp"
#include "self_interface/msg/chassis_cmd.hpp" // 自定义消息类型
#include "self_interface/msg/ego_status.hpp"  // 自定义消息类型
#include <algorithm>                          // 包含 std::clamp 函数所在的头文件
#include <random>
#include "eigen3/Eigen/Core"

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

/*
订阅：
    self_interface::msg::chassis_cmd    chassis_cmd        (50Hz发布)
发布：
    self_interface::msg::ego_status     ego_status_msg     (100Hz）
*/

class PIDController
{
public:
    PIDController(double kp, double ki, double kd, double integral_limit = std::numeric_limits<double>::max())
        : kp_(kp), ki_(ki), kd_(kd), integral_limit_(integral_limit), integral_(0), previous_error_(0), first_time_(true), output(0) {}

    double PIDcalculate(double xref, double x)
    {
        double e = xref - x;
        if (e > 100)
        {
            reset();
        }
        if (!first_time_)
        {
            integral_ += e;
            if (integral_limit_ != std::numeric_limits<double>::max())
            {
                integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
            }
        }
        double derivative;
        if (!first_time_)
        {
            derivative = e - previous_error_;
        }
        else
        {
            derivative = 0; // 第一次计算时，将微分置零
            // output = xref;
            first_time_ = false;
        }
        // output += kp_ * e + ki_ * integral_ + kd_ * derivative;
        output = kp_ * e + ki_ * integral_ + kd_ * derivative;

        previous_error_ = e;
        // std::cout << "err = " << e << std::endl;
        // std::cout << std::endl;
        return output;
    }

    void reset()
    {
        integral_ = 0;
        previous_error_ = 0;
        output = 0;
        first_time_ = true;
    }

private:
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double integral_limit_ = std::numeric_limits<double>::max();
    double integral_ = 0.0;
    double previous_error_ = 0.0;
    bool first_time_ = true;
    double output = 0.0;
};

class NodeDynamic : public rclcpp::Node
{
public:
    NodeDynamic() : Node("NodeDynamic")
    {
        read_chassis_sim_ini();
        // 创建订阅者对象（消息类型、话题名、回调函数、队列长度）
        subscription1_ = this->create_subscription<self_interface::msg::ChassisCmd>(
            "chassis_cmd", 10, std::bind(&NodeDynamic::chassis_cmd_callback, this, std::placeholders::_1));
        // 创建发布者对象（消息类型、话题名、队列长度）
        publisher1_ = this->create_publisher<self_interface::msg::EgoStatus>("ego_status_msg", 10);
        // 创建一个定时器,定时执行回调函数
        std::chrono::milliseconds interval(static_cast<long long>(0.5 * T * 1000));
        timer1_ = this->create_wall_timer(interval, std::bind(&NodeDynamic::timer_callback, this));

        if (use_identification_model)
        {
            X_curvature << 0.0,
                0.0,
                0.0;

            A_curvature << 0.9809, 0.0483, 0.0045,
                0.0101, 0.9283, 0.0130,
                0.0152, 0.0988, 0.5779;

            B_curvature << -0.4879,
                -2.3664,
                65.9523;

            C_curvature << -2.3232, -0.0398, -0.0208;

            D_curvature << 0.0;
        }
    }

private:
    Eigen::MatrixXd X_curvature = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd A_curvature = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd B_curvature = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd C_curvature = Eigen::MatrixXd::Zero(1, 3);
    Eigen::MatrixXd D_curvature = Eigen::MatrixXd::Zero(1, 1);

    bool use_identification_model = 1;
    void read_chassis_sim_ini()
    {
        try
        {
            boost::property_tree::ptree pt;
            boost::property_tree::ini_parser::read_ini("/home/dengjia/DengJia_ws/src/chassis_sim/src/chassis_sim.ini", pt);

            use_identification_model = pt.get<bool>("chassis_dynamic.use_identification_model");

            ego_x = pt.get<double>("ego_initial_state.initial_ego_x");
            ego_y = pt.get<double>("ego_initial_state.initial_ego_y");
            ego_theta = pt.get<double>("ego_initial_state.initial_ego_theta");
            ego_velocity = pt.get<double>("ego_initial_state.initial_ego_velocity");
            ego_w = pt.get<double>("ego_initial_state.initial_ego_w");
            ego_curvature = pt.get<double>("ego_initial_state.initial_ego_curvature");
        }
        catch (const boost::property_tree::ini_parser_error &e)
        {
            // 读取失败，输出错误信息
            std::cerr << "Error reading chassis_sim.ini: " << e.what() << std::endl;
        }
    }

    void chassis_cmd_callback(const self_interface::msg::ChassisCmd::SharedPtr msg)
    {
        /*
            float64 vel_req             # 表示期望的纵向速度
            float64 turn_radius_req     # 表示期望的转弯半径
            bool steer_mod_req          # 0中心转向，1正常行驶
            float64 center_steer_req
        */
        if (msg->steer_mod_req == 1) // 正常行驶
        {
            if (use_identification_model)
            {
                double vel_in = msg->vel_req; // 单位：m/s
                ego_velocity += 20 * (-1.0 * ego_velocity + vel_in) * T;
                if (ego_velocity > 0.5) // 速度太小认为是驻车状态
                {
                    ego_velocity += gaussianNoise(0.0, 0.03);
                }

                if (msg->turn_radius_req <= 0.001 && msg->turn_radius_req >= -0.001)
                {
                    std::cout << "Error: 控制指令错误，正常行驶下转弯半径太小！！" << std::endl;
                    return;
                }
                if (msg->turn_radius_req >= 400 || msg->turn_radius_req <= -400)
                {
                    ego_w = 0.0;
                }
                else
                {
                    // 曲率响应特性
                    double curvature_in = 1.0 / (msg->turn_radius_req);
                    Eigen::MatrixXd temp = C_curvature * X_curvature + D_curvature * curvature_in;
                    double curvature_out = temp(0, 0);
                    // std::cout << "in: " << curvature_in << ", out:" << curvature_out << std::endl;
                    X_curvature = A_curvature * X_curvature + B_curvature * curvature_in;

                    ego_w = ego_velocity * curvature_out;
                }
                ego_w = std::clamp(ego_w, -5.0, 5.0);
            }
            else
            {
                ego_velocity = msg->vel_req;
                if (msg->turn_radius_req <= 0.001 && msg->turn_radius_req >= -0.001)
                {
                    std::cout << "Error: 控制指令错误，正常行驶下转弯半径太小！！" << std::endl;
                    return;
                }
                if (msg->turn_radius_req >= 400 || msg->turn_radius_req <= -400)
                {
                    ego_w = 0.0;
                }
                else
                {
                    ego_w = ego_velocity / msg->turn_radius_req;
                }
            }
        }
        else
        { // 中心转向
            ego_velocity = 0;
            double ego_yaw_deg = (ego_theta / 3.14159) * 180.0;
            ego_w = pid_CenterSteer.PIDcalculate(msg->center_steer_req, ego_yaw_deg);
        }

        ego_x += ego_velocity * cos(ego_theta) * T;
        ego_y += ego_velocity * sin(ego_theta) * T;
        ego_theta += ego_w * T;
        ego_theta = NormalizeAngle(ego_theta);

        if (ego_velocity > 0.5 || ego_velocity <= -0.5)
        {
            ego_curvature = ego_w / ego_velocity;
        }
        else if (ego_velocity <= 0.5)
        {
            ego_curvature = 100.0; // 中心转向
        }
        else
        {
            ego_curvature = -100.0; // 中心转向
        }
    }

    double NormalizeAngle(const double angle)
    {
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0)
        {
            a += (2.0 * M_PI);
        }
        return a - M_PI;
    }

    void timer_callback()
    {
        // 构建EgoStatus消息
        auto ego_status_msg = std::make_shared<self_interface::msg::EgoStatus>();
        // 填充消息
        ego_status_msg->header.stamp = this->now();
        ego_status_msg->header.frame_id = "/odom"; // 使用odom坐标系
        ego_status_msg->ego_x = ego_x;
        ego_status_msg->ego_y = ego_y;
        ego_status_msg->ego_theta = ego_theta;
        ego_status_msg->ego_velocity = ego_velocity;
        ego_status_msg->ego_w = ego_w;
        ego_status_msg->ego_curvature = ego_curvature;

        // 发布EgoStatus消息
        publisher1_->publish(*ego_status_msg);
    }

    // 订阅者
    rclcpp::Subscription<self_interface::msg::ChassisCmd>::SharedPtr subscription1_;

    // 发布者
    rclcpp::TimerBase::SharedPtr timer1_;                                     // 定时器指针
    rclcpp::Publisher<self_interface::msg::EgoStatus>::SharedPtr publisher1_; // 发布者指针

    double T = 0.02; // 发布间隔，以秒为单位

    double ego_x;
    double ego_y;
    double ego_theta;
    double ego_velocity;
    double ego_w;
    double ego_curvature;

    PIDController pid_CenterSteer = PIDController(0.02, 0.0, 0.0001);

    // 生成服从均值为mean，标准差为stddev的高斯分布随机数
    double gaussianNoise(double mean, double stddev)
    {
        static std::mt19937 gen(std::random_device{}());
        std::normal_distribution<double> dist(mean, stddev);
        return dist(gen);
    }
};

int main(int argc, char *argv[])
{
    // 初始化ROS 2运行时环境
    rclcpp::init(argc, argv);

    auto node_chassis = std::make_shared<NodeDynamic>();
    // 执行节点主循环
    rclcpp::spin(node_chassis);

    // 清理资源
    rclcpp::shutdown();

    return 0;
}
