#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist.hpp"  
#include <iostream>
#include <algorithm> // 用于 std::clamp
#include <limits>
using namespace std::chrono_literals;
#include "self_interface/msg/chassis_cmd.hpp" // 自定义消息类型
#include "self_interface/msg/ego_status.hpp" // 自定义消息类型

struct ChassisCmd {
    double vel_req;
    double turn_radius_req;
    bool SteerModReq;  // 0中心转向，1正常行驶
    double CenterSteerReq; // 中心转向请求，-180 deg ~ 180 deg，目标航向角
    ChassisCmd() : vel_req(0.0), turn_radius_req(1000.0), SteerModReq(1), CenterSteerReq(0.0) {};
};
    
class PIDController {
public:
    PIDController(double kp, double ki, double kd, double integral_limit = std::numeric_limits<double>::max())
        : kp_(kp), ki_(ki), kd_(kd), integral_limit_(integral_limit), integral_(0), previous_error_(0), first_time_(true), output(0){}

    double PIDcalculate(double xref, double x) {
        double e = xref - x;
        if(e>100){
            reset();
        }
        if (!first_time_) {
            integral_ += e;
            if (integral_limit_ != std::numeric_limits<double>::max()) {
                integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
            }
        } 
        double derivative;
        if (!first_time_) {
            derivative = e - previous_error_;
        } else {
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

    void reset() {
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
    double previous_error_  = 0.0;
    bool first_time_ = true;
    double output  = 0.0;
};

class AddPIDController {
public:
    AddPIDController(double kp, double ki, double kd, double integral_limit = std::numeric_limits<double>::max())
        : kp_(kp), ki_(ki), kd_(kd), integral_limit_(integral_limit), integral_(0), previous_error_(0), first_time_(true), output(0){}

    double PIDcalculate(double xref, double x) {
        double e = xref - x;
        if(e>100){
            reset();
        }
        if (!first_time_) {
            integral_ += e;
            if (integral_limit_ != std::numeric_limits<double>::max()) {
                integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
            }
        } 
        double derivative;
        if (!first_time_) {
            derivative = e - previous_error_;
        } else {
            derivative = 0; // 第一次计算时，将微分置零
            output = xref;
            first_time_ = false;
        }
        output += kp_ * e + ki_ * integral_ + kd_ * derivative;
        // output = kp_ * e + ki_ * integral_ + kd_ * derivative;

        previous_error_ = e;
        // std::cout << "err = " << e << std::endl;
        // std::cout << std::endl;
        return output;
    }

    void reset() {
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
    double previous_error_  = 0.0;
    bool first_time_ = true;
    double output  = 0.0;
};

/*
订阅：
    odom
    chassis_cmd
发布：
    twist  50Hz              给底盘控制器
    nav_msgs::msg::Path  path_msg                       与odom消息同频率
    self_interface::msg::ego_status  ego_status_msg     与odom消息同频率
*/
class NodeChassis : public rclcpp::Node{
public:
    NodeChassis(): Node("NodeChassis") {
        // 创建订阅者对象（消息类型、话题名、回调函数、队列长度）
        subscription1_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&NodeChassis::odom_callback, this, std::placeholders::_1));
        subscription2_ = this->create_subscription<self_interface::msg::ChassisCmd>(
            "chassis_cmd", 10, std::bind(&NodeChassis::chassis_cmd_callback, this, std::placeholders::_1));
        // 创建发布者对象（消息类型、话题名、队列长度）
        publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // 创建一个定时器,定时执行回调函数
        std::chrono::milliseconds interval(static_cast<long long>(T*1000));
        timer1_ = this->create_wall_timer(interval, std::bind(&NodeChassis::timer_callback, this));

        publisher_path_ = this->create_publisher<nav_msgs::msg::Path>("path_msg", 10);

        publisher_ego_status_ = this->create_publisher<self_interface::msg::EgoStatus>("ego_status_msg", 10);
    };

private:
    // rclcpp::NodeOptions options;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        ego_x = msg->pose.pose.position.x;
        ego_y = msg->pose.pose.position.y;
        ego_z = msg->pose.pose.position.z;

        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);
        tf2::Matrix3x3 mat(quat);
        mat.getRPY(ego_roll, ego_pitch, ego_yaw);

        v_x = msg->twist.twist.linear.x;
        v_y = msg->twist.twist.linear.y;
        v_z = msg->twist.twist.linear.z;

        w_x = msg->twist.twist.angular.x;
        w_y = msg->twist.twist.angular.y;
        w_z = msg->twist.twist.angular.z;

        ego_turn_radius = v_x / w_z;
        ego_turn_radius = std::clamp(ego_turn_radius, -Rmax, Rmax);

        odom_msg_2_path_msg(msg); // 发布path_msg
        odom_msg_2_ego_status_msg(); // ego_status_msg
        // Debug_PrintEgoStatus();
    }
    void Debug_PrintEgoStatus() {
        std::cout << "Ego Position: (" << ego_x << ", " << ego_y << ", " << ego_z << ")" << std::endl;
        std::cout << "Ego Orientation (RPY): (" << ego_roll << ", " << ego_pitch << ", " << ego_yaw << ")" << std::endl;
        std::cout << "Linear Velocity: (" << v_x << ", " << v_y << ", " << v_z << ")" << std::endl;
        std::cout << "Angular Velocity: (" << w_x << ", " << w_y << ", " << w_z << ")" << std::endl;
        std::cout << "Turn Radius: (" << ego_turn_radius << ")" << std::endl;
        std::cout << std::endl;
    }
    void odom_msg_2_path_msg(const nav_msgs::msg::Odometry::SharedPtr msg){
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "/odom"; // 使用odom坐标系
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path_msg.header;
        pose_stamped.pose.position.x = msg->pose.pose.position.x;
        pose_stamped.pose.position.y = msg->pose.pose.position.y;
        pose_stamped.pose.position.z = msg->pose.pose.position.z;
        pose_stamped.pose.orientation = msg->pose.pose.orientation;
        path_msg.poses.push_back(pose_stamped);
        publisher_path_->publish(path_msg);
    }
    void odom_msg_2_ego_status_msg(){
        self_interface::msg::EgoStatus ego_status_msg;
        ego_status_msg.header.stamp = this->now();
        ego_status_msg.header.frame_id = "/odom"; // 使用odom坐标系

        ego_status_msg.ego_x = ego_x;
        ego_status_msg.ego_y = ego_y;
        ego_status_msg.ego_theta = ego_yaw;
        ego_status_msg.ego_velocity = v_x;
        ego_status_msg.ego_w = w_z;

        if (v_x <= 0.5 && v_x >= 0.0){
            ego_status_msg.ego_curvature = 2.0; // 转弯半径很小，中心转向；曲率置为2.0
        }else if(v_x >= -0.5 && v_x < 0.0){
            ego_status_msg.ego_curvature = -2.0; // 转弯半径很小，中心转向；曲率置为-2.0
        }else{
            ego_status_msg.ego_curvature = w_z / v_x;
        }

        if ((ego_status_msg.ego_curvature >= -1.0/Rmax) && (ego_status_msg.ego_curvature <= 1.0/Rmax)){
            ego_status_msg.ego_curvature = 0;  // 直线
        }

        publisher_ego_status_->publish(ego_status_msg);
    }

    void chassis_cmd_callback(const self_interface::msg::ChassisCmd::SharedPtr msg){
        ChassisCmd_.SteerModReq = msg->steer_mod_req;
        ChassisCmd_.CenterSteerReq = msg->center_steer_req;
        ChassisCmd_.turn_radius_req = msg->turn_radius_req;
        ChassisCmd_.vel_req = msg->vel_req;
    }

    int n = 0;
    void timer_callback(){
        /*
        n++;
        if (n > 20)
        {
            n = 0;
            if (ChassisCmd_.SteerModReq == 0){
                std::cout <<"中心转向模式："<< std::endl;
                std::cout <<"            "<<"ChassisCmd_.CenterSteerReq: "<<ChassisCmd_.CenterSteerReq << std::endl;
                std::cout <<"            "<<"  误差: "<<ChassisCmd_.CenterSteerReq - ((ego_yaw / 3.14159) * 180.0) << std::endl;
            }else{
                std::cout <<"正常转向模式："<< std::endl;
                std::cout <<"            "<<"ChassisCmd_.vel_req: "<<ChassisCmd_.vel_req << std::endl;
                std::cout <<"            "<<"turn_radius_req: "<<ChassisCmd_.turn_radius_req << std::endl;
                std::cout <<"            "<<"  速度误差: "<<ChassisCmd_.vel_req - v_x << std::endl;
                std::cout <<"            "<<"  实际转弯半径: "<<ego_turn_radius << std::endl;
                std::cout <<"            "<<"  曲率误差: "<<1.0/ChassisCmd_.turn_radius_req - 1.0/ego_turn_radius << std::endl;
            }
        }
        */
        double linear_vel = ChassisCmd_.vel_req;
        //double linear_vel = pid_vel.PIDcalculate(ChassisCmd_.vel_req, v_x);
        double angular_vel = 0;
        if (ChassisCmd_.SteerModReq == 1){
            if (ChassisCmd_.turn_radius_req >=Rmax || ChassisCmd_.turn_radius_req <= -Rmax){
            // 转弯半径太大，认为是直线
            angular_vel = 0;
            }
            else if (ChassisCmd_.turn_radius_req >=Rmin || ChassisCmd_.turn_radius_req <= -Rmin){
                // 正常行驶下转向
                angular_vel = ChassisCmd_.vel_req / ChassisCmd_.turn_radius_req;
                //angular_vel = (v_x / ChassisCmd_.turn_radius_req);
            }
            else{ // 转弯半径很小，需要中心转向
                linear_vel = 0.0;
                angular_vel = 0.0;
                std::cout << "目标曲率太大" << std::endl;
            }
        }
        else{
            linear_vel = 0.0;
            double ego_yaw_deg = (ego_yaw / 3.14159) * 180.0;
            /* 
            中心转向初始的ego_yaw有问题，刚开始转向的时候还没发过来
            double initial_ego_yaw = 0.0; 
            if (initial_ego_yaw_flag){
                initial_ego_yaw = ego_yaw_deg;
                initial_ego_yaw_flag = 0;
            }
            std::cout << "初始Yaw:" << initial_ego_yaw << std::endl;
            */

            if (ChassisCmd_.CenterSteerReq <=180 && ChassisCmd_.CenterSteerReq > -180){
                // angular_vel = pid_CenterSteer.PIDcalculate(initial_ego_yaw + ChassisCmd_.CenterSteerReq, ego_yaw_deg);
                // 直接给成目标航向
                angular_vel = pid_CenterSteer.PIDcalculate(ChassisCmd_.CenterSteerReq, ego_yaw_deg);
            }
            else{
                angular_vel = 0.0;
                std::cout << "目标中心转向角度不在范围内" << std::endl;
            }
            double e = ChassisCmd_.CenterSteerReq - ego_yaw_deg;
            if (e < 5 && e > -5){
                angular_vel = 0.0;
                // std::cout << "中心转向成功！" << std::endl;
                linear_vel = 0.0;
                angular_vel = 0.0;
            }
        }
        publish_cmd(linear_vel, angular_vel);
    }
    void publish_cmd(double linear_vel, double angular_vel)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_vel;
        msg.angular.z = angular_vel;
        publisher1_->publish(msg);
    }
    
    double deg2rad(double deg){
        if(deg>180){
            deg -= 360;
        }
        if(deg<=-180){
            deg += 360;
        }
        if(deg>-180 && deg <=180){
            return (deg/180)*3.14159265;
        }
        else{
            deg2rad(deg);
        }
    }

    // 订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription1_;
    rclcpp::Subscription<self_interface::msg::ChassisCmd>::SharedPtr subscription2_;
    // 位置、姿态；速度、角速度
    double ego_x = 0.0;
    double ego_y = 0.0;
    double ego_z = 0.0;

    double ego_roll = 0.0;
    double ego_pitch = 0.0;
    double ego_yaw = 0.0;

    double v_x = 0.0;
    double v_y = 0.0;
    double v_z = 0.0;

    double w_x = 0.0;
    double w_y = 0.0;
    double w_z = 0.0;

    double ego_turn_radius = Rmax;

    // 发布者1
    rclcpp::TimerBase::SharedPtr timer1_;                               // 定时器指针
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_; // 发布者指针
    // 发布者2
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_path_; // 发布者指针
    // 发布者3
    rclcpp::Publisher<self_interface::msg::EgoStatus>::SharedPtr publisher_ego_status_; // 发布者指针

    ChassisCmd ChassisCmd_;
    // bool initial_ego_yaw_flag = 1;

    double T = 0.02;  // 控制指令发布间隔，以秒为单位
    double Rmax = 400; // 最大转弯半径，大于等于400认为是直线，角速度置0
    double Rmin = 2;   // 正常行驶下允许的最小转弯半径

    AddPIDController pid_vel = AddPIDController(0.0002, 0.0, 0.01);
    PIDController pid_CenterSteer = PIDController(0.02, 0.0, 0.0001);
}; 


int main(int argc, char * argv[]) {
    // 初始化ROS 2运行时环境
    rclcpp::init(argc, argv);

    auto node_chassis = std::make_shared<NodeChassis>();
    // 执行节点主循环
    rclcpp::spin(node_chassis);

    // 清理资源
    rclcpp::shutdown();

    return 0;
}

