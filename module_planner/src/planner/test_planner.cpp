#include "common/common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "self_interface/msg/planner_traj.hpp"       // 自定义消息类型
#include "self_interface/msg/planner_traj_point.hpp" // 自定义消息类型
#include "self_interface/msg/ego_status.hpp"         // 自定义消息类型
#include "common/referenceline.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "common/traj_sampling.hpp"

#include "common/traj_self.hpp"

#include <iostream>
#include <cmath>
#include <chrono>

/*
    发布：
        self_interface::msg::PlannerTraj    planner_traj    5Hz
    订阅：
        self_interface::msg::EgoStatus      ego_status_msg (与odom发布频率相同，具体多少Hz暂时未知)
        visualization_msgs::msg::Marker     static_obstacles
*/
class TrajPublisher : public rclcpp::Node
{
public:
    TrajPublisher() : Node("NodeTrajPublisher")
    {
        // 创建订阅者对象（消息类型、话题名、回调函数、队列长度）
        subscription1_ = this->create_subscription<self_interface::msg::EgoStatus>(
            "ego_status_msg", 10, std::bind(&TrajPublisher::ego_status_callback, this, std::placeholders::_1));

        // 读取.txt路网，生成参考线
        referenceline_.GenerateReferenceLine(referenceline_traj_);
        //
        subscription2_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "static_obstacles", 10, std::bind(&TrajPublisher::static_obstacles_callback, this, std::placeholders::_1));

        // 发布planner_traj
        traj_publisher_ = this->create_publisher<self_interface::msg::PlannerTraj>("planner_traj", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&TrajPublisher::publishTraj, this));
    }

private:
    void ego_status_callback(const self_interface::msg::EgoStatus msg)
    {
        EgoStatus_.ego_x = msg.ego_x;
        EgoStatus_.ego_y = msg.ego_y;
        EgoStatus_.ego_theta = msg.ego_theta;
        EgoStatus_.ego_velocity = msg.ego_velocity;
        EgoStatus_.ego_w = msg.ego_w;
        EgoStatus_.ego_curvature = msg.ego_curvature;
    }

    void static_obstacles_callback(const visualization_msgs::msg::Marker msg)
    {
        StaticObstacle temp;
        temp.center_x = msg.pose.position.x;
        temp.center_y = msg.pose.position.y;
        // 设置阈值
        const double distance_threshold = 0.1; // 设定一个阈值，单位是米
        // 检查是否已经存在相近的元素
        bool duplicate_found = false;
        for (const auto &obstacle : static_obstacles.static_obstacle_set)
        {
            double distance = sqrt(pow(obstacle.center_x - temp.center_x, 2) + pow(obstacle.center_y - temp.center_y, 2));
            if (distance < distance_threshold)
            {
                duplicate_found = true;
                break;
            }
        }
        // 如果未找到相近的元素，则添加到static_obstacles.static_obstacle_set中
        if (!duplicate_found)
        {
            static_obstacles.static_obstacle_set.push_back(temp);
        }
        // std::cout << "障碍物个数: " << static_obstacles.static_obstacle_set.size() << std::endl;
    }

    void publishTraj()
    {
        static_obstacles.set_s_l(referenceline_traj_); // 计算障碍物s和l
        DeterminPlanStart();
        traj_ = traj_sampling.obtain_result_traj(plannering_start_point,
                                                 static_obstacles,
                                                 referenceline_traj_); // 采样生成的待平滑的轨迹
        TrajStanley TrajStanleySmoothMethod = TrajStanley(traj_);
        traj_ = TrajStanleySmoothMethod.TrajStanleySmooth(); // 平滑后的轨迹
        auto msg = std::make_shared<self_interface::msg::PlannerTraj>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "/odom";
        // 遍历 traj_ 中的每个轨迹点，将其转换为 PlannerTrajPoint，并添加到 msg->traj_points 中
        for (const auto &traj_point : traj_.traj_points)
        {
            msg->traj_points.push_back(convertTomsg(traj_point));
        }
        traj_publisher_->publish(*msg);
    }

    void DeterminPlanStart()
    {
        WayPoint waypoint;
        waypoint.x = EgoStatus_.ego_x;
        waypoint.y = EgoStatus_.ego_y;
        TrajPoint projection_point; // 注意这是新轨迹上的投影点
        if (!calculateProjectionPointWithPose(traj_, waypoint, projection_point))
        {
            plannering_start_point.x = EgoStatus_.ego_x;
            plannering_start_point.y = EgoStatus_.ego_y;
            plannering_start_point.theta = EgoStatus_.ego_theta;
            return;
        }
        else
        {
            double dx = projection_point.x - EgoStatus_.ego_x;
            double dy = projection_point.y - EgoStatus_.ego_y;
            double dtheta = projection_point.theta - EgoStatus_.ego_theta;
            double theta_thereshold = 60.0 * M_PI / 180.0;
            if ((dx * dx + dy * dy) > 0.5 || std::abs(dtheta) > theta_thereshold)
            {
                plannering_start_point.x = EgoStatus_.ego_x;
                plannering_start_point.y = EgoStatus_.ego_y;
                plannering_start_point.theta = EgoStatus_.ego_theta;
                std::cout << "重规划！！" << std::endl;
                std::cout << "位置差: " << std::sqrt(dx * dx + dy * dy)
                          << "航向差: " << std::abs(dtheta) << std::endl;
                return;
            }
            else
            {
                plannering_start_point.x = projection_point.x;
                plannering_start_point.y = projection_point.y;
                plannering_start_point.theta = projection_point.theta;
            }
        }
    }

    void debug(Traj traj_)
    {
        std::cout << "traj_size:" << traj_.traj_points.size() << std::endl;
    }

    // 定义一个函数，将TrajPoint转换为自定义消息：PlannerTrajPoint
    self_interface::msg::PlannerTrajPoint convertTomsg(const TrajPoint &traj_point)
    {
        self_interface::msg::PlannerTrajPoint planner_traj_point;
        planner_traj_point.x = traj_point.x;
        planner_traj_point.y = traj_point.y;
        planner_traj_point.s = traj_point.s;
        planner_traj_point.l = traj_point.l;
        planner_traj_point.kappa = traj_point.kappa;
        planner_traj_point.dkappa = traj_point.dkappa;
        planner_traj_point.theta = traj_point.theta;
        planner_traj_point.v = traj_point.v;
        planner_traj_point.t = traj_point.t;
        return planner_traj_point;
    }

    rclcpp::Publisher<self_interface::msg::PlannerTraj>::SharedPtr traj_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<self_interface::msg::EgoStatus>::SharedPtr subscription1_; // 订阅者指针
    EgoStatus EgoStatus_;

    Traj referenceline_traj_;
    ReferenceLine referenceline_;

    TrajSampling traj_sampling;
    TrajPoint plannering_start_point; // 规划起点(要x，y，theta(确定起点的dl/ds))

    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription2_; // 订阅者指针
    StaticObstacleSet static_obstacles;                                              // 静态障碍物集合

    Traj traj_; // 规划出的局部轨迹

    CartesianFrenetConverter CartesianFrenetConverterMethed;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajPublisher>());
    rclcpp::shutdown();
    return 0;
}
