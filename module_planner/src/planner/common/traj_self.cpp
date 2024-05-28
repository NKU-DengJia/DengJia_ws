#include "traj_self.hpp"

#include <iostream>

TrajStanley::TrajStanley(const Traj &traj_)
    : traj0(traj_)
{
    read_module_planner_ini();
    traj_result.traj_points.clear();
    if (!traj0.traj_points.empty())
    {
        ego_x = traj0.traj_points[0].x;
        ego_y = traj0.traj_points[0].y;
        // ego_yaw = traj0.traj_points[0].theta;
        // 避免使用原轨迹上的theta
        foresight_s = 0.5;
        TrajPoint foresight_point;
        CartesianFrenetConverter_method.frenet_to_cartesian(foresight_s,
                                                            traj0,
                                                            foresight_point);
        double dx = foresight_point.x - ego_x;
        double dy = foresight_point.y - ego_y;
        ego_yaw = std::atan2(dy, dx);

        traj_result.traj_points.push_back(traj0.traj_points[0]);
        traj_result.traj_points[0].s = 0.0;
    }
}
void TrajStanley::read_module_planner_ini()
{
    try
    {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini("/home/dengjia/DengJia_ws/src/module_planner/src/module_planner.ini", pt);
        kp = pt.get<double>("self_traj_smoother.kp");
        ki = pt.get<double>("self_traj_smoother.ki");
        kd = pt.get<double>("self_traj_smoother.kd");
        vel = pt.get<double>("self_traj_smoother.virtual_vehicle_speed");
    }
    catch (const boost::property_tree::ini_parser_error &e)
    {
        // 读取失败，输出错误信息
        std::cerr << "Error reading module_planner.ini: " << e.what() << std::endl;
    }
}

Traj TrajStanley::TrajStanleySmooth()
{
    int i = 0;
    int traj0_last_index = traj0.traj_points.size() - 1;
    if (traj0_last_index >= 0)
    {
        double smax = traj0.traj_points[traj0_last_index].s - traj0.traj_points[0].s; // 待平滑轨迹的长度
        // std::cout << "smax:" << smax << std::endl;
        double s = 0.0; // 新轨迹的大概长度
        while (i < 100)
        {
            s += vel * T * 10;
            if (s > smax)
            {
                break;
            }
            CalculateSmoothTraj();
            i++;
        }
        // std::cout << "s:" << s << std::endl;

        // 给轨迹点补充曲率信息
        if (traj_result.traj_points.size() <= 2)
        {
            // 不足3个点，转换成空轨迹
            traj_result.traj_points.clear();
            return traj_result;
        }
        // 中间点
        for (size_t i = 1; i < traj_result.traj_points.size() - 1; ++i)
        {
            const auto &point0 = traj_result.traj_points[i - 1];
            const auto &point1 = traj_result.traj_points[i];
            const auto &point2 = traj_result.traj_points[i + 1];

            double dx1 = point1.x - point0.x;
            double dy1 = point1.y - point0.y;
            double dx2 = point2.x - point1.x;
            double dy2 = point2.y - point1.y;
            double dx3 = point2.x - point0.x;
            double dy3 = point2.y - point0.y;

            double ds1 = sqrt(dx1 * dx1 + dy1 * dy1);
            double ds2 = sqrt(dx2 * dx2 + dy2 * dy2);
            double ds3 = sqrt(dx3 * dx3 + dy3 * dy3);

            double s = 0.5 * (ds1 + ds2 + ds3);
            double kappa = 0.0;
            double epsilon = 0.02;
            if (ds1 <= epsilon || ds2 <= epsilon || ds3 <= epsilon)
            {
                kappa = 100.0;
            }
            else
            {
                kappa = 4 * sqrt(fabs(s * (s - ds1) * (s - ds2) * (s - ds3))) / (ds1 * ds2 * ds3);
            }
            traj_result.traj_points[i].kappa = kappa;
        }
        // 更新第一个点的曲率为与第二个点相同
        traj_result.traj_points.front().kappa = traj_result.traj_points[1].kappa;
        // 更新最后一个点的曲率与倒数第二个点相同
        size_t last_index = traj_result.traj_points.size() - 1;
        traj_result.traj_points[last_index].kappa = traj_result.traj_points[last_index - 1].kappa;
    }
    return traj_result;
}

void TrajStanley::CalculateSmoothTraj()
{
    WayPoint waypoint;
    waypoint.x = ego_x;
    waypoint.y = ego_y;
    TrajPoint projection_point;
    if (calculateProjectionPointWithPose(traj0, waypoint, projection_point))
    {
        foresight_s = projection_point.s + 1.0 * vel + 0.5;
        TrajPoint foresight_point;
        CartesianFrenetConverter_method.frenet_to_cartesian(foresight_s,
                                                            traj0,
                                                            foresight_point);
        double dx = foresight_point.x - ego_x;
        double dy = foresight_point.y - ego_y;
        double target_yaw = std::atan2(dy, dx);
        double target_yaw_deg = (target_yaw / 3.14159) * 180.0;
        // std::cout << "target_yaw_deg: " << target_yaw_deg << std::endl;
        for (int j = 0; j < 10; j++)
        {
            double ego_yaw_deg = (ego_yaw / 3.14159) * 180.0;
            double ego_w = 10.0 * pid_w.PIDcalculate(target_yaw_deg, ego_yaw_deg);

            ego_x += vel * std::cos(ego_yaw) * T;
            ego_y += vel * std::sin(ego_yaw) * T;
            ego_yaw += ego_w * T;
        }

        TrajPoint temp;
        temp.x = ego_x;
        temp.y = ego_y;
        temp.theta = ego_yaw;
        traj_result.traj_points.push_back(temp);
        int end = traj_result.traj_points.size() - 1;
        if (end > 0)
        {
            double dx = traj_result.traj_points[end].x - traj_result.traj_points[end - 1].x;
            double dy = traj_result.traj_points[end].y - traj_result.traj_points[end - 1].y;
            traj_result.traj_points[end].s = traj_result.traj_points[end - 1].s + std::sqrt(dx * dx + dy * dy);
        }
    }
}