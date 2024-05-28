#include "common.hpp"

#include <iostream>

Path Way::toPath(double starting_s) const
{
    Path path;
    if (way_points.size() <= 2)
    {
        // 不足3个点，转换成空path
        return path;
    }
    double cumulative_s = starting_s;
    // 第一个点
    const auto &first_point = way_points[0];
    const auto &second_point = way_points[1];
    double dx_first = second_point.x - first_point.x;
    double dy_first = second_point.y - first_point.y;
    double theta_first = atan2(dy_first, dx_first);
    path.path_points.emplace_back(first_point.x, first_point.y, cumulative_s, 0.0, 0.0, 0.0, theta_first);
    // 中间点
    for (size_t i = 1; i < way_points.size() - 1; ++i)
    {
        const auto &point0 = way_points[i - 1];
        const auto &point1 = way_points[i];
        const auto &point2 = way_points[i + 1];

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

        // std::cout << "kappa: " << kappa << ", "
        //           << "fenzi:" << s * (s - ds1) * (s - ds2) * (s - ds3) << ", "
        //           << "fenmu:" << (ds1 * ds2 * ds3) << ", "
        //           << "ds1: " << ds1 << ", "
        //           << "ds2: " << ds2 << ", "
        //           << "ds3: " << ds3 << ", "
        //           << "s: " << s << std::endl;

        cumulative_s += ds1;
        double theta = atan2(dy2, dx2);
        path.path_points.emplace_back(point1.x, point1.y, cumulative_s, 0.0, kappa, 0.0, theta);
    }

    // 处理最后一个点
    size_t last_index = way_points.size() - 1;
    const auto &last_point = way_points[last_index];
    const auto &second_last_point = way_points[last_index - 1];
    double dx_last = last_point.x - second_last_point.x;
    double dy_last = last_point.y - second_last_point.y;
    double theta_last = atan2(dy_last, dx_last);
    path.path_points.emplace_back(last_point.x, last_point.y, cumulative_s, 0.0, 0.0, 0.0, theta_last);
    // 更新第一个点的曲率为与第二个点相同
    path.path_points.front().kappa = path.path_points[1].kappa;
    // 更新最后一个点的曲率与倒数第二个点相同
    path.path_points[last_index].kappa = path.path_points[last_index - 1].kappa;
    return path;
}

Traj Way::toTraj(double starting_s) const
{
    Path path = this->toPath(starting_s);
    return path.toTraj();
}

Way Path::toWay() const
{
    Way way;
    for (const auto &point : path_points)
    {
        way.way_points.emplace_back(point.x, point.y);
    }
    return way;
}

Traj Path::toTraj() const
{
    Traj traj;
    for (const auto &point : path_points)
    {
        traj.traj_points.emplace_back(point.x, point.y, point.s, point.l, point.kappa, point.dkappa, point.theta);
    }
    return traj;
}

Way Traj::toWay() const
{
    Way way;
    for (const auto &point : traj_points)
    {
        way.way_points.emplace_back(point.x, point.y);
    }
    return way;
}

bool calculateProjectionPointWithPose(const Traj traj, const WayPoint waypoint,
                                      TrajPoint &point)
{
    int traj_size = traj.traj_points.size();
    double min_dist = 10e6;
    int min_temp = -1;
    for (int i = 0; i < traj_size; i++)
    {
        double dx = waypoint.x - traj.traj_points[i].x;
        double dy = waypoint.y - traj.traj_points[i].y;
        double dist = sqrt(dx * dx + dy * dy);
        if (dist < min_dist && dist < 10.0)
        {
            min_dist = dist;
            min_temp = i;
        }
    }
    if (min_temp == -1)
    {
        std::cout << "规划：参考线上, could not find match point !!!" << std::endl;
        std::cout << "traj.traj_points.size(): " << traj.traj_points.size() << std::endl;
        if (traj.traj_points.size() > 0)
        {
            std::cout << "traj.traj_points[0]: " << traj.traj_points[0].x << ","
                      << traj.traj_points[0].y << std::endl;
        }
        std::cout << "point: " << waypoint.x << ", " << waypoint.y << std::endl;

        return false;
    }
    // std::cout << "参考线上, success !!!"
    //           << "轨迹长度: " << traj_size
    //           << ", min_temp: " << min_temp << ", kappa: " << traj.traj_points[min_temp].kappa
    //           << ", x:" << traj.traj_points[min_temp].x
    //           << ", y:" << traj.traj_points[min_temp].y << std::endl;
    if (traj_size == 1)
    {
        point = traj.traj_points.front();
        return true;
    }
    int temp_next;
    double factor = 0.0;
    int final_next;
    if (min_temp == 0)
    {
        temp_next = min_temp + 1;
        Eigen::Vector2d v1(waypoint.x - traj.traj_points[min_temp].x, waypoint.y - traj.traj_points[min_temp].y);
        Eigen::Vector2d v2(traj.traj_points[temp_next].x - traj.traj_points[min_temp].x, traj.traj_points[temp_next].y - traj.traj_points[min_temp].y);
        double vector_dot = v1.dot(v2);
        if (vector_dot < 0)
        {
            point = traj.traj_points[min_temp];
            return true;
        }
        else
        {
            factor = (vector_dot / v2.norm()) / v2.norm();

            final_next = temp_next;
        }
    }
    else if (min_temp == traj_size - 1)
    {
        temp_next = min_temp - 1;
        Eigen::Vector2d v1(waypoint.x - traj.traj_points[min_temp].x, waypoint.y - traj.traj_points[min_temp].y);
        Eigen::Vector2d v2(traj.traj_points[temp_next].x - traj.traj_points[min_temp].x, traj.traj_points[temp_next].y - traj.traj_points[min_temp].y);
        double vector_dot = v1.dot(v2);
        if (vector_dot < 0)
        {
            point = traj.traj_points[min_temp];
            return true;
        }
        else
        {
            factor = (vector_dot / v2.norm()) / v2.norm();
            final_next = temp_next;
        }
    }
    else
    {
        temp_next = min_temp + 1;
        int temp_before = min_temp - 1;
        Eigen::Vector2d v1(waypoint.x - traj.traj_points[min_temp].x, waypoint.y - traj.traj_points[min_temp].y);
        Eigen::Vector2d v2(traj.traj_points[temp_next].x - traj.traj_points[min_temp].x, traj.traj_points[temp_next].y - traj.traj_points[min_temp].y);
        Eigen::Vector2d v3(traj.traj_points[temp_before].x - traj.traj_points[min_temp].x, traj.traj_points[temp_before].y - traj.traj_points[min_temp].y);
        double vector_dot = v1.dot(v2);
        if (vector_dot < 0)
        {

            if (std::abs(v3.norm()) > 0.01)
            {
                factor = (v1.dot(v3) / v3.norm()) / v3.norm();
            }
            else
            {
                factor = 0.0;
            }
            final_next = temp_before;
        }
        else
        {
            if (std::abs(v2.norm()) > 0.01)
            {
                factor = (vector_dot / v2.norm()) / v2.norm();
            }
            else
            {
                factor = 0.0;
            }
            final_next = temp_next;
        }
    }
    point = traj.traj_points[min_temp];

    // std::cout << "--x:" << point.x << ", y:" << point.y << std::endl;
    // std::cout << "--factor:" << factor << std::endl;

    // std::cout << "min_point_curvature: " << point.kappa << std::endl;
    point.x += (traj.traj_points[final_next].x - traj.traj_points[min_temp].x) * factor;
    point.y += (traj.traj_points[final_next].y - traj.traj_points[min_temp].y) * factor;
    point.s += (traj.traj_points[final_next].s - traj.traj_points[min_temp].s) * factor;
    point.l += (traj.traj_points[final_next].l - traj.traj_points[min_temp].l) * factor;
    point.kappa += (traj.traj_points[final_next].kappa - traj.traj_points[min_temp].kappa) * factor;
    /*
    std::cout << "min_temp: " << min_temp << std::endl;
    std::cout << "final_next: " << final_next << std::endl;
    std::cout << "traj.traj_points[final_next].kappa: " << traj.traj_points[final_next].kappa << std::endl;
    std::cout << "traj.traj_points[min_temp].kappa: " << traj.traj_points[min_temp].kappa << std::endl;
    std::cout << "factor: " << factor << std::endl;
    std::cout << "projection_point_curvature: " << point.kappa << std::endl;
    */
    point.theta += NormalizeAngle((traj.traj_points[final_next].theta - traj.traj_points[min_temp].theta)) * factor;
    point.theta = NormalizeAngle(point.theta);
    point.v += (traj.traj_points[final_next].v - traj.traj_points[min_temp].v) * factor;
    point.t += (traj.traj_points[final_next].t - traj.traj_points[min_temp].t) * factor;
    // point.forward = traj[min_temp].forward;
    // point.parking_point = traj[final_next].parking_point;

    // std::cout << "--final_next:" << final_next
    //           << ", x:" << traj.traj_points[final_next].x
    //           << ", y:" << traj.traj_points[final_next].y << std::endl;
    // std::cout << "--x:" << point.x
    //           << ", y:" << point.y << std::endl;
    return true;
}

void StaticObstacleSet::set_s_l(const Traj &traj)
{
    for (auto &static_obstacle : static_obstacle_set)
    {
        WayPoint waypoint;
        waypoint.x = static_obstacle.center_x;
        waypoint.y = static_obstacle.center_y;
        TrajPoint projection_point;
        if (calculateProjectionPointWithPose(traj, waypoint, projection_point))
        {
            static_obstacle.center_s = projection_point.s;

            double dx = projection_point.x - waypoint.x;
            double dy = projection_point.y - waypoint.y;

            const double cos_theta_r = std::cos(projection_point.theta);
            const double sin_theta_r = std::sin(projection_point.theta);

            const double cross_rd_nd = -1.0 * cos_theta_r * dy + sin_theta_r * dx;
            static_obstacle.center_l = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
        }
        else
        {
            static_obstacle.center_s = 0.0;
            static_obstacle.center_l = 0.0;
        }
    }
}