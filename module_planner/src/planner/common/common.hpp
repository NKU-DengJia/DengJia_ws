#ifndef WAY_PATH_TRAJ_HPP
#define WAY_PATH_TRAJ_HPP

#include <vector>
#include <cmath> // for atan2
#include "eigen3/Eigen/Core"
#include <algorithm>
#include "ultis_math.hpp"

class WayPoint
{
public:
    double x;
    double y;
    WayPoint(double x_val = 0.0, double y_val = 0.0) : x(x_val), y(y_val) {}
};

class PathPoint : public WayPoint
{
public:
    double s;
    double l;
    double kappa;
    double dkappa; //
    double theta;
    PathPoint(double x_val = 0.0, double y_val = 0.0, double s_val = 0.0, double l_val = 0.0, double k_val = 0.0, double dk_val = 0.0, double theta_val = 0.0)
        : WayPoint(x_val, y_val), s(s_val), l(l_val), kappa(k_val), dkappa(dk_val), theta(theta_val) {}
};

class TrajPoint : public PathPoint
{
public:
    double v;
    double t;
    TrajPoint(double x_val = 0.0, double y_val = 0.0, double s_val = 0.0, double l_val = 0.0, double k_val = 0.0, double dk_val = 0.0, double theta_val = 0.0, double v_val = 0.0, double t_val = 0.0)
        : PathPoint(x_val, y_val, s_val, l_val, k_val, dk_val, theta_val), v(v_val), t(t_val) {}
};

class Path;
class Traj; // 前向声明

class Way
{
public:
    std::vector<WayPoint> way_points;
    Path toPath(double starting_s = 0.0) const;
    Traj toTraj(double starting_s = 0.0) const;
};

class Path
{
public:
    std::vector<PathPoint> path_points;
    Way toWay() const;
    Traj toTraj() const;
};

class Traj
{
public:
    std::vector<TrajPoint> traj_points;
    Way toWay() const;
    Path toPath() const;
};



bool calculateProjectionPointWithPose(const Traj traj, const WayPoint waypoint,
                                 TrajPoint &point);


struct StaticObstacle
{
    double center_x = 0.0;
    double center_y = 0.0;
    double center_s = 0.0;
    double center_l = 0.0;
    // TODO: 后续补充其余属性
    StaticObstacle() : center_x(0.0), center_y(0.0), center_s(0.0), center_l(0.0) {}
};
struct StaticObstacleSet
{
    std::vector<StaticObstacle> static_obstacle_set;
    void set_s_l(const Traj& traj); // 传入参考线，补充出static_obstacle_set集合内所有障碍物的s和l
};


struct EgoStatus {
    double ego_x;
    double ego_y;
    double ego_theta;
    double ego_velocity;
    double ego_w;  // 自车实际的角速度
    double ego_curvature;
    EgoStatus() : ego_x(0.0), ego_y(0.0), ego_theta(0.0), ego_velocity(0.0), ego_w(0.0), ego_curvature(0.0) {};
};

#endif // WAY_PATH_TRAJ_HPP
