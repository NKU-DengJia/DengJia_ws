#ifndef CONTROLLER_BASE_HPP
#define CONTROLLER_BASE_HPP

#include <vector>

struct ChassisCmd {
    double vel_req;
    double turn_radius_req;
    bool SteerModReq;  // 0中心转向，1正常行驶
    double CenterSteerReq; // 中心转向请求，-180 deg ~ 180 deg，目标航向角
    ChassisCmd() : vel_req(0.0), turn_radius_req(1000.0), SteerModReq(1), CenterSteerReq(0.0) {};
};

struct ControlError {
    double lateral_err; 
    double yaw_err;  
    double dot_yaw_err;  
    double vel_err;  // 纵向速度偏差
    ControlError() : lateral_err(0.0), yaw_err(0.0), dot_yaw_err(0.0), vel_err(0.0) {};
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

struct TrajPoint {
    double x;
    double y;
    double s;
    double l;
    double kappa;
    double dkappa;
    double theta;
    double v;
    double t;
    TrajPoint(double x_val = 0.0, double y_val = 0.0, double s_val = 0.0, double l_val = 0.0, double k_val = 0.0, double dk_val = 0.0, double theta_val = 0.0, double v_val = 0.0, double t_val = 0.0)
        : x(x_val), y(y_val), s(s_val), l(l_val), kappa(k_val), dkappa(dk_val), theta(theta_val), v(v_val), t(t_val) {};
};

class PlannerTraj {
public:
    std::vector<TrajPoint> traj_points;
};

#endif // CONTROLLER_BASE_HPP