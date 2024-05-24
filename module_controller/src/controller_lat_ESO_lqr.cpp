#include "controller_lat_ESO_lqr.hpp"

#include <iostream>

lqr_ESO_control::lqr_ESO_control()
{
    control_period = 0.02;
    Ad = Eigen::MatrixXd::Zero(2, 2);
    Ad << 0.0, 1.0,
        0.0, 0.0;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(2, 2);
    Ad = I + Ad * control_period;
    Bd = Eigen::MatrixXd::Zero(2, 1);
    Bd << 0.0, 1.0;
    Bd *= control_period;

    ESO_State = Eigen::MatrixXd::Zero(3, 1);
    State = Eigen::MatrixXd::Zero(2, 1);
    Q = Eigen::MatrixXd::Identity(2, 2);
    Q << 0.5, 0.0,
        0.0, 2.0;
    R = Eigen::MatrixXd::Identity(1, 1);
    R << 1.0;

    curvature_max = 0.8;
}

void lqr_ESO_control::calculateControlCmd(EgoStatus EgoStatus_, PlannerTraj trajectory,
                                          ChassisCmd &ChassisCmd_)
{
    if (!update_Ego_State(EgoStatus_, trajectory))
    {
        // 如果没求出状态，还是给上一时刻曲率
        curvature_cmd = std::clamp(curvature_cmd, -curvature_max, curvature_max);
        if (curvature_cmd <= 0.0025 && curvature_cmd >= -0.0025)
        {
            ChassisCmd_.turn_radius_req = 400; // 直线
        }
        else
        {
            ChassisCmd_.turn_radius_req = 1.0 / curvature_cmd;
        }
        return;
    }
    update_control_State();
    // 求feedback_cmd
    // SolveLQRProblem(Ad, Bd, Q, R, lqr_eps, lqr_max_iteration, &K);
    // double feedback_cmd = -(K * State)(0, 0);
    // std::cout << "k1 = " << K(0, 0) << ", k2 = " << K(0, 1) << std::endl;
    // double x3 = (ego_velocity * ego_velocity * (curvature_cmd - projection_point_curvature) +
    //              ESO_State(2, 0));
    // feedback_cmd += -5.0 * x3;

    // double ki = 0.4;
    // double kp = 40 * std::fabs(ESO_State(0, 0));
    // double kd = 0.2;
    // double feedback_cmd = -ki * func_fal(ESO_State(0, 0), 0.5, 0.2) -
    //                       kp * func_fal(ESO_State(1, 0), 0.25, 0.2);
    // double e3 = (ego_velocity * ego_velocity * (curvature_cmd - projection_point_curvature) +
    //              ESO_State(2, 0));
    // feedback_cmd += -kd * func_fal(e3, 0.125, 0.2);

    double ki = 0.4;
    double kp = 5.0;
    double kd = 0.2;
    double feedback_cmd = -ki * ESO_State(0, 0) -
                          kp * ESO_State(1, 0);
    double e3 = (ego_velocity * ego_velocity * (curvature_cmd - projection_point_curvature) +
                 ESO_State(2, 0));
    feedback_cmd += -kd * e3;

    if (std::abs(ego_velocity * ego_velocity) > 0.5)
    {
        curvature_cmd = (feedback_cmd - all_disturbance) / (ego_velocity * ego_velocity) + projection_point_curvature;
    }
    else
    {
        curvature_cmd = 0.0;
    }

    curvature_cmd = std::clamp(curvature_cmd, -curvature_max, curvature_max);
    if (curvature_cmd <= 0.0025 && curvature_cmd >= -0.0025)
    {
        ChassisCmd_.turn_radius_req = 400; // 直线
    }
    else
    {
        ChassisCmd_.turn_radius_req = 1.0 / curvature_cmd;
    }
}

bool lqr_ESO_control::update_Ego_State(EgoStatus EgoStatus_, PlannerTraj PlannerTraj_)
{
    // 求匹配点
    TrajPoint projection_point;
    if (!calculateProjectionPointWithPose(PlannerTraj_, EgoStatus_, projection_point))
    {
        return false;
    }
    projection_point_curvature = projection_point.kappa;
    // 航向偏差
    ControlError_.yaw_err = NormalizeAngle(EgoStatus_.ego_theta - projection_point.theta);
    // 横向偏差
    double dx = EgoStatus_.ego_x - projection_point.x;
    double dy = EgoStatus_.ego_y - projection_point.y;
    double cos_projection_point_yaw = std::cos(projection_point.theta);
    double sin_projection_point_yaw = std::sin(projection_point.theta);
    ControlError_.lateral_err = dy * cos_projection_point_yaw - dx * sin_projection_point_yaw;

    ego_velocity = EgoStatus_.ego_velocity;
    ego_w = EgoStatus_.ego_w;

    return true;
}

void lqr_ESO_control::update_control_State()
{
    update_ESO_State();
    State(0, 0) = ESO_State(0, 0);
    // State(0, 0) = ControlError_.lateral_err;
    State(1, 0) = ESO_State(1, 0);
}
void lqr_ESO_control::update_ESO_State()
{
    double k1 = 100.0;
    double k2 = 50.0;
    double k3 = 10.0;
    for (int i = 0; i < n_period; i++)
    {
        double ESO_error = ESO_State(0, 0) - ControlError_.lateral_err;
        // std::cout << "ESO_State(0, 0): " << ESO_State(0, 0)
        //           << ", ControlError_.lateral_err: " << ControlError_.lateral_err
        //           << ", ESO_error: " << ESO_error << std::endl;

        double dot_ESO_State_0 = ESO_State(1, 0) - k1 * ESO_error;
        double dot_ESO_State_1 = ego_velocity * ego_velocity * (curvature_cmd - projection_point_curvature) +
                                 ESO_State(2, 0) - k2 * func_fal(ESO_error, 0.5, 0.01);
        double dot_ESO_State_2 = -k3 * func_fal(ESO_error, 0.25, 0.01);

        ESO_State(0, 0) += dot_ESO_State_0 * ESO_period;
        ESO_State(1, 0) += dot_ESO_State_1 * ESO_period;
        ESO_State(2, 0) += dot_ESO_State_2 * ESO_period;
    }
    all_disturbance = ESO_State(2, 0);

    std::cout << "ESO_error: " << ESO_State(0, 0) - ControlError_.lateral_err << std::endl;
}

void lqr_ESO_control::getControlError(ControlError &_ControlError_)
{
    _ControlError_.lateral_err = ControlError_.lateral_err;
    _ControlError_.yaw_err = ControlError_.yaw_err;
    _ControlError_.dot_yaw_err = ego_w - ego_velocity * projection_point_curvature;
}

Eigen::MatrixXd lqr_ESO_control::getEsoError()
{
    return ESO_State;
}
