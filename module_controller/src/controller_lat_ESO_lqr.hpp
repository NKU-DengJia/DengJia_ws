#ifndef ESO_LQR_CONTROL_H
#define ESO_LQR_CONTROL_H

#include "math/controller_base.hpp"
#include "math/trajectory_analysis.hpp"
#include "math/ultis_math.hpp"
#include "math/lqr_solver.hpp"
// #include <Eigen/Core>
#include "eigen3/Eigen/Core"
#include <algorithm> // 包含 std::clamp 函数所在的头文件

class lqr_ESO_control
{
public:
    lqr_ESO_control();

    void calculateControlCmd(EgoStatus EgoStatus_, PlannerTraj trajectory,
                             ChassisCmd &ChassisCmd_);

    void getControlError(ControlError &ControlError_);

    Eigen::MatrixXd getEsoError();

    ~lqr_ESO_control() = default;

private:
    bool update_Ego_State(EgoStatus EgoStatus_, PlannerTraj PlannerTraj_);

    void update_ESO_State();

    void update_control_State();

private:
    double control_period = 0.02; // 控制周期
    int n_period = 20;
    double ESO_period = control_period / n_period; // ESO周期

    double all_disturbance = 0.0; // 总扰动

    // 状态空间方程
    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;
    Eigen::MatrixXd State;
    Eigen::MatrixXd ESO_State;
    // lqr中的
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd K; // 反馈增益矩阵

    double lqr_eps = 0.01;       // 求解DARE中的容忍度
    int lqr_max_iteration = 100; // 求解DARE中的最大迭代次数

    double ego_velocity = 0.0;
    double ego_w = 0.0;

    double projection_point_curvature = 0.0;

    ControlError ControlError_;

    double curvature_cmd = 400.0;
    double curvature_max = 0.8;
};

#endif // ESO_LQR_CONTROL_H