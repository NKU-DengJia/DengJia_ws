#include "controller_lat_lqr.hpp"

lqr_control::lqr_control(double dt_) : dt(dt_), projection_point_curvature(0.0),
                                       lqr_eps(0.01), lqr_max_iteration(100), curvature_max(0.8),
                                       ego_velocity(0.0), ego_w(0.0)
{
    read_module_controller_ini();
    int dim_u = 1;
    int dim_x = 2;
    const int matrix_size = dim_x;
    Ad = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Bd = Eigen::MatrixXd::Zero(matrix_size, dim_u);
    State = Eigen::MatrixXd::Zero(matrix_size, 1);
    ;
    Q = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    R = Eigen::MatrixXd::Identity(dim_u, dim_u);
    K = Eigen::MatrixXd::Zero(dim_u, matrix_size);
}
void lqr_control::read_module_controller_ini()
{
    try
    {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini("/home/dengjia/DengJia_ws/src/module_controller/src/module_controller.ini", pt);
        kd = pt.get<double>("lat_lqr.kd");
    }
    catch (const boost::property_tree::ini_parser_error &e)
    {
        // 读取失败，输出错误信息
        std::cerr << "Error reading module_controller.ini: " << e.what() << std::endl;
    }
}

void lqr_control::calculateControlCmd(EgoStatus EgoStatus_, PlannerTraj PlannerTraj_, ChassisCmd &ChassisCmd_)
{
    if (!updateState(EgoStatus_, PlannerTraj_))
    {
        ChassisCmd_.turn_radius_req = 400.0;
        return;
    }
    updateModleMatrix();
    loadQRmatrix();
    SolveLQRProblem(Ad, Bd, Q, R, lqr_eps, lqr_max_iteration, &K);
    double feedback_cmd = -(K * State)(0, 0);
    feedback_cmd += -1.0 * kd * (ego_w - ego_velocity * projection_point_curvature);

    // std::cout << "k1 = " << K(0, 0) << ", k2 = " << K(0, 1) << std::endl;

    double curvature_cmd;
    curvature_cmd = feedback_cmd / EgoStatus_.ego_velocity + projection_point_curvature;
    // std::cout << "feedback_cmd: " << feedback_cmd << std::endl;
    curvature_cmd = std::clamp(curvature_cmd, -curvature_max, curvature_max);
    if (curvature_cmd <= 0.0025 && curvature_cmd >= -0.0025)
    {
        ChassisCmd_.turn_radius_req = 400; // 直线
    }
    else
    {
        ChassisCmd_.turn_radius_req = 1.0 / curvature_cmd;
    }
    // std::cout << "ChassisCmd_.turn_radius_req: " << ChassisCmd_.turn_radius_req << std::endl;
}

bool lqr_control::updateState(EgoStatus EgoStatus_, PlannerTraj PlannerTraj_)
{
    TrajPoint projection_point;
    if (!calculateProjectionPointWithPose(PlannerTraj_, EgoStatus_, projection_point))
    {
        return false;
    }
    projection_point_curvature = projection_point.kappa;
    // 航向偏差
    double yaw_err = NormalizeAngle(EgoStatus_.ego_theta - projection_point.theta);
    // 横向偏差
    double dx = EgoStatus_.ego_x - projection_point.x;
    double dy = EgoStatus_.ego_y - projection_point.y;
    double cos_projection_point_yaw = std::cos(projection_point.theta);
    double sin_projection_point_yaw = std::sin(projection_point.theta);
    double lateral_err = dy * cos_projection_point_yaw - dx * sin_projection_point_yaw;

    State(0, 0) = lateral_err;
    State(1, 0) = yaw_err;
    ego_velocity = EgoStatus_.ego_velocity;
    ego_w = EgoStatus_.ego_w;

    return true;
}

// 基于车速更新Ad矩阵
void lqr_control::updateModleMatrix()
{
    Ad << 0.0, ego_velocity,
        0.0, 0.0;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(2, 2);
    Ad = I + Ad * dt;

    Bd << 0.0, 1.0;
    Bd *= dt;
}

// 基于车速设置QR矩阵
void lqr_control::loadQRmatrix()
{
    Q << 1.0, 0.0,
        0.0, 0.5;
    R << 1;
}

// 获取横向控制误差
void lqr_control::getControlError(ControlError &ControlError_)
{
    ControlError_.lateral_err = State(0, 0);
    ControlError_.yaw_err = State(1, 0);
    ControlError_.dot_yaw_err = ego_w - ego_velocity * projection_point_curvature;
}
