#include "math/controller_base.hpp"
#include "math/trajectory_analysis.hpp"
#include "math/ultis_math.hpp"
#include "math/lqr_solver.hpp"
// #include <Eigen/Core>
#include "eigen3/Eigen/Core"
#include <algorithm> // 包含 std::clamp 函数所在的头文件

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

class lqr_control{
public:
    lqr_control(double dt_ = 0.02);

    // 更新控制指令ChassisCmd_中的turn_radius_req
    void calculateControlCmd(EgoStatus EgoStatus_, PlannerTraj PlannerTraj_, ChassisCmd &ChassisCmd_);

    // 获取控制误差
    void getControlError(ControlError &ControlError_);

    ~lqr_control() = default;
private:
    double kd = 0.0;
    void read_module_controller_ini();
    // 计算横向偏差，航向偏差，航向偏差变化率
    bool updateState(EgoStatus EgoStatus_, PlannerTraj PlannerTraj_);

    // 基于车速更新Ad矩阵
    void updateModleMatrix();

    //基于车速设置QR矩阵
    void loadQRmatrix();

    /*
        滤波随后考虑
    */

    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;
    Eigen::MatrixXd State;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd K;

    double dt;
    double projection_point_curvature;
    double lqr_eps;
    int lqr_max_iteration;
    double curvature_max;

    double ego_velocity; // 实车实际的速度
    double ego_w; // 实车实际的角速度
};

