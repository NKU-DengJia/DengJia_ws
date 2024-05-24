/*
不太行，要换
*/

#include <cmath>
#include "common.hpp"
#include "cartesian_frenet_conversion.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

class PIDController
{
public:
    PIDController(double kp, double ki, double kd, double integral_limit = std::numeric_limits<double>::max())
        : kp_(kp), ki_(ki), kd_(kd), integral_limit_(integral_limit), integral_(0), previous_error_(0), first_time_(true), output(0) {}

    double PIDcalculate(double xref, double x)
    {
        double e = xref - x;
        if (e > 100)
        {
            reset();
        }
        if (!first_time_)
        {
            integral_ += e;
            if (integral_limit_ != std::numeric_limits<double>::max())
            {
                integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
            }
        }
        double derivative;
        if (!first_time_)
        {
            derivative = e - previous_error_;
        }
        else
        {
            derivative = 0; // 第一次计算时，将微分置零
            first_time_ = false;
        }
        output = kp_ * e + ki_ * integral_ + kd_ * derivative;

        previous_error_ = e;
        // std::cout << "err = " << e << std::endl;
        // std::cout << std::endl;
        return output;
    }

    void reset()
    {
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
    double previous_error_ = 0.0;
    bool first_time_ = true;
    double output = 0.0;
};

class TrajStanley
{
public:
    TrajStanley(const Traj &traj_);

    Traj TrajStanleySmooth(); // 返回平滑后的轨迹

    ~TrajStanley() = default;

private:
    void read_module_planner_ini();

    void CalculateSmoothTraj();

    Traj traj0;       // 待平滑的轨迹
    Traj traj_result; // 平滑后的轨迹

    double ego_x = 0.0;
    double ego_y = 0.0;
    double ego_yaw = 0.0;
    double vel = 0.0;

    double T = 0.01; // 虚拟车更新频率

    double foresight_s = 0.0; // 前视s

    double kp = 0.15;
    double ki = 0.01;
    double kd = 0.0001;
    PIDController pid_w = PIDController(kp, ki, kd);

    CartesianFrenetConverter CartesianFrenetConverter_method;
};