#include <iostream>
#include <vector>
#include <chrono>
#include "eigen3/Eigen/Dense"
#include "common.hpp"
#include "cartesian_frenet_conversion.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

struct CoefficientsPolynomial
{
    double a0 = 0.0;
    double a1 = 0.0;
    double a2 = 0.0;
    double a3 = 0.0;
    double a4 = 0.0;
    double a5 = 0.0;
};

struct TrajPolynomial
{
    double s0 = 0.0;
    double send = 0.0;
    CoefficientsPolynomial coefficientspolynomial{};
    double cost = 0.0;
};

struct ExecutionTime
{
    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point start_generate_sample_traj_set;
    std::chrono::steady_clock::time_point end_generate_sample_traj_set;
    std::chrono::steady_clock::time_point start_select_traj_DP;
    std::chrono::steady_clock::time_point end_select_traj_DP;
    std::chrono::steady_clock::time_point end;
};

class TrajSampling
{
public:
    TrajSampling();

    /*
    输入：
        规划起点：const TrajPoint trajpoint;
        静态障碍物集合：const StaticObstacleSet static_obstacles;
        参考线：const Traj referenceline;
    输出：
        采样+DP选出的最终路径：Traj xxxx;
    */
    Traj obtain_result_traj(const TrajPoint trajpoint,
                            const StaticObstacleSet static_obstacles,
                            const Traj referenceline);

    void get_cout_execution_time();

private:
    void read_module_planner_ini();

    void generate_lon_sample_point_set(const double s0, const double smax,
                                       std::vector<double> &lon_sample_point_set);

    void generate_lat_sample_point_set(std::vector<double> &lat_sample_point_set);

    void generate_sample_traj_set(const TrajPoint trajpoint, const TrajPoint projectionpoint,
                                  const std::vector<double> lon_sample_point_set,
                                  const std::vector<double> lat_sample_point_set,
                                  std::vector<std::vector<std::vector<TrajPolynomial>>> &all_sample_traj_set);

    void solvePolynomialCoefficients(double x0, double y0,
                                     double dy0, double d2y0,
                                     double x1, double y1,
                                     double dy1, double d2y1,
                                     CoefficientsPolynomial &a);

    void calculate_traj_cost(const StaticObstacleSet static_obstacles,
                             TrajPolynomial &trajpolynomial);

    void select_traj_DP(std::vector<std::vector<std::vector<TrajPolynomial>>> all_sample_traj_set,
                        std::vector<TrajPolynomial> &result_sample_traj);

    Way PolynomialToWay(const TrajPolynomial trajpolynomial,
                        const Traj referenceline);

    double PolynomialCalculateL(double s, const CoefficientsPolynomial &coefficients);

    double lon_ratio = 10.0;
    double lat_ratio = 3.0;

    double lon_expect = 100.0;
    double lat_expect = 10.0;

    double cost_weight_1 = 0.0;
    double cost_weight_2 = 0.0;
    double cost_weight_3 = 0.0;
    double cost_weight_4 = 0.0;
    double cost_weight_5 = 0.0;

    ExecutionTime execution_time{
        std::chrono::steady_clock::time_point(), // start
        std::chrono::steady_clock::time_point(), // start_generate_sample_traj_set
        std::chrono::steady_clock::time_point(), // end_generate_sample_traj_set
        std::chrono::steady_clock::time_point(), // start_select_traj_DP
        std::chrono::steady_clock::time_point(), // end_select_traj_DP
        std::chrono::steady_clock::time_point()  // end
    };
};
