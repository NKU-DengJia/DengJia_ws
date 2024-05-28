#include "traj_sampling.hpp"

#include <iostream>
#include <cstdlib>

TrajSampling::TrajSampling()
{
    read_module_planner_ini();
}
void TrajSampling::read_module_planner_ini()
{
    try
    {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini("/home/dengjia/DengJia_ws/src/module_planner/src/module_planner.ini", pt);
        cost_weight_1 = pt.get<double>("trajectory_sampling.cost_weight_1");
        cost_weight_2 = pt.get<double>("trajectory_sampling.cost_weight_2");
        cost_weight_3 = pt.get<double>("trajectory_sampling.cost_weight_3");
        cost_weight_4 = pt.get<double>("trajectory_sampling.cost_weight_4");
        cost_weight_5 = pt.get<double>("trajectory_sampling.cost_weight_5");
        lon_expect = pt.get<double>("trajectory_sampling.lon_expect");
        lon_ratio = pt.get<double>("trajectory_sampling.lon_ratio");
        lat_expect = pt.get<double>("trajectory_sampling.lat_expect");
        lat_ratio = pt.get<double>("trajectory_sampling.lon_ratio");
    }
    catch (const boost::property_tree::ini_parser_error &e)
    {
        // 读取失败，输出错误信息
        std::cerr << "Error reading module_planner.ini: " << e.what() << std::endl;
    }
}

Traj TrajSampling::obtain_result_traj(TrajPoint trajpoint,
                                      const StaticObstacleSet static_obstacles,
                                      const Traj referenceline)
{
    execution_time.start = std::chrono::steady_clock::now();

    Way result_way;
    Traj result;
    TrajPoint projectionpoint;
    WayPoint waypoint_temp;
    waypoint_temp.x = trajpoint.x;
    waypoint_temp.y = trajpoint.y;
    if (!calculateProjectionPointWithPose(referenceline, waypoint_temp, projectionpoint))
    {
        return result; // 返回空轨迹
    }
    // 给规划起点补上s和l信息
    CartesianFrenetConverter temp;
    temp.cartesian_to_frenet(trajpoint.x, trajpoint.y, projectionpoint, &trajpoint.s, &trajpoint.l);

    std::vector<double> lon_sample_point_set;
    std::vector<double> lat_sample_point_set;
    generate_lon_sample_point_set(projectionpoint.s, referenceline.traj_points.back().s, lon_sample_point_set);
    generate_lat_sample_point_set(lat_sample_point_set);
    std::vector<std::vector<std::vector<TrajPolynomial>>> all_sample_traj_set;
    // 生成轨迹集合似乎有些慢
    execution_time.start_generate_sample_traj_set = std::chrono::steady_clock::now();
    // std::cout << "------------------111111111111111111111111-----------" << std::endl;
    generate_sample_traj_set(trajpoint, projectionpoint, lon_sample_point_set, lat_sample_point_set, all_sample_traj_set);
    // std::cout << "------------------22222222222222-----------" << std::endl;
    execution_time.end_generate_sample_traj_set = std::chrono::steady_clock::now();

    // std::cout << " lon_sample_point_set.size():" << lon_sample_point_set.size() << std::endl;
    // std::cout << " lat_sample_point_set.size(): " << lat_sample_point_set.size() << std::endl;

    // 统一计算cost
    for (auto &traj_groups : all_sample_traj_set)
    {
        for (auto &traj_group : traj_groups)
            for (auto &traj_polynomial : traj_group)
            {
                {
                    calculate_traj_cost(static_obstacles, traj_polynomial);
                }
            }
    }

    // DP选路
    std::vector<TrajPolynomial> result_sample_traj;
    execution_time.start_select_traj_DP = std::chrono::steady_clock::now();
    select_traj_DP(all_sample_traj_set, result_sample_traj);
    execution_time.end_select_traj_DP = std::chrono::steady_clock::now();

    // std::cout << "result_sample_traj.size(): " << result_sample_traj.size() << std::endl;
    // std::cout << "result_size: " << result.traj_points.size() << std::endl;
    for (auto traj_polynomial : result_sample_traj)
    {
        // std::cout << "traj_polynomial.cost:  " << traj_polynomial.cost << std::endl;
        // std::cout << "traj_polynomial.s0:  " << traj_polynomial.s0 << std::endl;
        // std::cout << "traj_polynomial.send:  " << traj_polynomial.send << std::endl;
        // std::cout << "traj_polynomial.coefficientspolynomial:  "
        //           << traj_polynomial.coefficientspolynomial.a5
        //           << " " << traj_polynomial.coefficientspolynomial.a4
        //           << " " << traj_polynomial.coefficientspolynomial.a3
        //           << " " << traj_polynomial.coefficientspolynomial.a2
        //           << " " << traj_polynomial.coefficientspolynomial.a1
        //           << " " << traj_polynomial.coefficientspolynomial.a0 << std::endl;
        // std::cout << "----------------------------------------" << std::endl;
        Way way = PolynomialToWay(traj_polynomial, referenceline);
        result_way.way_points.insert(result_way.way_points.end(), way.way_points.begin(), way.way_points.end());
    }
    result = result_way.toTraj(0.0);

    execution_time.end = std::chrono::steady_clock::now();

    return result;
}

void TrajSampling::generate_lon_sample_point_set(const double s0, const double smax,
                                                 std::vector<double> &lon_sample_point_set)
{
    lon_sample_point_set.clear();
    double lon = s0;
    if (s0 >= smax)
    {
        return;
    }
    while (true)
    {
        lon += lon_ratio;
        if (lon > smax)
        {
            lon_sample_point_set.push_back(smax);
            break;
        }
        lon_sample_point_set.push_back(lon);
        if (lon >= s0 + lon_expect)
        {
            break;
        }
    }
}
void TrajSampling::generate_lat_sample_point_set(std::vector<double> &lat_sample_point_set)
{
    lat_sample_point_set.clear();
    double lat = 0;
    while (lat <= lat_expect)
    {
        lat_sample_point_set.push_back(lat);
        lat = lat + lat_ratio;
    }
    lat = -lat_ratio;
    while (lat >= -lat_expect)
    {
        lat_sample_point_set.push_back(lat);
        lat = lat - lat_ratio;
    }
}

void TrajSampling::generate_sample_traj_set(const TrajPoint trajpoint, const TrajPoint projectionpoint,
                                            const std::vector<double> lon_sample_point_set,
                                            const std::vector<double> lat_sample_point_set,
                                            std::vector<std::vector<std::vector<TrajPolynomial>>> &all_sample_traj_set)
{
    all_sample_traj_set.clear();
    // 预先分配足够的空间
    size_t lon_size = lon_sample_point_set.size();
    size_t lat_size = lat_sample_point_set.size();
    all_sample_traj_set.resize(lon_size, std::vector<std::vector<TrajPolynomial>>(lat_size, std::vector<TrajPolynomial>()));
    // 起点到第一组横向集合
    // 获取起点坐标
    double lon_start = trajpoint.s;
    double lat_start = trajpoint.l;
    double lat_start_prime = (1 - projectionpoint.kappa * trajpoint.l) * std::tan(trajpoint.theta - projectionpoint.theta);
    // double lat_start_prime = std::tan(trajpoint.theta - projectionpoint.theta);

    // std::cout << "规划起点:" << std::endl;
    // std::cout << "  lon_start:" << trajpoint.s << std::endl;
    // std::cout << "  lat_start:" << trajpoint.l << std::endl;
    // std::cout << "  lat_start_prime:" << lat_start_prime << std::endl;
    // std::cout << "++++++++++++++++++++++++" << std::endl;
    // std::cout << "采样集合size:" << std::endl;
    // std::cout << "  lon_size:" << lon_sample_point_set.size() << std::endl;
    // std::cout << "  lat_size:" << lat_sample_point_set.size() << std::endl;
    for (size_t k = 0; k < lat_size; k++)
    {
        // 获取终点坐标
        double lon_end = lon_sample_point_set[0];
        double lat_end = lat_sample_point_set[k];
        // 计算多项式系数
        CoefficientsPolynomial coefficients;
        solvePolynomialCoefficients(lon_start, lat_start, lat_start_prime, 0, lon_end, lat_end, 0, 0, coefficients);
        // 创建轨迹多项式对象
        TrajPolynomial traj_polynomial;
        traj_polynomial.s0 = lon_start;
        traj_polynomial.send = lon_end;
        traj_polynomial.coefficientspolynomial = coefficients;

        all_sample_traj_set[0][0].push_back(traj_polynomial);
        // all_sample_traj_set[i][j][k]表示：从“第i列第j行”到“第i+1列第k行”的轨迹; 起点唯一：是第0行第0列
    }

    // 遍历lon_sample_point_set和lat_sample_point_set，以每对点为起点生成五次多项式曲线
    for (size_t i = 0; i < lon_size - 1; i++)
    {
        for (size_t j = 0; j < lat_size; j++)
        {
            // 获取起点坐标
            double lon_start = lon_sample_point_set[i];
            double lat_start = lat_sample_point_set[j];

            for (size_t k = 0; k < lat_size; k++)
            {
                // 获取终点坐标
                double lon_end = lon_sample_point_set[i + 1];
                double lat_end = lat_sample_point_set[k];

                // 计算多项式系数
                CoefficientsPolynomial coefficients;
                solvePolynomialCoefficients(lon_start, lat_start, 0, 0, lon_end, lat_end, 0, 0, coefficients);

                // std::cout << "start(s,l): " << lon_start << ", " << lat_start << std::endl
                //           << "end(s,l): " << lon_end << ", " << lat_end << std::endl;
                // std::cout << "coefficients:  "
                //           << coefficients.a5
                //           << " " << coefficients.a4
                //           << " " << coefficients.a3
                //           << " " << coefficients.a2
                //           << " " << coefficients.a1
                //           << " " << coefficients.a0 << std::endl;
                // std::cout << "---------------------------" << std::endl;

                // 创建轨迹多项式对象
                TrajPolynomial traj_polynomial;
                traj_polynomial.s0 = lon_start;
                traj_polynomial.send = lon_end;
                traj_polynomial.coefficientspolynomial = coefficients;

                all_sample_traj_set[i + 1][j].push_back(traj_polynomial);
                // all_sample_traj_set[i][j][k]表示：从“第i列第j行”到“第i+1列第k行”的轨迹; 起点唯一：是第0行第0列
            }
        }
    }
}

void TrajSampling::solvePolynomialCoefficients(double x0, double y0,
                                               double dy0, double d2y0,
                                               double x1, double y1,
                                               double dy1, double d2y1,
                                               CoefficientsPolynomial &a)
{
    Eigen::Matrix<double, 6, 6> A;
    Eigen::VectorXd b(6);

    A << pow(x0, 5), pow(x0, 4), pow(x0, 3), pow(x0, 2), x0, 1,
        pow(x1, 5), pow(x1, 4), pow(x1, 3), pow(x1, 2), x1, 1,
        5 * pow(x0, 4), 4 * pow(x0, 3), 3 * pow(x0, 2), 2 * x0, 1, 0,
        5 * pow(x1, 4), 4 * pow(x1, 3), 3 * pow(x1, 2), 2 * x1, 1, 0,
        20 * pow(x0, 3), 12 * pow(x0, 2), 6 * x0, 2, 0, 0,
        20 * pow(x1, 3), 12 * pow(x1, 2), 6 * x1, 2, 0, 0;

    b << y0, y1, dy0, dy1, d2y0, d2y1;

    // Eigen::VectorXd coefficients = A.fullPivLu().solve(b);

    Eigen::Matrix<double, 6, 6> A_inv = A.inverse();
    Eigen::VectorXd coefficients = A_inv * b;

    a.a0 = coefficients(5);
    a.a1 = coefficients(4);
    a.a2 = coefficients(3);
    a.a3 = coefficients(2);
    a.a4 = coefficients(1);
    a.a5 = coefficients(0);
}

void TrajSampling::calculate_traj_cost(StaticObstacleSet static_obstacles,
                                       TrajPolynomial &trajpolynomial)
{
    // 初始化代价
    double cost1 = 0.0; // 平滑代价1
    double cost2 = 0.0; // 平滑代价2
    double cost3 = 0.0; // 平滑代价3
    double cost4 = 0.0; // 参考线距离代价
    double cost5 = 0.0; // 障碍物距离代价

    // 遍历计算代价
    double s0 = trajpolynomial.s0;
    double send = trajpolynomial.send;
    double sample_interval = 0.5;
    for (double s = s0; s <= send; s += sample_interval)
    {
        double l = PolynomialCalculateL(s, trajpolynomial.coefficientspolynomial);

        cost1 += pow(5 * trajpolynomial.coefficientspolynomial.a5 * pow(s, 4) +
                         4 * trajpolynomial.coefficientspolynomial.a4 * pow(s, 3) +
                         3 * trajpolynomial.coefficientspolynomial.a3 * pow(s, 2) +
                         2 * trajpolynomial.coefficientspolynomial.a2 * pow(s, 1) +
                         1 * trajpolynomial.coefficientspolynomial.a1,
                     2);

        cost2 += pow(20 * trajpolynomial.coefficientspolynomial.a5 * pow(s, 3) +
                         12 * trajpolynomial.coefficientspolynomial.a4 * pow(s, 2) +
                         6 * trajpolynomial.coefficientspolynomial.a3 * pow(s, 1) +
                         2 * trajpolynomial.coefficientspolynomial.a2,
                     2);

        cost3 += pow(60 * trajpolynomial.coefficientspolynomial.a5 * pow(s, 2) +
                         24 * trajpolynomial.coefficientspolynomial.a4 * pow(s, 1) +
                         6 * trajpolynomial.coefficientspolynomial.a3,
                     2);

        cost4 += l * l;

        if (!static_obstacles.static_obstacle_set.empty())
        {
            for (auto &static_obstacle : static_obstacles.static_obstacle_set)
            {
                if (static_obstacle.center_s < send + 20.0 && static_obstacle.center_s > s0 - 20.0)
                {
                    double dl = std::fabs(static_obstacle.center_l - l);
                    double ds = std::fabs(static_obstacle.center_s - s);
                    // std::cout << "dl:  " << dl << std::endl;
                    if ((dl * dl + ds * ds) <= 25)
                    {
                        cost5 += 99999;
                        // break;
                    }
                    else if ((dl * dl + ds * ds) > 400)
                    {
                        cost5 += 0;
                    }
                    else
                    {
                        cost5 += 1.0 / (dl * dl + ds * ds - 25);
                    }
                }
            }
        }
    }
    trajpolynomial.cost = cost_weight_1 * cost1 + cost_weight_2 * cost2 +
                          cost_weight_3 * cost3 + cost_weight_4 * cost4 +
                          cost_weight_5 * cost5;
}

void TrajSampling::select_traj_DP(std::vector<std::vector<std::vector<TrajPolynomial>>> all_sample_traj_set,
                                  std::vector<TrajPolynomial> &result_sample_traj)
{
    int num_i = all_sample_traj_set.size();
    if (num_i == 0)
        return;

    int max_j = 0;
    for (const auto &groups : all_sample_traj_set)
    {
        max_j = std::max(max_j, static_cast<int>(groups.size()));
    }

    int max_k = 0;
    for (const auto &groups : all_sample_traj_set)
    {
        for (const auto &group : groups)
        {
            max_k = std::max(max_k, static_cast<int>(group.size()));
        }
    }
    int num_j = std::max(max_j, max_k);

    std::vector<std::vector<double>> DP(num_i, std::vector<double>(num_j, std::numeric_limits<double>::max()));
    std::vector<std::vector<double>> DP_list(num_i, std::vector<double>(num_j, std::numeric_limits<double>::max()));

    for (int k = 0; k < max_k && k < static_cast<int>(all_sample_traj_set[0][0].size()); k++)
    {
        DP[0][k] = all_sample_traj_set[0][0][k].cost; // 从起点开始到第0+1列第k个点的最小代价
        DP_list[0][k] = 0;                            // 第0+1列第k个节点的父节点是起点(第0列第0个节点)
    }

    for (int i = 1; i < num_i; i++)
    {
        for (int j = 0; j < max_j && j < static_cast<int>(all_sample_traj_set[i].size()); j++)
        {
            for (int k = 0; k < max_k && k < static_cast<int>(all_sample_traj_set[i][j].size()); k++)
            {
                if (!all_sample_traj_set[i][j].empty())
                {
                    double temp = DP[i - 1][j] + all_sample_traj_set[i][j][k].cost;
                    if (temp < DP[i][k])
                    {
                        DP[i][k] = temp;   // 从起点开始到第i+1列第k个点的最小代价
                        DP_list[i][k] = j; // 第i+1列第k个节点的父节点是第i列第j个节点
                    }
                }
            }
        }
    }

    int end_min_cost_idx = 0;
    for (int k = 0; k < max_k; k++)
    {
        if (DP[num_i - 1][k] < DP[num_i - 1][end_min_cost_idx])
        {
            end_min_cost_idx = k;
        }
    }

    int i = DP_list.size() - 1; // 最后一列的索引
    int k = end_min_cost_idx;   // 最后一列的第k个点的索引
    // 从最后一个点(第i+1列第k个点)开始回溯到起点
    // std::cout << "num_i=: " << num_i << std::endl;
    // std::cout << "max_j=: " << max_j << std::endl;
    // std::cout << "max_k=: " << max_k << std::endl;
    // std::cout << "i=: " << i << std::endl;
    while (i >= 0)
    {
        // 获取当前节点的父节点索引
        int j = DP_list[i][k];
        // 根据父节点索引获取对应的轨迹
        if (j >= 0 && j < static_cast<int>(all_sample_traj_set[i].size()) && k >= 0 && k < static_cast<int>(all_sample_traj_set[i][j].size()))
        {
            // 插入轨迹到结果集中
            result_sample_traj.insert(result_sample_traj.begin(), all_sample_traj_set[i][j][k]);
        }
        // 更新 i 和 k
        --i;
        k = j;
    }

    if (DP[num_i - 1][end_min_cost_idx] >= 100 * 99999)
    {
        std::cout << "无安全路径！采样失败！！！！！！: " << DP[num_i - 1][end_min_cost_idx] << std::endl;
        // exit(EXIT_FAILURE);
    }
}

// 把单独的一条五次多项式轨迹 转成 Way
Way TrajSampling::PolynomialToWay(const TrajPolynomial trajpolynomial,
                                  const Traj referenceline)
{
    // 获取起点和终点
    double s0 = trajpolynomial.s0;
    double send = trajpolynomial.send;
    // std::cout << "trajpolynomial.s0: " << s0 << std::endl;
    // std::cout << "trajpolynomial.send: " << send << std::endl;

    // 采样间隔
    double sample_interval = 0.5;
    // 创建新路
    Way way_;
    // 遍历采样点
    for (double s = s0; s <= send - 0.5; s += sample_interval)
    {
        double l = PolynomialCalculateL(s, trajpolynomial.coefficientspolynomial);
        // 创建路点
        WayPoint point;
        CartesianFrenetConverter CartesianFrenetConverter_;
        CartesianFrenetConverter_.frenet_to_cartesian(s, l, referenceline, &point.x, &point.y);
        way_.way_points.push_back(point);
    }
    return way_;
}

double TrajSampling::PolynomialCalculateL(double s, const CoefficientsPolynomial &coefficients)
{
    double polynomial_value = coefficients.a5 * pow(s, 5) +
                              coefficients.a4 * pow(s, 4) +
                              coefficients.a3 * pow(s, 3) +
                              coefficients.a2 * pow(s, 2) +
                              coefficients.a1 * s +
                              coefficients.a0;
    return polynomial_value;
}

void TrajSampling::get_cout_execution_time()
{
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(execution_time.end - execution_time.start); // 计算执行时间
    double time_seconds1 = duration1.count() / 1000.0;                                                                 // 将时间转换为秒
    std::cout << "总执行时间: " << time_seconds1 << " seconds" << std::endl;                                           // 打印执行时间

    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(execution_time.end_generate_sample_traj_set - execution_time.start_generate_sample_traj_set); // 计算执行时间
    double time_seconds2 = duration2.count() / 1000.0;                                                                                                                   // 将时间转换为秒
    std::cout << "     生成所有轨迹所用时间: " << time_seconds2 << " seconds" << std::endl;                                                                              // 打印执行时间

    auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(execution_time.end_select_traj_DP - execution_time.start_select_traj_DP); // 计算执行时间
    double time_seconds3 = duration3.count() / 1000.0;                                                                                               // 将时间转换为秒
    std::cout << "     DP耗时: " << time_seconds3 << " seconds" << std::endl;                                                                        // 打印执行时间
}