#include "cartesian_frenet_conversion.hpp"

// 根据(x,y)和投影点(rx,ry,rs,rtheta)，得到(s,l)
void CartesianFrenetConverter::cartesian_to_frenet(const double x, const double y,
                                                   const double rx, const double ry,
                                                   const double rs, const double rtheta,
                                                   double *ptr_s, double *ptr_d)
{
    const double dx = x - rx;
    const double dy = y - ry;

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    *ptr_d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
    *ptr_s = rs;
}

// 根据(x,y)和投影点TrajPoint，得到(s,l)
void CartesianFrenetConverter::cartesian_to_frenet(const double x, const double y,
                                                   const TrajPoint trajpoint,
                                                   double *ptr_s, double *ptr_d)
{
    double rx = trajpoint.x;
    double ry = trajpoint.y;
    double rs = trajpoint.s;
    double rtheta = trajpoint.theta;
    cartesian_to_frenet(x, y, rx, ry, rs, rtheta, ptr_s, ptr_d);
}

// 根据(x,y,v,a,theta,kappa)和投影点(rx,ry,rs,rtheta,rkappa,rdkappa)
// 得到(s_condition,l_p_condition,l_d_condition)
void CartesianFrenetConverter::cartesian_to_frenet(const double x, const double y,
                                                   const double v, const double a,
                                                   const double theta, const double kappa,
                                                   const double rx, const double ry,
                                                   const double rs, const double rtheta,
                                                   const double rkappa, const double rdkappa,
                                                   std::array<double, 3> *const ptr_s_condition,
                                                   std::array<double, 3> *const ptr_l_p_condition,
                                                   std::array<double, 3> *const ptr_l_d_condition)
{
    const double dx = x - rx;
    const double dy = y - ry;

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    ptr_l_p_condition->at(0) =
        std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);

    const double delta_theta = theta - rtheta;
    const double tan_delta_theta = std::tan(delta_theta);
    const double cos_delta_theta = std::cos(delta_theta);

    const double one_minus_kappa_r_d = 1 - rkappa * ptr_l_p_condition->at(0);
    ptr_l_p_condition->at(1) = one_minus_kappa_r_d * tan_delta_theta;

    const double kappa_r_d_prime =
        rdkappa * ptr_l_p_condition->at(0) + rkappa * ptr_l_p_condition->at(1);

    ptr_l_p_condition->at(2) =
        -kappa_r_d_prime * tan_delta_theta +
        one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
            (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

    ptr_s_condition->at(0) = rs;

    ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;

    const double delta_theta_prime =
        one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
    ptr_s_condition->at(2) =
        (a * cos_delta_theta -
         ptr_s_condition->at(1) * ptr_s_condition->at(1) *
             (ptr_l_p_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) /
        one_minus_kappa_r_d;

    ptr_l_d_condition->at(0) = ptr_l_p_condition->at(0);
    ptr_l_d_condition->at(1) = ptr_l_p_condition->at(1) * ptr_s_condition->at(1);
    ptr_l_d_condition->at(2) = ptr_l_p_condition->at(2) *
                                   ptr_s_condition->at(1) * ptr_s_condition->at(1) +
                               ptr_l_p_condition->at(1) * ptr_s_condition->at(2);
}

///////////////////////////////////
///////////////////////////////////

// 根据s在轨迹上插值，得到投影点
void CartesianFrenetConverter::frenet_to_cartesian(const double s,
                                                   const Traj traj,
                                                   TrajPoint &trajpoint)
{
    int traj_size = traj.traj_points.size();
    double min_dist = 10e6;
    int min_temp = -1;
    for (int i = 0; i < traj_size; i++)
    {
        double dist = std::fabs(traj.traj_points[i].s - s); // 计算当前点与参数s的距离
        if (dist < min_dist)
        {
            min_dist = dist;
            min_temp = i;
        }
    }
    trajpoint = traj.traj_points[min_temp];

    int temp_next = min_temp;
    int temp_before = min_temp;
    if (min_temp > 0)
    {
        temp_before = min_temp - 1;
    }
    if (min_temp < traj_size - 1)
    {
        temp_next = min_temp + 1;
    }

    int final_next;
    double factor = 0.0;
    if (s >= traj.traj_points[min_temp].s)
    {
        factor = s - traj.traj_points[min_temp].s;
        final_next = temp_next;
    }
    else
    {
        factor = traj.traj_points[min_temp].s - s;
        final_next = temp_before;
    }

    trajpoint.x += (traj.traj_points[final_next].x - traj.traj_points[min_temp].x) * factor;
    trajpoint.y += (traj.traj_points[final_next].y - traj.traj_points[min_temp].y) * factor;
    trajpoint.s += (traj.traj_points[final_next].s - traj.traj_points[min_temp].s) * factor;
    trajpoint.l += (traj.traj_points[final_next].l - traj.traj_points[min_temp].l) * factor;
    trajpoint.kappa += (traj.traj_points[final_next].kappa - traj.traj_points[min_temp].kappa) * factor;
    trajpoint.theta += NormalizeAngle((traj.traj_points[final_next].theta - traj.traj_points[min_temp].theta)) * factor;
    trajpoint.theta = NormalizeAngle(trajpoint.theta);
    trajpoint.v += (traj.traj_points[final_next].v - traj.traj_points[min_temp].v) * factor;
    trajpoint.t += (traj.traj_points[final_next].t - traj.traj_points[min_temp].t) * factor;
}

// 根据 l 和投影点(rx,ry,rtheta)，得到(x,y)
void CartesianFrenetConverter::frenet_to_cartesian(const double l,
                                                   const double rx, const double ry, const double rtheta,
                                                   double *ptr_x, double *ptr_y)
{
    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    *ptr_x = rx - sin_theta_r * l;
    *ptr_y = ry + cos_theta_r * l;
}

// 根据 l 和投影点TrajPoint，得到(x,y)
void CartesianFrenetConverter::frenet_to_cartesian(const double l,
                                                   const TrajPoint trajpoint,
                                                   double *ptr_x, double *ptr_y)
{
    double rtheta = trajpoint.theta;
    double rx = trajpoint.x;
    double ry = trajpoint.y;

    frenet_to_cartesian(l, rx, ry, rtheta, ptr_x, ptr_y);
}

// 根据(s,l)和轨迹，得到(x,y)
void CartesianFrenetConverter::frenet_to_cartesian(const double s, const double l,
                                                   const Traj traj,
                                                   double *ptr_x, double *ptr_y)
{
    TrajPoint trajpoint;
    frenet_to_cartesian(s, traj, trajpoint); // 根据s插值找到匹配点trajpoint
    frenet_to_cartesian(l, trajpoint, ptr_x, ptr_y);
}