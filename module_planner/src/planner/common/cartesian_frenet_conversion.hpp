#ifndef CARTESIAN_FRENET_CONVERTER_HPP
#define CARTESIAN_FRENET_CONVERTER_HPP

#include <cmath>
#include <array>

#include "common.hpp"
#include "ultis_math.hpp"


class CartesianFrenetConverter
{
public:
    CartesianFrenetConverter() = default;

    // 根据(x,y)和投影点(rx,ry,rs,rtheta)，得到(s,l)
    void cartesian_to_frenet(const double x, const double y,
                             const double rx, const double ry,
                             const double rs, const double rtheta,
                             double *ptr_s, double *ptr_d);

    // 根据(x,y)和投影点TrajPoint，得到(s,l)
    void cartesian_to_frenet(const double x, const double y,
                             const TrajPoint trajpoint,
                             double *ptr_s, double *ptr_d);

    // 根据(x,y,v,a,theta,kappa)和投影点(rx,ry,rs,rtheta,rkappa,rdkappa)
    // 得到(s_condition,l_p_condition,l_d_condition)
    /*
    Notations:
        s_condition = [s, s_dot, s_ddot]
        s: longitudinal coordinate w.r.t reference line.
        s_dot: ds / dt
        s_ddot: d(s_dot) / dt

        l_p_condition = [l, l_prime, l_pprime]
        l: lateral coordinate w.r.t. reference line
        l_prime: dd / ds
        l_pprime: d(d_prime) / ds

        l_d_condition = [l, l_dot, l_ddot]
        l_dot: dl / dt
        l_ddot: d(l_dot) / dt
        d: the same as l.
        d: lateral coordinate w.r.t. reference line

        kappa: 该点的曲率
        dkappa： d(kappa)/ds
    */
    void cartesian_to_frenet(const double x, const double y,
                             const double v, const double a,
                             const double theta, const double kappa,
                             const double rx, const double ry,
                             const double rs, const double rtheta,
                             const double rkappa, const double rdkappa,
                             std::array<double, 3> *const ptr_s_condition,
                             std::array<double, 3> *const ptr_l_p_condition,
                             std::array<double, 3> *const ptr_l_d_condition);

    /////////////////////////
    /////////////////////////

    // 根据(s,l)和轨迹，得到(x,y)
    void frenet_to_cartesian(const double s, const double l,
                             const Traj traj,
                             double *ptr_x, double *ptr_y);

    // 根据s在轨迹上插值，得到投影点
    void frenet_to_cartesian(const double s,
                             const Traj traj,
                             TrajPoint &trajpoint);

    // 根据 l 和投影点(rx,ry,rtheta)，得到(x,y)
    void frenet_to_cartesian(const double l,
                             const double rx, const double ry, const double rtheta,
                             double *ptr_x, double *ptr_y);

    // 根据 l 和投影点TrajPoint，得到(x,y)
    void frenet_to_cartesian(const double l,
                             TrajPoint trajpoint,
                             double *ptr_x, double *ptr_y);
};

#endif // CARTESIAN_FRENET_CONVERTER_HPP