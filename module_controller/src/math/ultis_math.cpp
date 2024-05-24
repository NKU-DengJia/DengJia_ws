#include "ultis_math.hpp"

double NormalizeAngle(const double angle)
{
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0)
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

double DeadLimit(double value, double limit)
{
    if (std::fabs(value) < limit)
    {
        return 0;
    }
    else
    {
        return value;
    }
}

double Sign(double a)
{
    if (a > 0)
        return 1;
    if (a < 0)
        return -1;
    return 0;
}

double func_fal(double e, double h, double delta)
{
    if (std::fabs(e) > delta)
    {
        return std::pow(std::fabs(e), 0.5) * Sign(e);
        // return std::pow(std::fabs(e), h) * Sign(e);
    }
    else
    {
        return e / std::pow(delta, h);
        // return e / std::pow(delta, 1.0 - h);
    }
}