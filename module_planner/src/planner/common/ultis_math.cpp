#include "ultis_math.hpp"

double NormalizeAngle(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

double DeadLimit(double value, double limit) {
    if (std::fabs(value) < limit) {
        return 0;
    } else {
        return value;
    }
}