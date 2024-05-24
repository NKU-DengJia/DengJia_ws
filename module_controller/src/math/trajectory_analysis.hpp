// #include <Eigen/Core>
#include "eigen3/Eigen/Core"
#include <algorithm>
#include "controller_base.hpp"
#include "ultis_math.hpp"
#include <iostream>

bool calculateProjectionPointWithPose(const PlannerTraj traj, const EgoStatus egostatus,
                                 TrajPoint &point);
