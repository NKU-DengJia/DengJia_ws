//#include <Eigen/Dense>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include <stdio.h>
#include <iostream>

void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                     const Eigen::MatrixXd &R,
                     const Eigen::MatrixXd &M, const double &tolerance, const uint max_num_iteration,
                     Eigen::MatrixXd *ptr_K);

void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                     const Eigen::MatrixXd &R, const double tolerance,
                     const uint max_num_iteration, Eigen::MatrixXd *ptr_K);
