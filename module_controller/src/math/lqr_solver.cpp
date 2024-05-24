#include "lqr_solver.hpp"

void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                     const Eigen::MatrixXd &R,
                     const Eigen::MatrixXd &M, const double &tolerance, const uint max_num_iteration,
                     Eigen::MatrixXd *ptr_K) {
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
        Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols() ||
        M.rows() != Q.rows() || M.cols() != R.cols()) {
        printf("LQR solver: one or more matrices have incompatible dimensions.\n");
        return;
    }
    Eigen::MatrixXd AT = A.transpose();
    Eigen::MatrixXd BT = B.transpose();
    Eigen::MatrixXd MT = M.transpose();
    Eigen::MatrixXd P = Q;
    uint num_iteration = 0;
    double diff = std::numeric_limits<double>::max();
    while (num_iteration++ < max_num_iteration && diff > tolerance) {
        Eigen::MatrixXd P_next =
                AT * P * A -
                (AT * P * B + M) * (R + BT * P * B).inverse() * (BT * P * A + MT) + Q;
        diff = std::fabs((P_next - P).maxCoeff());
        P = P_next;
    }
    if (num_iteration >= max_num_iteration) {

    }
    *ptr_K = (R + BT * P * B).inverse() * (BT * P * A + MT);
}

void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                     const Eigen::MatrixXd &R, const double tolerance,
                     const uint max_num_iteration, Eigen::MatrixXd *ptr_K) {

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(Q.rows(), R.cols());
    SolveLQRProblem(A, B, Q, R, M, tolerance, max_num_iteration, ptr_K);
}