#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"

class self_OSQP
{
public:
    self_OSQP(int numberOfVariables, int numberOfConstraints)
    {
        hessian.resize(numberOfVariables, numberOfConstraints);
        hessian.setZero();

        gradient.resize(numberOfVariables);
        gradient.setZero();

        linearMatrix.resize(numberOfConstraints, numberOfVariables);
        linearMatrix.setZero();

        lowerBound = Eigen::VectorXd::Constant(numberOfConstraints, -OsqpEigen::INFTY);
        upperBound = Eigen::VectorXd::Constant(numberOfConstraints, OsqpEigen::INFTY);

        solver.settings()->setWarmStart(true); // WarmStart是一种技术，用于在连续的优化问题之间重用先前的解.
        solver.data()->setNumberOfVariables(numberOfVariables);
        solver.data()->setNumberOfConstraints(numberOfConstraints);
    }

private:
    int numberOfVariables = 0;
    int numberOfConstraints = 0;

    Eigen::SparseMatrix<double> hessian; // H矩阵
    Eigen::VectorXd gradient;            // f矩阵

    // 约束
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // 最优解
    Eigen::VectorXd QPSolution;

    // 求解器
    OsqpEigen::Solver solver;
};