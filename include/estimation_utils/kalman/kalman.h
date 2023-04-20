#ifndef ESTIMATION_UTILS_KALMAN_H
#define ESTIMATION_UTILS_KALMAN_H

#include <Eigen/Dense>

namespace estimation_utils
{

class KalmanFilter
{

public:

    KalmanFilter(int nx);

    const int nx;

    bool initialize(const Eigen::VectorXd& x0,
                    const Eigen::MatrixXd& P0);

    // x' = A*x + bias + N(0, Q)
    const Eigen::VectorXd& predict(const Eigen::MatrixXd& A,
                                   const Eigen::VectorXd& bias,
                                   const Eigen::MatrixXd& Q);

    // y = C*x + N(0, R)
    const Eigen::VectorXd& correct(const Eigen::VectorXd& y,
                                   const Eigen::MatrixXd& C,
                                   const Eigen::MatrixXd& R);

    const Eigen::MatrixXd& P() const;

    const Eigen::VectorXd& xhat() const;

private:

    Eigen::VectorXd _xhat, _e;
    Eigen::MatrixXd _P, _S, _K, _I;

    Eigen::VectorXd _xtmp;
    Eigen::MatrixXd _Ptmp, _Ktmp, _Stmp;

    Eigen::LLT<Eigen::MatrixXd> _llt;

};

}

#endif // KALMAN_H
