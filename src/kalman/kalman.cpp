#include <estimation_utils/kalman/kalman.h>

using namespace estimation_utils;

KalmanFilter::KalmanFilter(int _nx):
    nx(_nx)
{
    _I.setIdentity(nx, nx);
}

bool KalmanFilter::initialize(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0)
{
    if(_xhat.size() != nx)
    {
        return false;
    }

    if(P0.rows() != nx || P0.cols() != nx)
    {
        return false;
    }

    _xhat = x0;
    _P = P0;

    return true;
}

const Eigen::VectorXd &KalmanFilter::predict(const Eigen::MatrixXd &A,
                                             const Eigen::VectorXd &bias, const Eigen::MatrixXd &Q)
{
    _xtmp.noalias() = A * _xhat + bias;
    _xhat = _xtmp;

    _Ptmp.noalias() = A*_P;
    _P = _Ptmp;
    _Ptmp.noalias() = _P*A.transpose() + Q;
    _P = _Ptmp;

    return _xhat;
}

const Eigen::VectorXd &KalmanFilter::correct(const Eigen::VectorXd &y, const Eigen::MatrixXd &C, const Eigen::MatrixXd &R)
{
    // compute innovation covariance S
    _Stmp.noalias() = C*_P;
    _S.noalias() = _Stmp*C.transpose() + R;

    // cholesky of S
    _llt.compute(_S);

    // solve S*K = C*P -> K = P*C'*Sinv
    _K.noalias() = C*_P;
    _llt.solveInPlace(_K);
    _K.transposeInPlace();

    // innovation
    _e.noalias() = y - C*_xhat;

    // correction
    _xhat.noalias() += _K * _e;
    _Ptmp = (_I - _K*C)*_P;
    _P = 0.5*(_Ptmp + _Ptmp.transpose());

    return _xhat;
}

const Eigen::MatrixXd &KalmanFilter::P() const
{
    return _P;
}

const Eigen::VectorXd &KalmanFilter::xhat() const
{
    return _xhat;
}
