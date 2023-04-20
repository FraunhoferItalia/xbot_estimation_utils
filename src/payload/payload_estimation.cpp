#include <estimation_utils/payload/payload_estimation.h>
#include <XBotInterface/Utils.h>

using namespace estimation_utils;


PayloadEstimation::PayloadEstimation(XBot::ModelInterface::ConstPtr model,
                                     std::string payload_link,
                                     double torque_noise_cov,
                                     double mass_noise_cov,
                                     double com_noise_cov):
    _model(model),
    _kalman(4),
    _payload_link(payload_link)
{
    _R = Eigen::VectorXd::Constant(model->getJointNum(),
                                   torque_noise_cov).asDiagonal();

    _Q = Eigen::Vector4d(
                mass_noise_cov,
                mass_noise_cov*com_noise_cov,
                mass_noise_cov*com_noise_cov,
                mass_noise_cov*com_noise_cov).asDiagonal();

    _A.setIdentity(4, 4);

    if(!model->getJacobian(payload_link, _J))
    {
        throw std::invalid_argument("invalid payload link: " + payload_link);
    }

    _Y.setZero(6, 4);

    _zero4d.setZero(4);

    Eigen::MatrixXd P0;
    P0.setIdentity(4, 4);

    if(!_kalman.initialize(_zero4d, P0))
    {
        throw std::runtime_error("kalman init failed");
    }

}

bool PayloadEstimation::compute(Eigen::VectorXd &payload_torque,
                                Eigen::Vector4d &payload_params)
{
    // gravity
    Eigen::Vector3d g(0, 0, -9.81);

    // skew(g)
    Eigen::Matrix3d Sg = XBot::Utils::skewSymmetricMatrix(g);

    // payload orientation w.r.t. world
    Eigen::Matrix3d R;
    _model->getOrientation(_payload_link, R);

    // payload jacobian
    _model->getJacobian(_payload_link, _J);

    // payload param -> wrench matrix
    // F = Y*x
    _Y.topLeftCorner<3, 1>() = g;
    _Y.bottomRightCorner<3, 3>() = -Sg*R;

    // form measurment equation
    // C*x = J'Y*x = g(q) - tau
    _model->computeGravityCompensation(_grav);
    _model->getJointEffort(_tau);
    _C.noalias() = _J.transpose()*_Y;
    _r = _grav - _tau;

    // kalman
    _kalman.predict(_A, _zero4d, _Q);
    payload_params = _kalman.correct(_r, _C, _R);
    payload_torque.noalias() = -_C*payload_params;

    return true;
}

Eigen::Matrix4d PayloadEstimation::getCovariance() const
{
    return _kalman.P();
}
