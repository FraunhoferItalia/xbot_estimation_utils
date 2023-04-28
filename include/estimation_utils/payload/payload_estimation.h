#ifndef ESTIMATION_UTILS_PAYLOAD_ESTIMATION_H
#define ESTIMATION_UTILS_PAYLOAD_ESTIMATION_H

#include <Eigen/Dense>
#include <XBotInterface/ModelInterface.h>

#include "../kalman/kalman.h"

namespace estimation_utils
{

class PayloadEstimation
{

public:

    PayloadEstimation(XBot::ModelInterface::ConstPtr model,
                      std::string payload_link,
                      double torque_noise_cov,
                      double mass_noise_cov,
                      double com_noise_cov);

    bool initialize(const Eigen::Vector4d& x0,
                    double mass_cov, double com_cov);

    bool compute(const Eigen::VectorXd& residual,
                 Eigen::VectorXd& payload_torque,
                 Eigen::Vector4d& payload_params);

    bool compute_static(Eigen::VectorXd& payload_torque,
                        Eigen::Vector4d& payload_params);

    void compute_payload_torque(const Eigen::Vector4d& payload_params, 
                                Eigen::VectorXd& payload_torque);

    Eigen::Matrix4d getCovariance() const;


private:

    void compute_meas_matrix();

    XBot::ModelInterface::ConstPtr _model;
    KalmanFilter _kalman;
    std::string _payload_link;

    Eigen::MatrixXd _J, _C, _Q, _R, _A, _Y;
    Eigen::VectorXd _r, _tau, _grav;
    Eigen::VectorXd _zero4d;

};

}

#endif // PAYLOAD_ESTIMATION_H
