#include <estimation_utils/payload/payload_estimation.h>

#include <ros/ros.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <XBotInterface/RobotInterface.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen_conversions/eigen_msg.h>


int main(int argc, char **argv)
{
    // init ros, get handles
    ros::init(argc, argv, "payload_estimation_node");
    ros::NodeHandle nh_priv("~");

    ros::Publisher payload_torque_pub = nh_priv.advertise<std_msgs::Float64MultiArray>("payload_torque", 1);
    ros::Publisher payload_params_pub = nh_priv.advertise<std_msgs::Float64MultiArray>("payload_params", 1);
    
    // get robot, model, and one imu
    auto robot = XBot::RobotInterface::getRobot(XBot::ConfigOptionsFromParamServer());
    auto model = XBot::ModelInterface::getModel(XBot::ConfigOptionsFromParamServer());

    double dt = nh_priv.param("rate", 0.01);
    double torque_noise = nh_priv.param("torque_noise", 1.0);
    double mass_rate = nh_priv.param("mass_rate", 0.01);
    double com_rate = nh_priv.param("com_rate", 0.01);
    std::string payload_link;
    if(!nh_priv.getParam("payload_link", payload_link))
    {
        ROS_ERROR("~payload_link mandatory parameter missing");
        exit(1);
    }

    estimation_utils::PayloadEstimation payload_estimator(
        model, 
        payload_link, 
        std::pow(torque_noise*dt, 2.), 
        std::pow(mass_rate*dt, 2.), 
        std::pow(com_rate*dt, 2.)
    );

    ros::Rate rate(1./dt);

    Eigen::VectorXd payload_torque;
    Eigen::Vector4d payload_params;

    while(ros::ok())
    {
        robot->sense(false);
        model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);
        model->update();

        payload_estimator.compute(payload_torque, payload_params);

        std_msgs::Float64MultiArray msg;
        
        tf::matrixEigenToMsg(payload_torque, msg);
        payload_torque_pub.publish(msg);

        tf::matrixEigenToMsg(payload_params, msg);
        payload_params_pub.publish(msg);

        rate.sleep();

    }
}