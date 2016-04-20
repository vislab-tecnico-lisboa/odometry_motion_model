#ifndef ODOMETRYMOTIONMODEL_H
#define ODOMETRYMOTIONMODEL_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
class OdometryMotionModel
{
    ros::NodeHandle nh_priv;
    ros::NodeHandle nh;
    ros::Publisher odom_pub;
    ros::Publisher odometry_undertainty;
    ros::Subscriber odom_sub;
    nav_msgs::Odometry last_odom_msg;
    std::string base_footprint;
    double rate;
    void odometry_callback(const nav_msgs::OdometryConstPtr & odom_msg);

    void angleOverflowCorrect(double& a)
    {
        while ((a) >  M_PI) a -= 2*M_PI;
        while ((a) < -M_PI) a += 2*M_PI;
    }

    void drawCovariance(const Eigen::Matrix<double,3,1>& mean, const Eigen::Matrix<double,3,3>& covMatrix);

    bool use_tf;

    // Odometry motion model params
    double alpha_1, alpha_2, alpha_3, alpha_4;

    Eigen::Matrix<double,2,2> getVelocityCov(const double & v,
                                             const double & w,
                                             const double & alpha_1_,
                                             const double & alpha_2_,
                                             const double & alpha_3_,
                                             const double & alpha_4_)
    {
        Eigen::Matrix<double,2,2> M;
        M(0,0)=alpha_1_*v*v+alpha_2_*w*w;
        M(1,1)=alpha_3_*v*v+alpha_4_*w*w;

        Eigen::Matrix<double,3,2> V;

        return M;
    }

    Eigen::Matrix<double,3,1> X;
    Eigen::Matrix<double,3,3> Q;
    double last_yaw;
    bool first;

public:
    OdometryMotionModel(const ros::NodeHandle & nh_);
};

#endif // ODOMETRYMOTIONMODEL_H
