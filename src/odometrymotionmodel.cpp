#include "odometry_motion_model/odometrymotionmodel.h"

OdometryMotionModel::OdometryMotionModel(const ros::NodeHandle & nh_) : nh_priv("~"), nh(nh_), last_yaw(0.0)
{
    nh_priv.param("alpha_1",alpha_1, 0.05);
    nh_priv.param("alpha_2",alpha_2, 0.001);
    nh_priv.param("alpha_3",alpha_3, 5.0);
    nh_priv.param("alpha_4",alpha_4, 0.05);
    nh_priv.param("use_tf",use_tf, false);
    nh_priv.param<std::string>("base_footprint",base_footprint, "base_footprint");

    ROS_INFO_STREAM("alpha_1:"<<alpha_1);
    ROS_INFO_STREAM("alpha_2:"<<alpha_2);
    ROS_INFO_STREAM("alpha_3:"<<alpha_3);
    ROS_INFO_STREAM("alpha_4:"<<alpha_4);
    ROS_INFO_STREAM("use_tf:"<<use_tf);

    if(!use_tf)
    {
        odom_sub=nh.subscribe("odom", 1, &OdometryMotionModel::odometry_callback, this);
    }

    odom_pub=nh.advertise<nav_msgs::Odometry>("/odom_with_uncertainty",1);
    odometry_undertainty = nh.advertise<visualization_msgs::Marker>("/location_undertainty",1);
    std::cout << "X:"<< X << std::endl;
}


void OdometryMotionModel::odometry_callback(const nav_msgs::OdometryConstPtr & odom_msg)
{

    tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double dx=odom_msg->pose.pose.position.x-last_odom_msg.pose.pose.position.x;
    double dy=odom_msg->pose.pose.position.y-last_odom_msg.pose.pose.position.y;
    double dtheta=yaw-last_yaw;

    //std::cout << yaw << " " << last_yaw<< std::endl;
    double delta_rot1=atan2(dy,dx);
    double delta_trans=sqrt(dx*dx+dy*dy);
    double delta_rot2=dtheta-delta_rot1;
    //std::cout << "dx:"<< dx<< " dy:"<< dy << " dtheta:"<< dtheta<< std::endl;
    // Update odom
    X[0] += cos(delta_rot1)*delta_trans;
    X[1] += sin(delta_rot1)*delta_trans;
    X[2] += dtheta;
    angleOverflowCorrect(X[2]);

    double sigma_rot1=alpha_1*fabs(delta_rot1)+alpha_2*delta_trans;
    double sigma_trans=alpha_3*delta_trans+alpha_4*(fabs(delta_rot1+delta_rot2));
    double sigma_rot2=alpha_1*fabs(delta_rot2)+alpha_2*delta_trans;
    Eigen::Matrix<double,3,3> Sigma;
    Sigma(0,0)=sigma_trans;
    Sigma(1,1)=sigma_rot1;
    Sigma(2,2)=sigma_rot2;

    Eigen::Matrix<double,3,3> J;
    J(0,0)=-sin(delta_rot1); J(0,1)=cos(delta_rot1); J(0,2)=0;
    J(1,0)=cos(delta_rot1);  J(1,1)=sin(delta_rot1);  J(1,2)=0;
    J(2,0)=1;  J(2,1)=0;  J(2,2)=1;
    Q += J*Sigma*J.transpose();
    Q = (Q + Q.transpose()) * 0.5;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(X[2]);
    nav_msgs::Odometry odom_msg_out;
    odom_msg_out.header=odom_msg->header;
    odom_msg_out.child_frame_id=odom_msg->child_frame_id;
    odom_msg_out.pose.pose.position.x=X[0];
    odom_msg_out.pose.pose.position.y=X[1];
    odom_msg_out.pose.pose.position.z=0;
    odom_msg_out.pose.pose.orientation=quat;

    for(int i=0; i<3; ++i)
    {
        for(int j=0; j<3;++j)
        {
            int index = j+i*3;
            odom_msg_out.pose.covariance[index]=Q(i,j);
        }
    }
    drawCovariance(X,Q);
    odom_pub.publish(odom_msg_out);
    last_odom_msg=*odom_msg;
    last_yaw=yaw;
    return;
}

void OdometryMotionModel::drawCovariance(const Eigen::Matrix<double,3,1>& mean, const Eigen::Matrix<double,3,3>& covMatrix)
{
    visualization_msgs::Marker tempMarker;
    tempMarker.pose.position.x = 0;
    tempMarker.pose.position.y = 0;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,3,3> > eig(covMatrix);

    const Eigen::Matrix<double,3,1>& eigValues (eig.eigenvalues());
    const Eigen::Matrix<double,3,3>& eigVectors (eig.eigenvectors());

    float angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));


    tempMarker.type = visualization_msgs::Marker::SPHERE;

    double lengthMajor = sqrt(eigValues[0]);
    double lengthMinor = sqrt(eigValues[1]);

    tempMarker.scale.x = 3*lengthMajor;
    tempMarker.scale.y = 3*lengthMinor;
    tempMarker.scale.z = 0.001;

    tempMarker.color.a = 1.0;
    tempMarker.color.r = 1.0;

    tempMarker.pose.orientation.w = cos(angle*0.5);
    tempMarker.pose.orientation.z = sin(angle*0.5);

    tempMarker.header.frame_id=base_footprint;
    tempMarker.id = 0;

    odometry_undertainty.publish(tempMarker);
}
