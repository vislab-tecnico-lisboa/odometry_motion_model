#include "odometry_motion_model/odometrymotionmodel.h"

OdometryMotionModel::OdometryMotionModel(const ros::NodeHandle & nh_) : nh_priv("~"), nh(nh_), last_yaw(0.0), first(true)
{
    nh_priv.param("alpha_1",alpha_1, 0.05);
    nh_priv.param("alpha_2",alpha_2, 0.001);
    nh_priv.param("alpha_3",alpha_3, 5.0);
    nh_priv.param("alpha_4",alpha_4, 0.05);
    nh_priv.param("use_tf",use_tf, false);
    nh_priv.param<std::string>("base_footprint",base_footprint, "base_footprint");
    alpha_1=alpha_1*M_PI/180; // Convert to radians
    alpha_3=alpha_3*M_PI/180; // Convert to radians
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

    if(first)
    {
        first=false;
        last_odom_msg=*odom_msg;
        last_yaw=yaw;
        return;
    }

    double dx=odom_msg->pose.pose.position.x-last_odom_msg.pose.pose.position.x;
    double dy=odom_msg->pose.pose.position.y-last_odom_msg.pose.pose.position.y;
    double dtheta=yaw-last_yaw;

    double delta_rot1=atan2(dy,dx);
    double delta_trans=sqrt(dx*dx+dy*dy);
    double delta_rot2=dtheta-delta_rot1;
    double delta_t=(odom_msg->header.stamp-last_odom_msg.header.stamp).toSec();
    std::cout << "dx:"<< dx << std::endl;
    std::cout << "dy:"<< dy << std::endl;

    std::cout << "delta_rot_before:"<< delta_rot1 << std::endl;
    std::cout << fabs(dx) << std::endl;
    if(fabs(dy)<0.001)
    {
        delta_rot1=0.0;
        delta_rot2=dtheta;
    }

    std::cout << "delta_rot1:"<< delta_rot1 << std::endl;
    std::cout << "delta_rot2:"<< delta_rot2 << std::endl;
    //angleOverflowCorrect(delta_rot1);
    // Update odom
    X[0] += cos(delta_rot1)*delta_trans;
    X[1] += sin(delta_rot1)*delta_trans;
    X[2] += dtheta;
    angleOverflowCorrect(X[2]);

    double var_rot1=alpha_1*delta_rot1*delta_rot1+alpha_2*delta_trans*delta_trans;
    double var_trans=alpha_3*delta_trans*delta_trans+alpha_4*(delta_rot1*delta_rot1+delta_rot2*delta_rot2);
    double var_rot2=alpha_1*delta_rot2*delta_rot2+alpha_2*delta_trans*delta_trans;

    Eigen::Matrix<double,3,3> Sigma;
    Sigma(0,0)=var_rot1;
    Sigma(1,1)=var_trans;
    Sigma(2,2)=var_rot2;

    std::cout << "SIGMA:"<< Sigma << std::endl;
    Eigen::Matrix<double,3,3> J;
    J(0,0)=1;  J(0,1)=0;  J(0,2)=-delta_trans*sin(last_yaw+delta_rot1);
    J(1,0)=0;  J(1,1)=1;  J(1,2)=delta_trans*cos(last_yaw+delta_rot1);
    J(2,0)=0;  J(2,1)=0;  J(2,2)=1;
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
    odom_msg_out.twist.twist.linear.x=delta_trans/delta_t;
    odom_msg_out.twist.twist.angular.z=dtheta/delta_t;

    for(int i=0; i<2; ++i)
    {
        for(int j=0; j<2;++j)
        {
            int index = j+i*2;
            odom_msg_out.pose.covariance[index]=Q(i,j);
        }
    }
    int index = 5+5*6;
    odom_msg_out.pose.covariance[index]=Q(2,2);

    index= 0+5*6;
    odom_msg_out.pose.covariance[index]=Q(2,0);
    index= 5+0*6;
    odom_msg_out.pose.covariance[index]=Q(0,2);
    index= 1+5*6;
    odom_msg_out.pose.covariance[index]=Q(2,1);
    index= 5+1*6;
    odom_msg_out.pose.covariance[index]=Q(1,2);

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

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,2,2> > eig(covMatrix.block<2,2>(0,0));

    const Eigen::Matrix<double,2,1>& eigValues (eig.eigenvalues());
    const Eigen::Matrix<double,2,2>& eigVectors (eig.eigenvectors());

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
