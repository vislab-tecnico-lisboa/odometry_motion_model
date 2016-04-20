#include "odometry_motion_model/odometrymotionmodel.h"



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odometry_motion_model_node");
    ros::NodeHandle node;

    ros::NodeHandle node_priv("~");

    double spin_rate;
    node_priv.param("rate",spin_rate, 30.0);
    ros::Rate rate(spin_rate);

    OdometryMotionModel odometry_motion_model(node);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();

    return 0;
}

