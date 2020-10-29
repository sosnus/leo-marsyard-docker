#include <leo_localization/Odometry.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_node");

    Odometry odom_node;

    ros::spin();
    return 0;
}
