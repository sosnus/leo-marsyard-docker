#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

class Odometry
{
public:
    Odometry();
    ~Odometry();
    struct robotPose
    {
        float x = 0.0;
        float y = 0.0;
        float theta = 0.0;

        float velX = 0.0;
        float velY = 0.0;
        float velPlatform = 0.0;
        float velWheelLeft = 0.0;
        float velWheelRight = 0.0;
        float omega = 0.0;

        float deltaTrans = 0.0;
        float deltaRot1 = 0.0;
        float deltaRot2 = 0.0;
        float sigmaTrans = 0.0;
        float sigmaRot1 = 0.0;
        float sigmaRot2 = 0.0;

        float deltaSLeft = 0.0;
        float deltaSRight = 0.0;
        //przemieszczenie chwilowe kół

        float angleLeft = 0.0;
        float angleRight = 0.0;
        //kąty kół

        ros::Time timestamp;
    };
    //########## parameters ##########
    double wheelbase = 0.8;
    double wheel_diameter = 0.0;

    bool use_initial_pose = false;
    bool gaussian = false;
    double alpha1RotRot = 0.01;
    double alpha2RotTrans = 0.01;
    double alpha3TransTrans = 0.01;
    double alpha4TransRot = 0.01;

    std::string frame_id_name = "/odom";
    std::string odomPublisher_topic = "/odometry/odom";
    std::string distPublisher_topic = "/odometry/pose_distribution";
    std::string plcSubscriber_topic = "/PLC";
    std::string resetSubscriber_topic = "/odometry/reset";
    std::string initialPoseSubscriber_topic = "/odometry/initial_pose";

    bool publishTf = false;
    //###############################
    struct robotPose simpleDiffKinematics(ros::Time timestamp, float vL, float vR);
    struct robotPose poseProbabilityDistribution(ros::Time timestamp, float vL, float vR);
    tf::TransformBroadcaster rightWheel;
    tf::TransformBroadcaster leftWheel;
    tf::TransformBroadcaster odomTf;


private:
    ros::NodeHandle nh;
    ros::Publisher odomPublisher;
    ros::Publisher distPublisher;
    ros::Subscriber plcSubscriber;
    ros::Subscriber resetSubscriber;
    ros::Subscriber initialPoseSubscriber;



    void plcCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void resetCallback(const std_msgs::Empty::ConstPtr& msg);
    void initialPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);


    bool reset = true; //flaga ustawiana po resecie


    struct robotPose initialPose;
    struct robotPose actualPosition2D;
};

#endif // WHEELODOMETRY_H
