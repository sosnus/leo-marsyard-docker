#include <ros/ros.h>
#include <cstdlib>
#include <leo_localization/Odometry.h>


Odometry::Odometry()
{
    nh.param("/leo_localization/robot_dimensions/wheelbase", wheelbase, 0.8);
    nh.param("/leo_localization/robot_dimensions/wheel_diameter", wheel_diameter, 0.2);

    nh.param("/leo_localization/odometry/use_initial_pose", use_initial_pose, false);
    nh.param("/leo_localization/odometry/gaussian", gaussian, false);

    nh.param("/leo_localization/odometry/alpha1RotRot", alpha1RotRot, 0.01);
    nh.param("/leo_localization/odometry/alpha2RotTrans", alpha2RotTrans, 0.01);
    nh.param("/leo_localization/odometry/alpha3TransTrans", alpha3TransTrans, 0.01);
    nh.param("/leo_localization/odometry/alpha4TransRot", alpha4TransRot, 0.01);

    nh.param<std::string>("/leo_localization/odometry/frame_id", frame_id_name, "odom");
    nh.param<std::string>("/leo_localization/odometry/topics/odomPublisher", odomPublisher_topic, "/odometry/odom");
    nh.param<std::string>("/leo_localization/odometry/topics/distPublisher", distPublisher_topic, "/odometry/pose_distribution");
    nh.param<std::string>("/leo_localization/odometry/topics/plcSubscriber", plcSubscriber_topic, "/PLC");
    nh.param<std::string>("/leo_localization/odometry/topics/resetSubscriber", resetSubscriber_topic, "/odometry/reset");
    nh.param<std::string>("/leo_localization/odometry/topics/initialPoseSubscriber", initialPoseSubscriber_topic, "/odometry/initial_pose");

    nh.param("/leo_localization/odometry/topics/publish_tf", publishTf, false);

    ROS_INFO_STREAM("wheelbase: " << wheelbase);
    ROS_INFO_STREAM("wheel_diameter: " << wheel_diameter);
    ROS_INFO_STREAM("use_initial_pose: " << use_initial_pose);
    ROS_INFO_STREAM("gaussian: " << gaussian);
    if(gaussian)
    {
        ROS_INFO_STREAM("alpha1RotRot: " << alpha1RotRot);
        ROS_INFO_STREAM("alpha2RotTrans: " << alpha2RotTrans);
        ROS_INFO_STREAM("alpha3TransTrans: " << alpha3TransTrans);
        ROS_INFO_STREAM("alpha4TransRot: " << alpha4TransRot);
    }
    ROS_INFO_STREAM("frame_id: " << frame_id_name);
    ROS_INFO_STREAM("publish_tf: " << publishTf);
    ROS_INFO("Topic names:");
    ROS_INFO_STREAM("odomPublisher: " << odomPublisher_topic);
    ROS_INFO_STREAM("distPublisher: " << distPublisher_topic);
    ROS_INFO_STREAM("plcSubscriber: " << plcSubscriber_topic);
    ROS_INFO_STREAM("resetSubscriber: " << resetSubscriber_topic);
    ROS_INFO_STREAM("initialPoseSubscriber: " << initialPoseSubscriber_topic);

    odomPublisher = nh.advertise<nav_msgs::Odometry>(odomPublisher_topic, 10, false);
    distPublisher = nh.advertise<geometry_msgs::PoseArray>(distPublisher_topic, 10, false);
    plcSubscriber = nh.subscribe<geometry_msgs::Twist> (plcSubscriber_topic, 100, &Odometry::plcCallback, this);
    resetSubscriber = nh.subscribe<std_msgs::Empty> (resetSubscriber_topic, 100, &Odometry::resetCallback, this);
    initialPoseSubscriber = nh.subscribe<nav_msgs::Odometry> (initialPoseSubscriber_topic, 100, &Odometry::initialPoseCallback, this);
}

Odometry::~Odometry()
{

}

void Odometry::plcCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    ros::Time actualTime = ros::Time::now();
    simpleDiffKinematics(actualTime, msg->linear.x, msg->linear.y);


    nav_msgs::Odometry odom;
    odom.header.stamp = actualTime;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = actualPosition2D.x;
    odom.pose.pose.position.y = actualPosition2D.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,actualPosition2D.theta);
    odom.twist.twist.linear.x = actualPosition2D.velPlatform;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = actualPosition2D.omega;

    for(int i = 0; i < 36; i++)                                     //fill covariances with zeros
    {
        odom.pose.covariance[i] = 0.0;
        odom.twist.covariance[i] = 0.0;
    }
    odom.pose.covariance[0] = 1.1 * actualPosition2D.velPlatform;       //x
    odom.pose.covariance[7] = 1.0;       //y
    odom.pose.covariance[14] = 9999999999.0;                   //z
    odom.pose.covariance[21] = 9999999999.0;                   //roll
    odom.pose.covariance[28] = 9999999999.0;                   //pitch
    odom.pose.covariance[35] = 10;       //yaw

    odom.twist.covariance[0] = 1.0;                           //vx
    odom.twist.covariance[7] = 1.0;                   //vy
    odom.twist.covariance[14] = 9999999999.0;
    odom.twist.covariance[21] = 9999999999.0;
    odom.twist.covariance[28] = 9999999999.0;
    odom.twist.covariance[35] = 1.5 * actualPosition2D.omega + 0.000001;                  //vyaw

    odomPublisher.publish(odom);


    geometry_msgs::TransformStamped leftWheelTransform;
    geometry_msgs::TransformStamped rightWheelTransform;
    geometry_msgs::TransformStamped odomTfMsg;
    leftWheelTransform.header.stamp = actualTime;
    leftWheelTransform.header.frame_id = "base_link";
    leftWheelTransform.child_frame_id = "wheel_left_link";
    leftWheelTransform.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, actualPosition2D.angleLeft*(3.1415/180.0), 0.0);

    rightWheelTransform.header.stamp = actualTime;
    rightWheelTransform.header.frame_id = "base_link";
    rightWheelTransform.child_frame_id = "wheel_right_link";
    rightWheelTransform.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, actualPosition2D.angleRight*(3.1415/180.0), 0.0);

   if(publishTf)
    {
      odomTfMsg.header.stamp = actualTime;
      odomTfMsg.header.frame_id = "odom";
      odomTfMsg.child_frame_id = "base_link";
      odomTfMsg.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, actualPosition2D.theta);
      odomTfMsg.transform.translation.x = actualPosition2D.x;
      odomTfMsg.transform.translation.y = actualPosition2D.y;
      odomTf.sendTransform(odomTfMsg);
    }

    rightWheel.sendTransform(rightWheelTransform);
    leftWheel.sendTransform(leftWheelTransform);

}

void Odometry::resetCallback(const std_msgs::Empty::ConstPtr &msg)
{
    robotPose empty = robotPose();
    this->actualPosition2D = empty;
    reset = true;
}

void Odometry::initialPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(reset && use_initial_pose)
    {
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        initialPose.x       = msg->pose.pose.position.x;
        initialPose.y       = msg->pose.pose.position.y;
        initialPose.theta   = (float)yaw;

        actualPosition2D.x = initialPose.x;
        actualPosition2D.y = initialPose.y;
        actualPosition2D.theta = initialPose.theta;
        reset = false;
    }

}


Odometry::robotPose Odometry::simpleDiffKinematics(ros::Time timestamp, float vL, float vR)
{
    struct robotPose newPose;

    float dt = 0.0;
    if((this->actualPosition2D.timestamp).toSec())
    {
        dt = (timestamp - this->actualPosition2D.timestamp).toSec();
    }

    //ROS_INFO_STREAM("dt: " << dt);
    float omega = 0.0;
    float v = 0.0, ds = 0.0, dTheta = 0.0;

    v = (vL + vR) / 2.0;

    omega = (vR - vL) / wheelbase;  //platform rotation speed around ICR point
    dTheta = omega*dt;              //angle since the previous measurement
    ds= v*dt;

//    if(vR == vL)
//    {

//          actualPosition2D.x      += vL*dt*cos(actualPosition2D.theta);
//          actualPosition2D.y      += vL*dt*sin(actualPosition2D.theta);
//          actualPosition2D.theta  += dTheta;
//    }
//    else
//    {
//          // Calculate the radius
//          float R  = (wheelbase/2.0)*((vL + vR) / (vR - vL));
//          float ICC_x = actualPosition2D.x - R*sin(dTheta);
//          float ICC_y = actualPosition2D.y + R*cos(dTheta);
//          actualPosition2D.x  = cos(dTheta)*(actualPosition2D.x-ICC_x) - sin(dTheta)*(actualPosition2D.y-ICC_y) + ICC_x;
//          actualPosition2D.y  = sin(dTheta)*(actualPosition2D.x-ICC_x) + cos(dTheta)*(actualPosition2D.y-ICC_y) + ICC_y;
//          actualPosition2D.theta = actualPosition2D.theta + dTheta;
//    }


    actualPosition2D.x      += ds*cos(actualPosition2D.theta);
    actualPosition2D.y      += ds*sin(actualPosition2D.theta);
    actualPosition2D.theta  += dTheta;

    actualPosition2D.velPlatform = v;
    actualPosition2D.omega = omega;

    actualPosition2D.deltaSLeft = vL*dt;
    actualPosition2D.deltaSRight = vR*dt;

    actualPosition2D.angleLeft += actualPosition2D.deltaSLeft;
    actualPosition2D.angleRight += actualPosition2D.deltaSRight;
    if(actualPosition2D.angleLeft >= 360) actualPosition2D.angleLeft -= 360;
    if(actualPosition2D.angleLeft <= -360) actualPosition2D.angleLeft += 360;

    if(actualPosition2D.angleRight >= 360) actualPosition2D.angleRight -= 360;
    if(actualPosition2D.angleRight <= -360) actualPosition2D.angleRight += 360;
    //skalowanie wartości kąta

    actualPosition2D.timestamp = timestamp;
    return newPose;
}

Odometry::robotPose Odometry::poseProbabilityDistribution(ros::Time timestamp, float vL, float vR)
{

}
