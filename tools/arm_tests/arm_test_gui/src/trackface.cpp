#include "arm_test_gui/trackface.h"
#include <tf/transform_listener.h>
#include <iostream>
#include "arm_test_gui/ikhelper.h"

TrackFace::TrackFace()
{
    nodeHandle = (ros::NodeHandle *)0;
    getNodeHandle(); // need to initialse nodeHandle for call to waitForService
}

TrackFace::~TrackFace()
{
    //std::cout << "Destructing TrackFace" << std::endl;
}

void TrackFace::startFaceTracking()
{
    int count = 0;
    ros::Rate r(10);
    while(true)
    {
        geometry_msgs::PointStampedPtr ps = getFacePoint();

        if (ps->point.x != 0.0)
        {
            char str[100];
            sprintf (str, "x = %f", ps->point.x);
            std::cout << str << std::endl;
        }

        if (count++ > 9)
        {
            //moveArmToFacePosition();
        }

        ros::spinOnce();
        r.sleep();
    }
}

void TrackFace::moveArmToFacePosition(const ros::TimerEvent& event)
{
}

void TrackFace::moveArmToFacePosition()
{
    IKHelper ik;

    double start_orientation[] = {0.655082, -0.393060, 0.095405, 0.638176};

    geometry_msgs::Pose pose;

    pose.position.x = 0.453;
    pose.position.y = 0.046;
    pose.position.z = 0.175;

    pose.orientation.x = start_orientation[0];
    pose.orientation.y = start_orientation[1];
    pose.orientation.z = start_orientation[2];
    pose.orientation.w = start_orientation[3];

    ik.moveToGoal(pose);
}

geometry_msgs::PointStampedPtr TrackFace::getFacePoint()
{
    geometry_msgs::PointStampedPtr ps;
    tf::TransformListener listener;

    ros::Rate rate(10.0);
    if (getNodeHandle()->ok())
    {
        geometry_msgs::PointStampedConstPtr target_msg = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/target_point");

        std::cout << "message received" << std::endl;

        try
        {
            if (!listener.waitForTransform("/torso_link", target_msg->header.frame_id, target_msg->header.stamp, ros::Duration(10.0)))
            {
                std::cout << "no transform to torso_link" << std::endl;
                return ps; //boost::shared_ptr<geometry_msgs::PointStamped>(&ps);
            }

            char src_msg[256];
            sprintf (src_msg, "Found transform from %s to torso_link. x = %f y = %f z = %f", target_msg->header.frame_id.c_str(), target_msg->point.x, target_msg->point.y, target_msg->point.z);
            std::cout << src_msg << std::endl;

            //tf::StampedTransform transform;
            geometry_msgs::PointStamped pout;

            listener.transformPoint("/torso_link", ros::Time(0), *target_msg, target_msg->header.frame_id, pout);

            return boost::make_shared<geometry_msgs::PointStamped>(pout);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
    }
    return ps; // .PointStamped_.//boost::shared_ptr<geometry_msgs::PointStamped>(&ps);
}

ros::NodeHandle *TrackFace::getNodeHandle()
{
    if (nodeHandle == (ros::NodeHandle *)0)
        nodeHandle = new ros::NodeHandle();

    return nodeHandle;
}
