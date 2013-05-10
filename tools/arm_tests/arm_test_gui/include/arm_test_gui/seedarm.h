#ifndef SEEDARM_H
#define SEEDARM_H

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> SimpleActionClient;

class SeedArm
{
public:
    SeedArm();

    void seed();

private:
    void initServiceClients();
    void moveToGoal();
    ros::NodeHandle *getNodeHandle();

private:
    ros::NodeHandle *nodeHandle;
    SimpleActionClient *move_arm;
};

#endif // SEEDARM_H
