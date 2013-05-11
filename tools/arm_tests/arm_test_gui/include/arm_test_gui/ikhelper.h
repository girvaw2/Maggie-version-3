#ifndef IKHELPER_H
#define IKHELPER_H

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <arm_test_gui/ExecuteCartesianIKTrajectory.h>
#include <vector>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

#define MAX_JOINT_VEL 0.5  //in radians/sec

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const std::string ARM_IK_NAME = "/maggie_right_arm_kinematics/get_ik";
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class IKHelper
{
public:
    IKHelper();
    ~IKHelper();
    void moveToGoal(geometry_msgs::Pose &pose);

private:
    bool executeCartesianIKTrajectory( arm_test_gui::ExecuteCartesianIKTrajectory::Request &req, arm_test_gui::ExecuteCartesianIKTrajectory::Response &res);
    bool getIKSolution(geometry_msgs::PoseStamped pose, double start_angles[7], double solution[7], std::string link_name);
    void getCurrentJointAngles(double current_angles[7]);
    bool executeJointTrajectory(std::vector<double *> joint_trajectory);
    void initialiseGoal(control_msgs::FollowJointTrajectoryGoal &goal);
    ros::NodeHandle *getNodeHandle();

private:
    ros::ServiceClient ik_client;
    ros::ServiceServer service;
    kinematics_msgs::GetPositionIK::Request  ik_request;
    kinematics_msgs::GetPositionIK::Response ik_response;
    TrajClient *action_client;
    ros::NodeHandle *nodeHandle;
};

#endif // IKHELPER_H
