#include "arm_test_gui/seedarm.h"

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

SeedArm::SeedArm()
{
    nodeHandle = (ros::NodeHandle *)0;
}

void SeedArm::seed()
{
    if (getNodeHandle()->ok())
    {
        initServiceClients();

        moveToGoal();
    }
}

void SeedArm::initServiceClients()
{
    // add before you call any kinematics services
    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
    ros::ServiceClient set_planning_scene_diff_client = getNodeHandle()->serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);

    arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;

    if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res))
    {
        ROS_WARN("BILLY Can't set planning scene");
        return;
    }

    move_arm = new SimpleActionClient("move_right_arm",true);
    move_arm->waitForServer();
    ROS_INFO("Connected to server");
}

void SeedArm::moveToGoal()
{
    arm_navigation_msgs::MoveArmGoal goalB;
    std::vector<std::string> names(7);
    names[0] = "shoulder_pan_joint";
    names[1] = "shoulder_tilt_joint";
    names[2] = "upper_arm_roll_joint";
    names[3] = "elbow_tilt_joint";
    names[4] = "forearm_roll_joint";
    names[5] = "wrist_tilt_joint";
    names[6] = "wrist_roll_joint";

    goalB.motion_plan_request.group_name = "right_arm";
    goalB.motion_plan_request.num_planning_attempts = 1;
    goalB.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

    goalB.motion_plan_request.planner_id= std::string("");
    goalB.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
    goalB.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

    for (unsigned int i = 0 ; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
    {
        goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
        goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.5;
        goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.2;
        goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.2;
    }

    if (getNodeHandle()->ok())
    {
        bool finished_within_time = false;
        move_arm->sendGoal(goalB);
        finished_within_time = move_arm->waitForResult(ros::Duration(2000.0));
        if (!finished_within_time)
        {
            move_arm->cancelGoal();
            ROS_INFO("Timed out achieving goal A");
        }
        else
        {
            actionlib::SimpleClientGoalState state = move_arm->getState();
            bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
            if(success)
                ROS_INFO("Action finished: %s",state.toString().c_str());
            else
                ROS_INFO("Action failed: %s",state.toString().c_str());
        }
    }
}

ros::NodeHandle *SeedArm::getNodeHandle()
{
    if (nodeHandle == (ros::NodeHandle *)0)
        nodeHandle = new ros::NodeHandle();

    return nodeHandle;
}
