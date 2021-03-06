#include "arm_test_gui/ikhelper.h"

IKHelper::IKHelper()
{

    nodeHandle = (ros::NodeHandle *)0;
    getNodeHandle(); // need to initialse nodeHandle for call to waitForService

    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);

    ros::ServiceClient set_planning_scene_diff_client = getNodeHandle()->serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);

    arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;

    if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res))
    {
        ROS_WARN("Can't get planning scene");
        return;
    }

    //create a client function for the IK service
    ik_client = getNodeHandle()->serviceClient<kinematics_msgs::GetPositionIK>(ARM_IK_NAME, true);


    fk_query_client = getNodeHandle()->serviceClient<kinematics_msgs::GetKinematicSolverInfo>(ARM_FK_SOLVER_NAME);
    fk_client = getNodeHandle()->serviceClient<kinematics_msgs::GetPositionFK>(ARM_FK_NAME);

    //wait for the various services to be ready
    ROS_INFO("Waiting for services to be ready");
    ros::service::waitForService(ARM_IK_NAME);
    ros::service::waitForService(ARM_FK_SOLVER_NAME);
    ros::service::waitForService(ARM_FK_NAME);
    ROS_INFO("Services ready");


    if(fk_query_client.call(fk_solver_request_,fk_solver_response_))
    {
      for(unsigned int i=0; i< fk_solver_response_.kinematic_solver_info.joint_names.size(); i++)
      {
        ROS_INFO("Joint: %d %s", i, fk_solver_response_.kinematic_solver_info.joint_names[i].c_str());
      }
    }
    else
    {
        std::cout << "Could not call FK query service" << std::endl;
    }

    //tell the joint trajectory action client that we want
    //to spin a thread by default
    action_client = new TrajClient("/arm_controller/follow_joint_trajectory", true);

    //wait for the action server to come up
    while(ros::ok() && !action_client->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the joint_trajectory_action action server to come up");
    }
}

IKHelper::~IKHelper()
{
    delete action_client;
}

void IKHelper::moveToGoal(geometry_msgs::Pose &pose)
{
    if (!isReachable(pose))
    {
        findIncrementalTrajectory(pose);
        std::cout << "Can't REACH THIS !!!!!" << std::endl;
    }

    // we can reach the target pose, so carry right on...
    arm_test_gui::ExecuteCartesianIKTrajectory::Request req;
    arm_test_gui::ExecuteCartesianIKTrajectory::Response res;

    req.header.frame_id = "torso_link";
    req.poses.push_back(pose);

    //buildTrajectoryRequest(req);
    executeCartesianIKTrajectory(req, res);
}

//service function for executeCartesianIKTrajectory
bool IKHelper::executeCartesianIKTrajectory( arm_test_gui::ExecuteCartesianIKTrajectory::Request &req, arm_test_gui::ExecuteCartesianIKTrajectory::Response &res)
{
    //    int trajectory_length = req.poses.size();
    int i, j;

    //IK takes in Cartesian poses stamped with the frame they belong to
    geometry_msgs::PoseStamped stamped_pose;
    stamped_pose.header = req.header;
    stamped_pose.header.stamp = ros::Time::now();
    bool success;

    std::vector<trajectory_array_ptr> joint_trajectory;

    //get the current joint angles (to find ik solutions close to)
    double last_angles[7];
    getCurrentJointAngles(last_angles);

    for(i=0; i<1 /*trajectory_length*/; i++)
    {
        stamped_pose.pose = req.poses[i];

        trajectory_array_ptr trajectory_array(new double[7]);

        success = getIKSolution(stamped_pose, last_angles, trajectory_array, "wrist_roll_link");
        joint_trajectory.push_back(trajectory_array);

        if(!success)
        {
            ROS_ERROR("IK solution not found for trajectory point number %d!\n", i);
            return 0;
        }

        for(j=0; j<7; j++)
        {
            last_angles[j] = trajectory_array[j];
        }
    }

    ROS_INFO("executing joint trajectory");
    success = executeJointTrajectory(joint_trajectory);
    res.success = success;

    return success;
}

bool IKHelper::getIKSolution(geometry_msgs::PoseStamped pose, double start_angles[7], trajectory_array_ptr solution, std::string link_name)
{

    kinematics_msgs::GetPositionIK::Request  ik_request;
    kinematics_msgs::GetPositionIK::Response ik_response;

    ik_request.timeout = ros::Duration(5.0);

    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("shoulder_pan_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("shoulder_tilt_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("upper_arm_roll_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("elbow_tilt_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("forearm_roll_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("wrist_tilt_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("wrist_roll_joint");

    ik_request.ik_request.ik_link_name = link_name;

    ik_request.ik_request.pose_stamped = pose;
    ik_request.ik_request.ik_seed_state.joint_state.position.resize(7);

    for(int i=0; i<7; i++)
    {
        ik_request.ik_request.ik_seed_state.joint_state.position[i] = start_angles[i];
        ROS_INFO("Seed state for joint %s = %f", ik_request.ik_request.ik_seed_state.joint_state.name[i].c_str(), ik_request.ik_request.ik_seed_state.joint_state.position[i]);
    }

    ROS_INFO("request pose: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

    bool ik_service_call = ik_client.call(ik_request,ik_response);
    if(!ik_service_call)
    {
        ROS_ERROR("IK service call failed!");
        return 0;
    }

    if(ik_response.error_code.val == ik_response.error_code.SUCCESS)
    {
        for(int i=0; i<7; i++)
        {
            solution[i] = ik_response.solution.joint_state.position[i];
        }
        ROS_INFO("solution angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", solution[0],solution[1], solution[2],solution[3], solution[4],solution[5], solution[6]);
        ROS_INFO("IK service call succeeded");
        return 1;
    }
    ROS_INFO("IK service call error code: %d", ik_response.error_code.val);
    return 0;
}


//send a desired joint trajectory to the joint trajectory action
//and wait for it to finish
bool IKHelper::executeJointTrajectory(std::vector<trajectory_array_ptr> joint_trajectory)
{
    int i, j;
    int trajectorylength = joint_trajectory.size();

    //get the current joint angles
    double current_angles[7];
    getCurrentJointAngles(current_angles);

    //fill the goal message with the desired joint trajectory
    control_msgs::FollowJointTrajectoryGoal goal;
    initialiseGoal(goal);

    goal.trajectory.points.resize(trajectorylength+1);

    //set the first trajectory point to the current position
    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].velocities.resize(7);
    for(j=0; j<7; j++)
    {
        ROS_INFO ("executeJointTrajectory - Joint names = %s Current Angle = %f", goal.trajectory.joint_names[j].c_str(), current_angles[j]);
        goal.trajectory.points[0].positions[j] = current_angles[j];
        goal.trajectory.points[0].velocities[j] = 0.0;
    }

    //make the first trajectory point start 0.25 seconds from when we run
    goal.trajectory.points[0].time_from_start = ros::Duration(0.25);

    //fill in the rest of the trajectory
    double time_from_start = 0.25;
    for(i=0; i<trajectorylength; i++)
    {
        goal.trajectory.points[i+1].positions.resize(7);
        goal.trajectory.points[i+1].velocities.resize(7);

        //fill in the joint positions (velocities of 0 mean that the arm
        //will try to stop briefly at each waypoint)
        for(j=0; j<7; j++)
        {
            goal.trajectory.points[i+1].positions[j] = joint_trajectory[i][j];
            goal.trajectory.points[i+1].velocities[j] = 0.0;
        }

        //compute a desired time for this trajectory point using a max
        //joint velocity
        double max_joint_move = 0;
        for(j=0; j<7; j++)
        {
            double joint_move = fabs(goal.trajectory.points[i+1].positions[j] - goal.trajectory.points[i].positions[j]);
            if(joint_move > max_joint_move)
                max_joint_move = joint_move;
        }

        double seconds = max_joint_move/MAX_JOINT_VEL;
        ROS_INFO("max_joint_move: %0.3f, seconds: %0.3f", max_joint_move, seconds);
        time_from_start += seconds;
        goal.trajectory.points[i+1].time_from_start = ros::Duration(time_from_start);
    }

    //when to start the trajectory
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.25);

    ROS_INFO("Sending goal to joint_trajectory_action");
    action_client->sendGoal(goal);

    action_client->waitForResult();

    //get the current joint angles for debugging
    getCurrentJointAngles(current_angles);
    ROS_INFO("joint angles after trajectory: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n",current_angles[0],current_angles[1],current_angles[2],current_angles[3],current_angles[4],current_angles[5],current_angles[6]);

    if(action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Hooray, the arm finished the trajectory!");
        return 1;
    }
    ROS_INFO("The arm failed to execute the trajectory.");
    return 0;
}

void IKHelper::initialiseGoal(control_msgs::FollowJointTrajectoryGoal &goal)
{
    //have to specify the order of the joints we're sending in our
    //joint trajectory goal, even if they're already on the param server

    goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("shoulder_tilt_joint");
    goal.trajectory.joint_names.push_back("upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("elbow_tilt_joint");
    goal.trajectory.joint_names.push_back("forearm_roll_joint");
    goal.trajectory.joint_names.push_back("wrist_tilt_joint");
    goal.trajectory.joint_names.push_back("wrist_roll_joint");
}

//figure out where the arm is now
void IKHelper::getCurrentJointAngles(double current_angles[7])
{
    //get a single message from the topic 'r_arm_controller/state'
    control_msgs::FollowJointTrajectoryFeedbackConstPtr state_msg = ros::topic::waitForMessage<control_msgs::FollowJointTrajectoryFeedback>("arm_controller/state");

    /*
     * The list of positions within the state_msg are in alphabtical order, so we need to populate the current_angle array correctly...
     */

    current_angles[0] = state_msg->actual.positions[2];
    current_angles[1] = state_msg->actual.positions[3];
    current_angles[2] = state_msg->actual.positions[4];
    current_angles[3] = state_msg->actual.positions[0];
    current_angles[4] = state_msg->actual.positions[1];
    current_angles[5] = state_msg->actual.positions[6];
    current_angles[6] = state_msg->actual.positions[5];
}

/*
 * If we haven't been able find a direct solution to the target pose,
 * subdivide the path to the target into discrete steps.
 * Then ensure that each step is reachable and finally we can reach the target.
 * If we cannot find a path to the target, assume that the target is not reachable.
 */
bool IKHelper::findIncrementalTrajectory(geometry_msgs::Pose &pose)
{
    geometry_msgs::Pose currentPose;
    if (!getCurrentPose(currentPose))
        return false;

    std::vector<float> seedRange;
    calcSeedRange(currentPose.position.y, pose.position.y, seedRange);

//    float yDiff = currentPose.position.y - pose.position.y;
//    ROS_INFO("Y Position difference: %f", yDiff);

//    int yCount = static_cast<int>(yDiff / SEED_INCREMENT);
////    for (int y = currentPose.position.y;;yDiff < 0 ?
    return false;

}

void showSeedRange (float i)
{
    std::cout << "Seed element " << i << std::endl;
}

class SeedRangeHelper
{
    public:
    SeedRangeHelper (float start_val, bool forward) : start_val_( start_val ), forward_(forward) {}

        float operator() ()
        {
            if (forward_)
                start_val_ += 0.1;
            else
                start_val_ -= 0.1;

            return start_val_;
        }

        private:
            float start_val_;
            bool forward_;
};

void IKHelper::calcSeedRange(float start, float end, std::vector<float> &range)
{
    int steps = static_cast<int>(fabs((start - end) / SEED_INCREMENT) - (SEED_INCREMENT / 2));

    SeedRangeHelper seedRangeHelper(start, end > start ? true : false);

    range.resize(steps);

    std::generate(range.begin(), range.end(), seedRangeHelper);

    for_each(range.begin(), range.end(), showSeedRange);
}

bool IKHelper::getCurrentPose(geometry_msgs::Pose &pose)
{
    kinematics_msgs::GetPositionFK::Request  fk_request;
    kinematics_msgs::GetPositionFK::Response fk_response;

    bool success = false;

    fk_request.header.frame_id = "torso_link";
    fk_request.fk_link_names.resize(1);
    fk_request.fk_link_names[0] = "wrist_roll_link";

    fk_request.robot_state.joint_state.position.resize(fk_solver_response_.kinematic_solver_info.joint_names.size());
    fk_request.robot_state.joint_state.name = fk_solver_response_.kinematic_solver_info.joint_names;

    double last_angles[7];
    getCurrentJointAngles(last_angles);

    for(unsigned int i=0; i< fk_solver_response_.kinematic_solver_info.joint_names.size(); i++)
    {
        fk_request.robot_state.joint_state.position[i] = last_angles[i];
        std::cout << "::: " << fk_solver_response_.kinematic_solver_info.joint_names.at(i) << std::endl;
    }

    if(fk_client.call(fk_request, fk_response))
    {
      if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
      {
        for(unsigned int i=0; i < fk_response.pose_stamped.size(); i ++)
        {
//          ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[i].c_str());
//          ROS_INFO_STREAM("Position: " <<
//            fk_response.pose_stamped[i].pose.position.x << "," <<
//            fk_response.pose_stamped[i].pose.position.y << "," <<
//            fk_response.pose_stamped[i].pose.position.z);
//          ROS_INFO("Orientation: %f %f %f %f",
//            fk_response.pose_stamped[i].pose.orientation.x,
//            fk_response.pose_stamped[i].pose.orientation.y,
//            fk_response.pose_stamped[i].pose.orientation.z,
//            fk_response.pose_stamped[i].pose.orientation.w);

          pose.position.x = fk_response.pose_stamped[i].pose.position.x;
          pose.position.y = fk_response.pose_stamped[i].pose.position.y;
          pose.position.z = fk_response.pose_stamped[i].pose.position.z;

          pose.orientation.x = fk_response.pose_stamped[i].pose.orientation.x;
          pose.orientation.y = fk_response.pose_stamped[i].pose.orientation.y;
          pose.orientation.z = fk_response.pose_stamped[i].pose.orientation.z;
          pose.orientation.w = fk_response.pose_stamped[i].pose.orientation.w;

          success = true;
        }
      }
      else
      {
        ROS_ERROR("Forward kinematics failed");
      }
    }
    else
    {
      ROS_ERROR("Forward kinematics service call failed");
    }
    return success;
}

// Check if the target is pose is reachable.
bool IKHelper::isReachable(geometry_msgs::Pose &pose)
{
    arm_test_gui::ExecuteCartesianIKTrajectory::Request req;
    req.header.frame_id = "torso_link";
    req.poses.push_back(pose);

    geometry_msgs::PoseStamped stamped_pose;
    stamped_pose.header = req.header;
    stamped_pose.header.stamp = ros::Time::now();
    bool success;

    double last_angles[7];
    getCurrentJointAngles(last_angles);

    for(int i=0; i<1 /*trajectory_length*/; i++)
    {
        stamped_pose.pose = req.poses[i];

        trajectory_array_ptr trajectory_array(new double[7]);

        success = getIKSolution(stamped_pose, last_angles, trajectory_array, "wrist_roll_link");

        if (success)
        {
            return true;
        }
    }
    return false;
}


ros::NodeHandle *IKHelper::getNodeHandle()
{
    if (nodeHandle == (ros::NodeHandle *)0)
        nodeHandle = new ros::NodeHandle();

    return nodeHandle;
}
