/*
 * maggie_kinematics_plugin.cpp
 * based on the pr2_arm_kinematics_plugin.cpp by Sachin Chitta 
 * and the katana implementation by Henning Deeken  // hdeeken@uos.de 
 */

#include <maggie_kinematics_constraint_aware/maggie_openrave_kinematics.h>
#include <pluginlib/class_list_macros.h>

using namespace tf;
using namespace kinematics;
using namespace std;
using namespace ros;

// register MaggieKinematics as a KinematicsBase implementation
PLUGINLIB_DECLARE_CLASS(maggie_kinematics_constraint_aware, MaggieKinematicsPlugin, maggie_kinematics_constraint_aware::MaggieKinematicsPlugin, kinematics::KinematicsBase)

namespace maggie_kinematics_constraint_aware
{

MaggieKinematicsPlugin::MaggieKinematicsPlugin() :
  active_(false)
{
}

bool MaggieKinematicsPlugin::isActive()
{
  if (active_)
    return true;
  return false;
}

bool MaggieKinematicsPlugin::initialize(std::string name)
{
  urdf::Model robot_model;
  std::string tip_name, xml_string;
  ros::NodeHandle private_handle("~/" + name);

  dimension_ = 5;

  while (!arm_kinematics_constraint_aware::loadRobotModel(private_handle, robot_model, root_name_, tip_name, xml_string)
      && private_handle.ok())
  {
    ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
    ros::Duration(0.5).sleep();
  }

  kinematics_msgs::KinematicSolverInfo kinematic_info;

  if (!arm_kinematics_constraint_aware::getChainInfoFromRobotModel(robot_model, root_name_, tip_name, kinematic_info))
  {
    ROS_FATAL("Could not get chain info!");
  }

  // connecting to services
  std::string fk_service;
  private_handle.param<std::string> ("fk_service", fk_service, "get_fk");
  fk_service_ = node_handle_.serviceClient<kinematics_msgs::GetPositionFK> (fk_service);

  std::string fk_info;
  private_handle.param<std::string> ("fk_info", fk_info, "get_fk_solver_info");
  fk_solver_info_service_ = node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo> (fk_info);

  std::string ik_service;
  private_handle.param<std::string> ("ik_service", ik_service, "IK");
  ik_service_ = node_handle_.serviceClient<orrosplanning::IK> (ik_service);

  if (!ros::service::waitForService(fk_info))
  {
    ROS_ERROR("Could not load fk info");
    active_ = false;
  }

  if (!ros::service::waitForService(fk_service))
  {
    ROS_ERROR("Could not load fk");
    active_ = false;
  }

  if (!ros::service::waitForService(ik_service))
  {
    ROS_ERROR("Could not load ik");
    active_ = false;
  }
  else
  {
    fk_solver_info_ = kinematic_info;
    ik_solver_info_ = fk_solver_info_;

    for (unsigned int i = 0; i < fk_solver_info_.joint_names.size(); i++)
    {
      ROS_INFO("MaggieKinematics:: joint name: %s",fk_solver_info_.joint_names[i].c_str());
    }
    for (unsigned int i = 0; i < ik_solver_info_.link_names.size(); i++)
    {
      ROS_INFO("MaggieKinematics can solve IK for %s",ik_solver_info_.link_names[i].c_str());
    }
    for (unsigned int i = 0; i < fk_solver_info_.link_names.size(); i++)
    {
      ROS_INFO("MaggieKinematics can solve FK for %s",fk_solver_info_.link_names[i].c_str());
    }
    ROS_INFO("MaggieKinematicsPlugin::active for %s",name.c_str());
    active_ = true;
  }

  ROS_DEBUG("Initializing the MaggieKinematicsPlugin was successful.");

  return active_;
}

bool MaggieKinematicsPlugin::initialize(const std::string& group_name,
                          const std::string& base_name,
                          const std::string& tip_name,
                          const double& search_discretization)
{
  return false;
}

bool MaggieKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state, std::vector<double> &solution,
                                           int &error_code)
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = kinematics::SUCCESS;
    return false;
  }

  ROS_DEBUG("Call getPositionIK()...");

  // set up the OpenRave IK request
  orrosplanning::IK srv;
  srv.request.manip_name = "arm";
  srv.request.joint_state.header.frame_id = root_name_;
  srv.request.joint_state.position = ik_seed_state;
  srv.request.pose_stamped.pose = ik_pose;
  srv.request.pose_stamped.header.frame_id = root_name_;
  srv.request.pose_stamped.header.stamp = ros::Time::now();
  srv.request.iktype = "TranslationDirection5D";
  srv.request.filteroptions = 0;

  ik_service_.call(srv);

  if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
  {
    if (srv.response.solutions.points.size() >= 1)
    {
      ROS_DEBUG("OpenRave IK found %d solutions,", srv.response.solutions.points.size());
      ROS_DEBUG("in cases of several solutions we discard all despite the first one");
    }

    solution.resize(dimension_);
    solution = srv.response.solutions.points[0].positions;
    error_code = kinematics::SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");
    error_code = kinematics::NO_IK_SOLUTION;
    return false;
  }
}

bool MaggieKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state, const double &timeout,
                                              std::vector<double> &solution, int &error_code)
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = -8; //kinematics::INACTIVE;
    return false;
  }

  ROS_DEBUG("Call searchPositionIK() without Callback Functions...");

  // set up the OpenRave IK request
  orrosplanning::IK srv;
  srv.request.manip_name = "arm";
  srv.request.joint_state.header.frame_id = root_name_;
  srv.request.joint_state.position = ik_seed_state;
  srv.request.pose_stamped.pose = ik_pose;
  srv.request.pose_stamped.header.frame_id = root_name_;
  srv.request.pose_stamped.header.stamp = ros::Time::now();
  srv.request.iktype = "TranslationDirection5D";
  srv.request.filteroptions = 0;

  ik_service_.call(srv);

  if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
  {
    if (srv.response.solutions.points.size() >= 1)
    {
      ROS_DEBUG("OpenRave IK found %d solutions,", srv.response.solutions.points.size());
      ROS_DEBUG("in cases of several solutions we discard all despite the first one");
    }

    solution.resize(dimension_);
    solution = srv.response.solutions.points[0].positions;
    error_code = kinematics::SUCCESS;

    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");
    error_code = kinematics::NO_IK_SOLUTION;
    return false;
  }
}

void MaggieKinematicsPlugin::desiredPoseCallback(const std::vector<double>& ik_seed_state,
                                                 const geometry_msgs::Pose& ik_pose,
                                                 motion_planning_msgs::ArmNavigationErrorCodes& error_code)
{

  int int_error_code;

  desiredPoseCallback_(ik_pose, ik_seed_state, int_error_code);

  if (int_error_code)
    error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::SUCCESS;
  else
    error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::NO_IK_SOLUTION;
}

void MaggieKinematicsPlugin::jointSolutionCallback(const std::vector<double>& solution,
                                                   const geometry_msgs::Pose& ik_pose,
                                                   motion_planning_msgs::ArmNavigationErrorCodes& error_code)
{
  int int_error_code;

  solutionCallback_(ik_pose, solution, int_error_code);

  if (int_error_code > 0)
    error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::SUCCESS;
  else
    error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::NO_IK_SOLUTION;
}

bool MaggieKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                const double &timeout,
                                const unsigned int& redundancy,
                                const double &consistency_limit,
                                std::vector<double> &solution,
                                int &error_code)
  {
    return false;
  }

bool MaggieKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state, 
					      const double &timeout,
                                              std::vector<double> &solution,
                                              const boost::function<void(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_solution, int &error_code)> &desired_pose_callback,
                                              const boost::function<void(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_solution, int &error_code)> &solution_callback,
                                              int &error_code_int)
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code_int = -8; // kinematics::INACTIVE;
    return false;
  }

  ROS_DEBUG("Call searchPositionIK() with Callback Functions...");

  desiredPoseCallback_ = desired_pose_callback;
  solutionCallback_ = solution_callback;

  motion_planning_msgs::ArmNavigationErrorCodes error_code;

  // perform IK and check for callback suitability

  if (!desired_pose_callback.empty())
    desiredPoseCallback(ik_seed_state, ik_pose, error_code);

  if (error_code.val != error_code.SUCCESS)
  {
    ROS_DEBUG("An IK solution could not be found, because the constraints in desired_pose_callback are not matched");
    error_code_int = kinematics::NO_IK_SOLUTION;
    return false;
  }

  std::vector<double> solution_;

  // set up the OpenRave IK request
  orrosplanning::IK srv;
  srv.request.manip_name = "arm";
  srv.request.joint_state.header.frame_id = root_name_;
  srv.request.joint_state.position = ik_seed_state;
  srv.request.pose_stamped.header.frame_id = root_name_;
  srv.request.pose_stamped.header.stamp = ros::Time::now();
  srv.request.iktype = "TranslationDirection5D";
  srv.request.filteroptions = 0;

  srv.request.pose_stamped.pose = ik_pose;

  ik_service_.call(srv);

  ROS_DEBUG("OpenRave Result %d", srv.response.error_code.val);

  if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
  {
    if (srv.response.solutions.points.size() >= 1)
    {
      ROS_DEBUG("OpenRave IK found %d solutions", srv.response.solutions.points.size());
    }
    solution_.resize(dimension_);
    solution_ = srv.response.solutions.points[0].positions;
  }

  bool callback_check = true;

  if (solution_callback.empty())
    callback_check = false;

  if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
  {
    if (callback_check)
    {
      jointSolutionCallback(solution_, ik_pose, error_code);

      if (error_code.val == error_code.SUCCESS)
      {
        solution.resize(dimension_);
        solution = solution_;
        error_code_int = kinematics::SUCCESS;
        ROS_DEBUG("Plugin sPIK w CB: found ik solution & solution_callback ok");
        return true;
      }
      else
      {
        ROS_DEBUG("Plugin sPIK w CB: found IK solution but solution call back fails");
        error_code_int = kinematics::NO_IK_SOLUTION;
        return false;
      }

    }
    else
    {
      error_code.val = error_code.SUCCESS;
      solution.resize(dimension_);
      solution = solution_;
      error_code_int = kinematics::SUCCESS;
      ROS_DEBUG("Plugin sPIK w CB: found ik solution & solution call back not necessary");
      return true;
    }
  }
  else
  {
    ROS_DEBUG("Plugin sPIK w CB: An IK solution could not be found");
    error_code_int = kinematics::NO_IK_SOLUTION;
    return false;
  }

  return false;
}

bool MaggieKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                const double &timeout,
                                const unsigned int& redundancy,
                                const double &consistency_limit,
                                std::vector<double> &solution,
                                const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                                const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
                                int &error_code)
{
  return searchPositionIK(ik_pose,
                   ik_seed_state,
                   timeout,
                   solution,
                   desired_pose_callback,
                   solution_callback,
                   error_code);
}

bool MaggieKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses)
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    return false;
  }

  ROS_DEBUG("Plugin: Call getPositionFK()...");

  kinematics_msgs::GetPositionFK srv;

  srv.request.header.frame_id = root_name_;
  srv.request.fk_link_names = link_names;
  srv.request.robot_state.joint_state.name = fk_solver_info_.joint_names;
  srv.request.robot_state.joint_state.position = joint_angles;

  fk_service_.call(srv);

  poses.resize(link_names.size());

  if (srv.response.error_code.val == srv.response.error_code.NO_FK_SOLUTION)
  {
    ROS_DEBUG("Plugin: Could not find a FK");
    return false;
  }

  if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
  {
    ROS_DEBUG("Successfully computed FK...");

    for (size_t i = 0; i < poses.size(); i++)
    {
      poses[i] = srv.response.pose_stamped[i].pose;
      ROS_DEBUG("PLUGIN Joint: %s Pose: %f %f %f // %f %f %f %f", link_names[i].c_str(),
          poses[i].position.x,
          poses[i].position.y,
          poses[i].position.z,
          poses[i].orientation.x,
          poses[i].orientation.y,
          poses[i].orientation.z,
          poses[i].orientation.w);
    }

    return true;
  }
  else
  {

    ROS_DEBUG("Plugin: Could not compute FK");

    return false;
  }
}

std::string MaggieKinematicsPlugin::getBaseFrame()
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    return std::string("");
  }
  return root_name_;
}

std::string MaggieKinematicsPlugin::getToolFrame()
{
  if (!active_ || ik_solver_info_.link_names.empty())
  {
    ROS_ERROR("kinematics not active");
    return std::string("");
  }

  return ik_solver_info_.link_names[0];
}

const std::vector<std::string>& MaggieKinematicsPlugin::getJointNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return ik_solver_info_.joint_names;
}

const std::vector<std::string>& MaggieKinematicsPlugin::getLinkNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }

  return fk_solver_info_.link_names;
}

}
// namespace
