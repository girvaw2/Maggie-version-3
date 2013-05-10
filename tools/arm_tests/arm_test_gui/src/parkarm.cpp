#include "arm_test_gui/parkarm.h"

ParkArm::ParkArm()
{
    nodeHandle = (ros::NodeHandle *)0;
}

void ParkArm::park()
{
    setVelocity("/wrist_roll_controller", 0.5);
    setVelocity("/wrist_tilt_controller", 0.5);
    setVelocity("/forearm_roll_controller", 0.5);
    setVelocity("/elbow_tilt_controller", 0.25);
    setVelocity("/upper_arm_roll_controller", 0.25);
    setVelocity("/shoulder_tilt_controller", 0.25);
    setVelocity("/shoulder_pan_controller", 0.25);

    publish("/wrist_roll_controller/command",0);
    publish("/wrist_tilt_controller/command",0.1);
    publish("/forearm_roll_controller/command",0);
    publish("/elbow_tilt_controller/command",0);
    publish("/upper_arm_roll_controller/command",0);
    publish("/shoulder_tilt_controller/command",-0.5);
    publish("/shoulder_pan_controller/command",1.53);
}

void ParkArm::setVelocity(std::string controller, double velocity)
{
    ros::ServiceClient client = getNodeHandle()->serviceClient<dynamixel_hardware_interface::SetVelocity>(controller + std::string("/set_velocity"));
    dynamixel_hardware_interface::SetVelocity srv;
    srv.request.velocity = velocity;
    client.call(srv);
}

void ParkArm::publish(std::string topic, double position)
{
  ros::Publisher pub = getNodeHandle()->advertise<std_msgs::Float64>(topic, 1000);

  ros::Rate r(10);

  int count = 10;
  while (count--)
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Float64 msg;
    msg.data = position;

    pub.publish(msg);
    ros::spinOnce();
    r.sleep();
  }
}

ros::NodeHandle *ParkArm::getNodeHandle()
{
    if (nodeHandle == (ros::NodeHandle *)0)
        nodeHandle = new ros::NodeHandle();

  return nodeHandle;
}
