#include "arm_test_gui/parkarm.h"

ParkArm::ParkArm()
{
    nodeHandle = (ros::NodeHandle *)0;
}

void chatterCallback(const dynamixel_hardware_interface::JointState::ConstPtr& msg)
{
  std::cout << "hello world" << std::endl;
}

void ParkArm::park()
{
  int argc;
  char ** argv = ((char **)0);
  ros::init(argc, argv, "listener");

  publish("/wrist_roll_controller/command",0);
  publish("/wrist_tilt_controller/command",0.1);
  publish("/forearm_roll_controller/command",0);
  publish("/elbow_tilt_controller/command",0);
  publish("/upper_arm_roll_controller/command",0);
  publish("/shoulder_tilt_controller/command",-0.5);
  publish("/shoulder_pan_controller/command",1.53);

}

void ParkArm::subscribe()
{
  ros::Subscriber sub = getNodeHandle()->subscribe("/wrist_roll_controller/state", 1000, chatterCallback);
  ros::spin();
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


//void ParkArm::publish()
//{
//  ros::Publisher wrist_roll_pub = getNodeHandle()->advertise<std_msgs::Float64>("/wrist_roll_controller/command", 1000);
//  ros::Publisher wrist_tilt_pub = getNodeHandle()->advertise<std_msgs::Float64>("/wrist_tilt_controller/command", 1000);

//  double position = 0;
//  while (ros::ok())
//  {
//    /**
//     * This is a message object. You stuff it with data, and then publish it.
//     */
//    std_msgs::Float64 msg;
//    msg.data = position;

//    wrist_roll_pub.publish(msg);
//    wrist_tilt_pub.publish(msg);



//    position += 0.1;

//    ros::spinOnce();

//    if (position > 1.1)
//      break;

//    ::sleep(1);
//  }
//}

ros::NodeHandle *ParkArm::getNodeHandle()
{
    if (nodeHandle == (ros::NodeHandle *)0)
        nodeHandle = new ros::NodeHandle();

  return nodeHandle;
}
