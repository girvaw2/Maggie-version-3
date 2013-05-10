#ifndef PARKARM_H
#define PARKARM_H

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "dynamixel_hardware_interface/JointState.h"
#include "dynamixel_hardware_interface/SetVelocity.h"

class ParkArm
{
public:
    ParkArm();
    void park();

private:
    void initialiseVelocity();

    void subscribe();
    void publish(std::string topic, double position);
    ros::NodeHandle *getNodeHandle();

private:
    ros::NodeHandle *nodeHandle;
};

#endif // PARKARM_H
