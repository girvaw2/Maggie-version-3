/*
 * non_constraint_aware_ik_adapter.h
 *
 * based upon the the katana implementation by Martin Günther <mguenthe@uos.de>
 */

#ifndef NON_CONSTRAINT_AWARE_IK_ADAPTER_H_
#define NON_CONSTRAINT_AWARE_IK_ADAPTER_H_

#include <ros/ros.h>

#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>

namespace maggie_kinematics_constraint_aware
{


class NonConstraintAwareIKAdapter
{
public:
  NonConstraintAwareIKAdapter();
  virtual ~NonConstraintAwareIKAdapter();
};

}

#endif /* NON_CONSTRAINT_AWARE_IK_ADAPTER_H_ */
