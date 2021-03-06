#ifndef MOTOR_H
#define MOTOR_H
#include "motor.h"
#endif

#ifndef LLIST_H
#define LLIST_H
#include "llist.h"
#endif

/*
 * Instruction codes
 */
#define PING           0x01
#define READ_DATA      0x02  
#define WRITE_DATA     0x03

#define SHOULDER_PAN_MOTOR    1
#define SHOULDER_TILT_MOTOR   2


#define GOAL_POSITION          0x1E
#define MOVING_SPEED           0x20

class Stepper : public Motor
{
public:

  Stepper(byte ID) : Motor(ID){};
  ~Stepper(){};
  
  virtual void sendData(LList<byte> *inputData);
  virtual void getResponse();
  virtual void doOperation(byte operation, LList<byte> *inputData);
  virtual void incrementPosition();
  virtual void moveToHomePosition();
  virtual void moveToReferencePosition();

protected:  
  enum ControlTable
  {
    modelNumber_L = 0,
    modelNumber_H,
    firmwareVersion,
    uniqueID,
    baudRate,  
    returnDelayTime, // 5
    cwAngleLimit_L,
    cwAngleLimit_H,
    ccwAngleLimit_L,
    ccwAngleLimit_H,
    // reserved
    limitTemperature_H = 11,
    limitVoltage_L,
    limitVoltage_H,
    maxTorque_L,
    maxTorque_H,
    statusReturnLevel,
    alarmLED,
    alarmShutdown,
    // reserved
    downCalibration_L = 20,
    downCalibration_H,
    upCalibration_L,
    upCalibration_H,
    torqueEnable,
    LED,
    cwComplianceMargin,
    ccwComplianceMargin,
    cwComplianceSlope,
    ccwComplianceSlope,
    goalPosition_L, // 30,
    goalPosition_H,
    movingSpeed_L,
    movingSpeed_H,
    torqueLimit_L,
    torqueLimit_H,
    presentPosition_L,
    presentPosition_H,
    presentSpeed_L,
    presentSpeed_H,
    presentLoad_L,
    presentLoad_H,
    presentVoltage,
    presentTemperature,
    registeredInstruction,
    reserved,
    moving
  };     
};

