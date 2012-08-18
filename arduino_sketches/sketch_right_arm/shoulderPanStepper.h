#ifndef MOTOR_H
#define MOTOR_H
#include "motor.h"
#endif

#ifndef STEPPER_H
#define STEPPER_H
#include "stepper.h"
#endif

#ifndef LLIST_H
#define LLIST_H
#include "llist.h"
#endif

class ShoulderPanStepper : public Stepper
{
public:

  ShoulderPanStepper(byte ID);
  ~ShoulderPanStepper(){};
  
  void sendData(LList<byte> *inputData);
  void getResponse();
  void doOperation(byte operation, LList<byte> *inputData);
  void incrementPosition();
  void moveToHomePosition();
  void moveToReferencePosition();

private: // methods
  void extractInputPacketField(byte field, byte index);
  void setResponsePacket();
  void addResponsePacketParameters();
  byte calcCheckSum();
  void initResponsePacket(); 
  int convertFromStepsToDynamixelUnits(int steps);
    int convertFromDynamixelUnitsToSteps(int dynamixelUnits);
  void reset();

  void enableTimer();
  void disableTimer();
  void setTargetSpeed(int dynamixelTargetSpeed);
  void moveToGoalPosition();

private: // variables  
  byte clockPin;
  byte enablePin;
  byte directionPin;

  byte instruction;
  byte inLength;
  byte outLength;
  LList<byte> responsePacket;
  LList<byte> inParameters;
  
  int centrePositionInDynamixelUnits;
  int centrePositionInSteps;
  int referencePositionFromCentreInSteps;
  
  int timerSpeedCount;
  int goalPosition; // in dynamixel units relative to zero (CCW) osition 
  volatile int currentPosition; // in steps relative to zero (CCW) position.
  volatile int stepsToDo;
  volatile bool stepBit;
  volatile bool clockwise;
  volatile bool isMoving;
  
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

