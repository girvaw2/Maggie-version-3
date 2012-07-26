#ifndef MOTOR_H
#define MOTOR_H
#include "motor.h"
#endif

#ifndef LLIST_H
#define LLIST_H
#include "llist.h"
#endif

#define SHOULDER_PAN_MOTOR    3
#define SHOULDER_TILT_MOTOR   4
#define UPPER_ARM_ROLL_MOTOR  5

class Stepper : public Motor
{
public:

  Stepper(byte ID);
  ~Stepper(){};
  
  virtual void sendData(LList<byte> *inputData);
  virtual void getResponse();
  virtual void doOperation(byte operation, LList<byte> *inputData);
  virtual void incrementPosition();
  //void incrementUpperArmRollPosition();
  virtual void moveToHomePosition();
  virtual void moveToReferencePosition();

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
    //...
    returnDelayTime = 5,
    cwAngleLimit_L,
    cwAngleLimit_H,
    ccwAngleLimit_L,
    ccwAngleLimit_H,
    //...
    limitVoltage_L = 12,
    limitVoltage_H,
    //...
    goalPosition_L = 30,
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

