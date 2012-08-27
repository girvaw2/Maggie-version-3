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

class ShoulderTiltStepper : public Stepper
{
public:

  ShoulderTiltStepper(byte ID);
  ~ShoulderTiltStepper(){};
  
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
};

