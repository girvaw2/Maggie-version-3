#ifndef MOTOR_H
#define MOTOR_H
#include "motor.h"
#endif

class Servo : public Motor
{
public:

  Servo(byte ID);
  ~Servo(){};
  
  void sendData(LList<byte> *inputData);
  void getResponse();

private:
  static const int timeout = 100;
};
