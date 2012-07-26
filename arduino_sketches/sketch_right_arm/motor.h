//#include "/usr/share/arduino/libraries/QueueList/QueueList.h"
//#include "/usr/share/arduino/libraries/stl/include/stl_vector.h"
//#include "/usr/share/arduino/libraries/HashMap/HashMap.h"

#ifndef LLIST_H
#define LLIST_H
#include "llist.h"
#endif

#include <WProgram.h>

#define SERVO_TOGGLE_PIN  11

class Motor
{
public:
  Motor(byte ID){motorID = ID;};
  ~Motor(){};
  
  virtual void home(){};
  //virtual bool isHome();
  //virtual void set_position(int position);
  //virtual void set_speed(int speed);
  
  virtual void sendData(LList<byte> *inputData) = 0;
  virtual void getResponse() = 0;
  virtual void doOperation(byte operation, LList<byte> *inputData){};
  
  //void extractMotorListFromBroadcast(LList<byte> *inputData, LList<byte> *motorList);
  
  byte getID(){return motorID;};
  

private:
  byte motorID;
};
