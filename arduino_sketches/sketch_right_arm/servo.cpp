#include "servo.h"


Servo::Servo (byte ID) : Motor (ID)
{
}

void Servo::sendData(LList<byte> *inputData)
{
  while (!(inputData->isEmpty()))
    Serial1.write(inputData->pop_front());
}

void Servo::getResponse()
{
  unsigned int time_counter = 0;
  
  delayMicroseconds(400);
  digitalWrite(SERVO_TOGGLE_PIN, LOW); //read
  
  while(Serial1.available() || time_counter < timeout)
  {  // Wait for Data
    time_counter++;
    delay(1);
    if( ((unsigned int)Serial1.peek()) == 255 )
    {
      break;
    }
    Serial1.read(); // read the invalid char...
  }
  
  while (Serial1.available())
  {
    Serial.write(Serial1.read());
  }
}
