#include <stdlib.h>

#ifndef STEPPER_H
#define STEPPER_H
#include "stepper.h"
#endif

#ifndef SHOULDER_PAN_STEPPER_H
#define SHOULDER_PAN_STEPPER_H
#include "shoulderPanStepper.h"
#endif

#ifndef SHOULDER_TILT_STEPPER_H
#define SHOULDER_TILT_STEPPER_H
#include "shoulderTiltStepper.h"
#endif

#ifndef SERVO_H
#define SERVO_H

#include "servo.h"
#endif

#define TIME_OUT          10
#define BROADCAST_ID      0xFE

void updateMotors(Motor *motor, LList<byte> *inputData);
Motor *addMotor(byte motorID);
Motor *getMotor(byte motorID);

bool isServoMotor(byte motorID);
bool isStepperMotor(byte motorID);
bool isBroadcastMotor(byte motorID);

void *operator new(size_t size);
void operator delete(void *ptr);

LList<Motor *> motorList;

void setup()
{ 
  cli();
  sei();
  
  pinMode(SERVO_TOGGLE_PIN, OUTPUT);
  digitalWrite(SERVO_TOGGLE_PIN, HIGH);

  initMotorPositions();
  
  pinMode(13, OUTPUT); // led pin
  digitalWrite(13, HIGH); // lED

  Serial.begin (1000000);
  Serial1.begin(1000000); 
}

void loop()
{
  byte length = 0;
  byte inByte;

  LList<byte>  inputList;
     
  Motor *motor;
    
  digitalWrite(SERVO_TOGGLE_PIN, HIGH);
  
  for (byte byteCount = 0; Serial.available(); byteCount++)
  {
    inByte = Serial.read();
    
    if (byteCount == 2)
    {
      motor = addMotor(inByte);
    }
    
    if (byteCount == 3)
    {
      length = inByte;
    }
      
    inputList.push_back(inByte);    
    
    if (byteCount > (length + 2))
    { 
      updateMotors(motor, &inputList);
      break;
    }
  }
}

void updateMotors(Motor *motor, LList<byte> *inputData)
{
   if (isBroadcastMotor(motor->getID()))
   {
       /*
        * We're dealing with a broadcast ID, so firstly, we need to find out what the instruction is... 
        */
        byte instructionLength = inputData->getElement(3);
        byte instruction = inputData->getElement(4);
  
        switch (instruction)
        {
          case 0x83: // sync write
  
            byte operation = inputData->getElement(5);
            byte dataLength = inputData->getElement(6);

            for (int i = 7; i < instructionLength + 3; i += dataLength + 1)
            {
              if (isStepperMotor(inputData->getElement(i)))
              {
                Motor *stepper = addMotor(inputData->getElement(i));
                
                LList<byte>  operationList;
                operationList.push_back(inputData->getElement(i+1)); 
                operationList.push_back(inputData->getElement(i+2)); 
                stepper->doOperation(operation, &operationList);
              }
            }
            break;
        }
        
        /*
         * Finish by broadcasting to the servo motors.
         */
        motor->sendData(inputData);
        // no response required for broadcast message.
        return;
    }
     

   motor->sendData(inputData);
   motor->getResponse();
}

Motor *addMotor(byte motorID)
{
  Motor *motor = getMotor(motorID);

  if (motor == NULL)
  {
    if (isServoMotor(motorID))
      motor = new Servo(motorID);
    else 
    if (isStepperMotor(motorID))
    {
      if (motorID == SHOULDER_PAN_MOTOR)
        motor = new ShoulderPanStepper(motorID);
      else
      if (motorID == SHOULDER_TILT_MOTOR)
        motor = new ShoulderTiltStepper(motorID);
    }
    else 
    if (isBroadcastMotor(motorID))
    {
      /*
       * We'll add a Servo motor for the moment, but we'll need to disect the instruction packet later in order to determine if there are any Steppers included in the broadcast...
       */
       motor = new Servo(BROADCAST_ID);
    }
      
    motorList.push_back(motor);
  }
  
  return motor;
}

Motor *getMotor(byte motorID)
{
  /*
   * A Linked List is far from ideal here - we really need a hashmap.
   * However, I haven't found a decent HashMap implementation for the Arduino yet,
   * so we'll use a Linked List implementation so long...
   */
  for (byte i=0; i<motorList.size(); i++)
  {
    Motor *motor = motorList.getElement(i);
    if (motor->getID() == motorID)
    {
      return motor;
    }
  }
  
  return NULL;
}

bool isServoMotor(byte motorID)
{
  //return (motorID < 3) || (motorID > 5);
  return !isStepperMotor(motorID);
}

bool isStepperMotor(byte motorID)
{
  return false; //(motorID == SHOULDER_TILT_MOTOR); 
}

bool isBroadcastMotor(byte motorID)
{
  return (motorID == BROADCAST_ID);
}

void initMotorPositions()
{
  /*
   * When the Arduino is reset, the Stepper motors will not know their positions.
   * We therefore need need to drive each Stepper motor to its reference position
   * before returning it to its home position.
   */
   
   Stepper *stepper = (Stepper *)addMotor(SHOULDER_PAN_MOTOR);
   //stepper->moveToReferencePosition();
   
   stepper = (Stepper *)addMotor(SHOULDER_TILT_MOTOR);
   //stepper->moveToReferencePosition();
}

ISR(TIMER3_COMPA_vect) 
{
  /*
   * We're in TIMER3's OC interrupt A, so get the shoulder pan's motor object, and start it moving
   */
   
  ((ShoulderPanStepper*)getMotor(SHOULDER_PAN_MOTOR))->incrementPosition();
}

ISR(TIMER4_COMPA_vect) 
{
  /*
   * We're in TIMER4's OC interrupt A, so get motor 4's object, and start it moving
   */
   
  ((ShoulderTiltStepper*)getMotor(SHOULDER_TILT_MOTOR))->incrementPosition();
}

void shoulderPanLimitReached()
{
  ((ShoulderPanStepper*)getMotor(SHOULDER_PAN_MOTOR))->moveToHomePosition();
}

void shoulderTiltLimitReached()
{
  digitalWrite(13, LOW); // lED01
  ((ShoulderTiltStepper*)getMotor(SHOULDER_TILT_MOTOR))->moveToHomePosition();
}

void *operator new(size_t size) { return malloc(size); }
void operator delete(void *ptr) { free(ptr); }





