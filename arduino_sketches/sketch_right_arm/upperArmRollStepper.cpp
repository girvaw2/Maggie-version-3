#include "upperArmRollStepper.h"

/*
 * Instruction codes
 */
#define PING           0x01
#define READ_DATA      0x02  

extern void shoulderPanLimitReached();
extern void shoulderTiltLimitReached();

UpperArmRollStepper::UpperArmRollStepper(byte ID) : Stepper (ID)
{
  isMoving = false;
  stepsToDo = 0;
  stepBit = true;
  
  clockPin = 34;
  enablePin = 36;
  directionPin = 38;
  
  //Disable enable pin for the moment - active high
  PORTC &= 0xFD;
  
  centrePositionInDynamixelUnits = 512;
  centrePositionInSteps = 432; // 50 degrees. 
  referencePositionFromCentreInSteps = 740; 
  
  goalPosition = centrePositionInDynamixelUnits;
  currentPosition = centrePositionInSteps;
  
  pinMode(clockPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(directionPin, OUTPUT);
}

void UpperArmRollStepper::sendData(LList<byte> *inputData)
{
    byte i = 0;
    while (!(inputData->isEmpty()))
    {
      extractInputPacketField(inputData->pop_front(), i++);
    }
}

void UpperArmRollStepper::getResponse()
{
  setResponsePacket();
  
  for (byte i=0; i<responsePacket.size(); i++)
  {
    Serial.write(responsePacket.getElement(i));
  }
  
  reset();
}

void UpperArmRollStepper::doOperation(byte operation, LList<byte> *inputData)
{
  switch (operation)
  {
    case 0x1E: // goal position
      goalPosition = inputData->getElement(8) + (256 * inputData->getElement(9));
      moveToGoalPosition();
      break;
  }
  
}

void UpperArmRollStepper::extractInputPacketField(byte field, byte index)
{
  switch (index)
  {
    case 0:
    case 1:
    case 2: // do nothing with the ID, as we already have it!
      break;
      
    case 3:
      inLength = field;
      break;
      
    case 4:
      instruction = field;
      break;
 
    default: // parameters
      if (index < (inLength + 3))
          inParameters.push_back(field);
      break;
  }
}

void UpperArmRollStepper::initResponsePacket()
{
  responsePacket.push_back(0xFF);
  responsePacket.push_back(0xFF);
  responsePacket.push_back(getID());
  responsePacket.push_back(outLength);
  responsePacket.push_back(0x00); //error code
}

void UpperArmRollStepper::setResponsePacket()
{
  switch (instruction)
  {
    case PING:
      outLength = 0x02; // no parameters
      initResponsePacket();
      responsePacket.push_back(calcCheckSum());
      break;
     
     case READ_DATA:
       outLength = inParameters.getElement(1) + 0x02; //inParameters(1) holds the number of parameters to be returned.
       initResponsePacket();
       addResponsePacketParameters();
       responsePacket.push_back(calcCheckSum());
  }
}

void UpperArmRollStepper::addResponsePacketParameters()
{
  byte startAddress = inParameters.getElement(0);
  for (int i = startAddress; i < startAddress + outLength - 0x02; i++)
  {
    switch(i) 
    {
      case modelNumber_L:
        responsePacket.push_back(0x0C);
        break;
      case modelNumber_H:
        responsePacket.push_back(0x00);
        break;
      case firmwareVersion:
        responsePacket.push_back(0x01);
        break;
      case returnDelayTime:
        responsePacket.push_back(0xFA);
        break;
      case cwAngleLimit_L:
        responsePacket.push_back(0x00);
        break;
      case cwAngleLimit_H:
        responsePacket.push_back(0x00);
        break;
      case ccwAngleLimit_L:
        responsePacket.push_back(0xFF);
        break;
      case ccwAngleLimit_H:
        responsePacket.push_back(0x03);
        break;
      case limitVoltage_L:
        responsePacket.push_back(0x3C);
        break;
      case limitVoltage_H:
        responsePacket.push_back(0xBE);
        break;
      case goalPosition_L:
        responsePacket.push_back(goalPosition % 256);
        break;
      case goalPosition_H:
        responsePacket.push_back(goalPosition / 256);
        break;
      case movingSpeed_L:
        responsePacket.push_back(0x00);
        break;
      case movingSpeed_H:
        responsePacket.push_back(0x00);
        break;
      case torqueLimit_L:
        responsePacket.push_back(0xFF);
        break;
      case torqueLimit_H:
        responsePacket.push_back(0x03);
        break;
      case presentPosition_L:
        responsePacket.push_back ((centrePositionInDynamixelUnits + convertFromStepsToDynamixelUnits(currentPosition - centrePositionInSteps)) % 256);
        break;
      case presentPosition_H:
        responsePacket.push_back ((centrePositionInDynamixelUnits + convertFromStepsToDynamixelUnits(currentPosition - centrePositionInSteps)) / 256);
        break;
      case presentSpeed_L:
        responsePacket.push_back(0x00);
        break;
      case presentSpeed_H:
        responsePacket.push_back(0x00);
        break;
      case presentLoad_L:
        responsePacket.push_back(0x00);
        break;
      case presentLoad_H:
        responsePacket.push_back(0x00);
        break;
      case presentVoltage:
        responsePacket.push_back(30);
        break;
      case presentTemperature:
        responsePacket.push_back(20);
        break;
      case registeredInstruction:
        responsePacket.push_back(0x00);
        break;
      case reserved:
        responsePacket.push_back(0x00);
        break;
      case moving:
        responsePacket.push_back(isMoving ? 0x01 : 0x00);
        break;
        
      default:
        break;
    }
  }
}

byte UpperArmRollStepper::calcCheckSum()
{
  unsigned int parameterSum = 0;
  
  for (byte i=2; i<outLength; i++)
  {
    parameterSum += responsePacket.getElement(i+3);
  }
  
  unsigned int checkSum = getID() + outLength + parameterSum;
  
  return 255 - ( (getID() + outLength + parameterSum) % 256 );
}

void UpperArmRollStepper::reset()
{
    inLength = outLength = 0;
    inParameters.clear();
    responsePacket.clear();
}

void UpperArmRollStepper::moveToGoalPosition()
{
  /*
   * The goal position is relative to the centre position.It is not an absolute distance from the current position.
   * We therefore need to work out where we are relative to the centre position, then move the motor to that position.
   */
  int goalPositionInSteps = convertFromDynamixelUnitsToSteps(goalPosition - centrePositionInDynamixelUnits) + centrePositionInSteps;
  
  clockwise = currentPosition < goalPositionInSteps;
  
  digitalWrite(directionPin, !clockwise ); 
     
  stepsToDo =  abs(currentPosition - goalPositionInSteps);
  
  if (stepsToDo)
  {
    isMoving = true;
    enableTimer();
  }
}

int UpperArmRollStepper::convertFromStepsToDynamixelUnits(int steps)
{
  /* 
   * Each step is 0.1158 degrees.
   * Steps * 0.1158 * 1023 / 300
   */
  return (int)((float)steps * 0.39498);
}

int UpperArmRollStepper::convertFromDynamixelUnitsToSteps(int dynamixelUnits)
{
  return (int)((float)dynamixelUnits / 0.39498);
}

void UpperArmRollStepper::incrementPosition()
{
  /*
   * The stepper pulses on every rising edge of the clk pin, so the motor will step every 2 pulses
   */
   stepBit = !stepBit;

   /*
    * Don't use a digital write on the clockPin here, as it causes the AVR to reset !!!
    */ 
   if (stepBit)
   {
     PORTC |= 0x08;
     
     if (clockwise)
       currentPosition++;
     else
     {
       currentPosition--;
       
       if (currentPosition < 0)
         currentPosition = 0;
     }
   }
   else
   {
     PORTC &= 0xF7;
   }
   
   if (stepBit && --stepsToDo == 0)
   {
     disableTimer();
   }
}

void UpperArmRollStepper::enableTimer()
{
    digitalWrite(13, LOW); // lED
    
    PORTC |= 0x08; // Set the clock pin to HIGH
    PORTC |= 0x02; // Set the enable pin to HIGH
    
    TCNT5 = 0;
    TCCR5A = 0; // set entire TCCR5A register to 0
    OCR5A = 250; // 1 ms pulse
    TCCR5B = 0x0B; // start timer, 64 x prescaler, CTC mode.
    bitWrite(TIMSK5, OCIE5A, 1); // Enable timer 5 OC interrupt A
}

void UpperArmRollStepper::disableTimer()
{
    TCCR5B = 0;
    TIMSK5 &= 0xFD;     // Disable timer 3 OC interrupt A
    PORTC &= 0xFD;      // Disable enable pin - active HIGH
    PORTC |= 0x08;      // Set the clock pin to HIGH
    TCNT5 = 0;
    isMoving = false;
}

void UpperArmRollStepper::moveToHomePosition()
{
}

void UpperArmRollStepper::moveToReferencePosition()
{
}


