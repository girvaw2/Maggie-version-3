#include "stepper.h"

/*
 * Instruction codes
 */
#define PING           0x01
#define READ_DATA      0x02  

extern void shoulderPanLimitReached();
extern void shoulderTiltLimitReached();

Stepper::Stepper(byte ID) : Motor (ID)
{
//  isMoving = false;
//  stepsToDo = 0;
//  stepBit = true;
//  
//  if (ID == SHOULDER_PAN_MOTOR)
//  {
//    clockPin = 22;
//    enablePin = 24;
//    directionPin = 26;
//    
//    centrePositionInDynamixelUnits = 512;
//    centrePositionInSteps = 432; // 50 degrees. 
//    referencePositionFromCentreInSteps = 740; 
//  }
//  else
//  if (ID == SHOULDER_TILT_MOTOR)
//  {
//    clockPin = 28;
//    enablePin = 30;
//    directionPin = 32;
//    
//    //Disable enable pin for the moment - active high
//    //PORTC &= 0x7F;
//    PORTC |= 0x80;
//    
//    centrePositionInDynamixelUnits = 512;
//    centrePositionInSteps = 432;  
//    referencePositionFromCentreInSteps = 450; 
//  }
//  else
//  if (ID == UPPER_ARM_ROLL_MOTOR)
//  {
//    clockPin = 34;
//    enablePin = 36;
//    directionPin = 38;
//    
//    //Disable enable pin for the moment - active high
//    PORTC &= 0xFD;
//    
//    centrePositionInDynamixelUnits = 512;
//    centrePositionInSteps = 432; // 50 degrees. 
//    referencePositionFromCentreInSteps = 740; 
//  } 
//  
//  goalPosition = centrePositionInDynamixelUnits;
//  currentPosition = centrePositionInSteps;
//  
//  pinMode(clockPin, OUTPUT);
//  pinMode(enablePin, OUTPUT);
//  pinMode(directionPin, OUTPUT);
}

void Stepper::sendData(LList<byte> *inputData)
{
    byte i = 0;
    while (!(inputData->isEmpty()))
    {
      extractInputPacketField(inputData->pop_front(), i++);
    }
}

void Stepper::getResponse()
{
  setResponsePacket();
  
  for (byte i=0; i<responsePacket.size(); i++)
  {
    Serial.write(responsePacket.getElement(i));
  }
  
  reset();
}

void Stepper::doOperation(byte operation, LList<byte> *inputData)
{
  switch (operation)
  {
    case 0x1E: // goal position
      goalPosition = inputData->getElement(8) + (256 * inputData->getElement(9));
      moveToGoalPosition();
      break;
  }
  
}

void Stepper::extractInputPacketField(byte field, byte index)
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

void Stepper::initResponsePacket()
{
  responsePacket.push_back(0xFF);
  responsePacket.push_back(0xFF);
  responsePacket.push_back(getID());
  responsePacket.push_back(outLength);
  responsePacket.push_back(0x00); //error code
}

void Stepper::setResponsePacket()
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

void Stepper::addResponsePacketParameters()
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

byte Stepper::calcCheckSum()
{
  unsigned int parameterSum = 0;
  
  for (byte i=2; i<outLength; i++)
  {
    parameterSum += responsePacket.getElement(i+3);
  }
  
  unsigned int checkSum = getID() + outLength + parameterSum;
  
  return 255 - ( (getID() + outLength + parameterSum) % 256 );
}

void Stepper::reset()
{
    inLength = outLength = 0;
    inParameters.clear();
    responsePacket.clear();
}

void Stepper::moveToGoalPosition()
{
  /*
   * The goal position is relative to the centre position.It is not an absolute distance from the current position.
   * We therefore need to work out where we are relative to the centre position, then move the motor to that position.
   */
//  int goalPositionInSteps = convertFromDynamixelUnitsToSteps(goalPosition - centrePositionInDynamixelUnits) + centrePositionInSteps;
//  
//  clockwise = currentPosition < goalPositionInSteps;
  
  /*
   * The shoulder pan stepper is now upside down, so CW for it is CCW for the shoulder turret!
   * Swap the direction for the shoulder pan Stepper therefore.
   */
//  if (getID() == SHOULDER_PAN_MOTOR)
//  {
//    digitalWrite(directionPin, !clockwise ); 
//  }
//  else
//  {
//    digitalWrite(directionPin, !clockwise ); 
//  }
//     
//  stepsToDo =  abs(currentPosition - goalPositionInSteps);
//  
//  if (stepsToDo)
//  {
//    isMoving = true;
//    enableTimer();
//  }
}

int Stepper::convertFromStepsToDynamixelUnits(int steps)
{
  /* 
   * Each step is 0.1158 degrees.
   * Steps * 0.1158 * 1023 / 300
   */
  return (int)((float)steps * 0.39498);
}

int Stepper::convertFromDynamixelUnitsToSteps(int dynamixelUnits)
{
  return (int)((float)dynamixelUnits / 0.39498);
}

void Stepper::incrementPosition()
{
  /*
   * The stepper pulses on every rising edge of the clk pin, so the motor will step every 2 pulses
   */
//   stepBit = !stepBit;
//
//   /*
//    * Don't use a digital write on the clockPin here, as it causes the AVR to reset !!!
//    */ 
//   if (stepBit)
//   {
//     if (getID() == SHOULDER_PAN_MOTOR)
//       PORTA |= 0x01; // clockPin
//     else 
//     if (getID() == SHOULDER_TILT_MOTOR)
//       PORTA |= 0x40;
//     else
//     if (getID() == UPPER_ARM_ROLL_MOTOR)
//       PORTC |= 0x08;
     
//     if (clockwise)
//       currentPosition++;
//     else
//     {
//       currentPosition--;
//       
//       if (currentPosition < 0)
//         currentPosition = 0;
//     }
//   }
//   else
//   {
//    if (getID() == SHOULDER_PAN_MOTOR)
//      PORTA &= 0xFE;
//    else
//    if (getID() == SHOULDER_TILT_MOTOR)
//      PORTA &= 0xBF;
//    else
//    if (getID() == UPPER_ARM_ROLL_MOTOR)
//       PORTC &= 0xF7;
//   }
//   
//   if (stepBit && --stepsToDo == 0)
//   {
//     disableTimer();
//   }
}

//void Stepper::incrementUpperArmRollPosition()
//{
//  /*
//   * The stepper pulses on every rising edge of the clk pin, so the motor will step every 2 pulses
//   */
//   stepBit = !stepBit;
//
//   /*
//    * Don't use a digital write on the clockPin here, as it causes the AVR to reset !!!
//    */ 
//   if (stepBit)
//   {
//     PORTC |= 0x08;
//     
//     if (clockwise)
//       currentPosition++;
//     else
//     {
//       currentPosition--;
//       
//       if (currentPosition < 0)
//         currentPosition = 0;
//     }
//   }
//   else
//   {
//       PORTC &= 0xF7;
//   }
//   
//   if (stepBit && --stepsToDo == 0)
//   {
//     //disableTimer();
//    TCCR5B = 0;
//    TIMSK5 &= 0xFD;     // Disable timer 3 OC interrupt A
//    PORTC &= 0xFD;      // Disable enable pin - active HIGH
//    PORTC |= 0x08;      // Set the clock pin to HIGH
//    TCNT5 = 0;
//    isMoving = false;
//   }
//}

void Stepper::enableTimer()
{
//  if (getID() == SHOULDER_PAN_MOTOR)
//  {
//    PORTA |= 0x01; // Set the clock pin to HIGH
//    PORTA |= 0x04; // Set the enable pin to HIGH
//    
//    TCNT3 = 0;
//    
//    /*
//     * A precaler or 64 & a OCRF3A count of 250 equates to an angular velocity of approximately 1 rad/s
//     */    
//    TCCR3A = 0; // set entire TCCR3A register to 0
//    OCR3A = 250; // 1 ms pulse
//    TCCR3B = 0x0B; // start timer, 64 x prescaler, CTC mode.
//    bitWrite(TIMSK3, OCIE3A, 1); // Enable timer 3 OC interrupt A
//  }
//  else
//  if (getID() == SHOULDER_TILT_MOTOR)
//  {
//    PORTA |= 0x40; // Set the clock pin to HIGH
//    //PORTC |= 0x80; // Set the enable pin to HIGH
//    
//    TCNT4 = 0;
//    
//    /*
//     * A precaler or 64 & a OCRF3A count of 250 equates to an angular velocity of approximately 1 rad/s
//     */
//    TCCR4A = 0; // set entire TCCR4A register to 0
//    //OCR4A = 500; // 4 ms pulse
//    OCR4A = 250; // 2 ms pulse
//    TCCR4B = 0x0B; // start timer, 64 x prescaler, CTC mode.
//    bitWrite(TIMSK4, OCIE4A, 1); // Enable timer 4 OC interrupt A
//  }
//  else
//  if (getID() == UPPER_ARM_ROLL_MOTOR)
//  {
//    digitalWrite(13, LOW); // lED
//    
//    PORTC |= 0x08; // Set the clock pin to HIGH
//    PORTC |= 0x02; // Set the enable pin to HIGH
//    
//    TCNT5 = 0;
//    TCCR5A = 0; // set entire TCCR5A register to 0
//    OCR5A = 250; // 1 ms pulse
//    TCCR5B = 0x0B; // start timer, 64 x prescaler, CTC mode.
//    bitWrite(TIMSK5, OCIE5A, 1); // Enable timer 5 OC interrupt A
//  }
}

void Stepper::disableTimer()
{
//  if (getID() == SHOULDER_PAN_MOTOR)
//  {
//    TCCR3B = 0;
//    TIMSK3 &= 0xFD;     // Disable timer 3 OC interrupt A
//    PORTA &= 0xFB;      // Disable enable pin - active HIGH
//    PORTA |= 0x01;      // Set the clock pin to HIGH
//    TCNT3 = 0;
//    isMoving = false;
//  }
//  else
//  if (getID() == SHOULDER_TILT_MOTOR)
//  {
//    TCCR4B = 0;
//    TIMSK4 &= 0xFD;     // Disable timer 3 OC interrupt A
//    //PORTC &= 0x7F;      // Disable enable pin - active HIGH
//    PORTA |= 0x40;      // Set the clock pin to HIGH
//    TCNT4 = 0;
//    isMoving = false;
//  }
//  else
//  if (getID() == UPPER_ARM_ROLL_MOTOR)
//  {
//    TCCR5B = 0;
//    TIMSK5 &= 0xFD;     // Disable timer 3 OC interrupt A
//    PORTC &= 0xFD;      // Disable enable pin - active HIGH
//    PORTC |= 0x08;      // Set the clock pin to HIGH
//    TCNT5 = 0;
//    isMoving = false;
//  }
}

void Stepper::moveToHomePosition()
{
    /*
     * We should never hit this position in the normal operation of the motor - other than during a reset. 
     * 
     */
     
//    disableTimer(); // stop the motor.
    
//    if (getID() == SHOULDER_PAN_MOTOR)
//    {
//     /*
//      * For the shoulder pan motor, move the turret fully CCW.
//      * For this motor, the min (CCW) is 0, the centre is 733 and the max (CW) 1466
//      */
//      detachInterrupt(0);
//      PORTA |= 0x01; // Set the clock pin to HIGH
//      PORTA |= 0x04; // Set the enable pin to HIGH
//      PORTA |= 0x10; // Set the direction pin to HIGH (CW)
//      clockwise = false;
//    }
//    else
//    if (getID() == SHOULDER_TILT_MOTOR)
//    {
//      detachInterrupt(1);
//      PORTA |= 0x40; // Set the clock pin to HIGH
//      PORTC |= 0x80; // Set the enable pin to HIGH
//      PORTC |= 0x20; // Set the direction pint to HIGH (CW)  
//      clockwise = false;    
//    }  
    
//    stepsToDo = referencePositionFromCentreInSteps;
//    currentPosition = centrePositionInSteps + referencePositionFromCentreInSteps;
//    goalPosition = centrePositionInDynamixelUnits;
//    enableTimer();
}

void Stepper::moveToReferencePosition()
{
//    if (getID() == SHOULDER_PAN_MOTOR)
//    {
//      attachInterrupt(0, shoulderPanLimitReached, LOW);
//      PORTA |= 0x01; // Set the clock pin to HIGH
//      PORTA |= 0x04; // Set the enable pin to HIGH
//      PORTA &= 0xEF; // Set the direction pin to LOW (CCW)
//    }
//    else
//    if (getID() == SHOULDER_TILT_MOTOR)
//    {
//      attachInterrupt(1, shoulderTiltLimitReached, LOW);
//      PORTA |= 0x40; // Set the clock pin to HIGH
//      PORTC |= 0x80; // Set the enable pin to HIGH
//      PORTC &= 0xBF; // Set the direction pint to LOW (CCW)
//    }  
//    stepsToDo = 10000; // keep going until we hit the reference position!
//    enableTimer();
}


