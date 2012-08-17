#include "shoulderTiltStepper.h"

extern void shoulderTiltLimitReached();

ShoulderTiltStepper::ShoulderTiltStepper(byte ID) : Stepper (ID)
{
  isMoving = false;
  stepsToDo = 0;
  stepBit = true;
  
  clockPin = 28;
  enablePin = 30;
  directionPin = 32;
  
  //Disable enable pin for the moment - active high
  //PORTC &= 0x7F;
  PORTC |= 0x80;
  
  centrePositionInDynamixelUnits = 512;
  centrePositionInSteps = 432;  
  referencePositionFromCentreInSteps = 450; 
  
  goalPosition = centrePositionInDynamixelUnits;
  currentPosition = centrePositionInSteps;
  
  pinMode(clockPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(directionPin, OUTPUT);
}

void ShoulderTiltStepper::sendData(LList<byte> *inputData)
{
    byte i = 0;
    while (!(inputData->isEmpty()))
    {
      extractInputPacketField(inputData->pop_front(), i++);
    }
}

void ShoulderTiltStepper::getResponse()
{
  setResponsePacket();
  
  for (byte i=0; i<responsePacket.size(); i++)
  {
    Serial.write(responsePacket.getElement(i));
  }
  
  reset();
}

void ShoulderTiltStepper::doOperation(byte operation, LList<byte> *inputData)
{
  switch (operation)
  {
    case 0x1E: // goal position
      goalPosition = inputData->getElement(8) + (256 * inputData->getElement(9));
      moveToGoalPosition();
      break;
  }
  
}

void ShoulderTiltStepper::extractInputPacketField(byte field, byte index)
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

void ShoulderTiltStepper::initResponsePacket()
{
  responsePacket.push_back(0xFF);
  responsePacket.push_back(0xFF);
  responsePacket.push_back(getID());
  responsePacket.push_back(outLength);
  responsePacket.push_back(0x00); //error code
}

void ShoulderTiltStepper::setResponsePacket()
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
       break;
     
     case WRITE_DATA:
       outLength = 0x02;
       initResponsePacket();
       responsePacket.push_back(calcCheckSum());
       break;
  }
}

void ShoulderTiltStepper::addResponsePacketParameters()
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
      case uniqueID:
        responsePacket.push_back(SHOULDER_TILT_MOTOR);
        break;
      case baudRate:
        responsePacket.push_back(0x01); // 1000000
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
      case limitTemperature_H:
        responsePacket.push_back(0x55);
        break;
      case limitVoltage_L:
        responsePacket.push_back(0x3C);
        break;
      case limitVoltage_H:
        responsePacket.push_back(0xBE);
        break;
      case maxTorque_L:
        responsePacket.push_back(0xFF);
        break;
      case maxTorque_H:
        responsePacket.push_back(0x03);
        break;
      case statusReturnLevel:
        responsePacket.push_back(0x02);
        break;
      case alarmLED:
        responsePacket.push_back(0x04);
        break;
      case alarmShutdown:
        responsePacket.push_back(0x04);
        break;
      case downCalibration_L:
        responsePacket.push_back(0x00);
        break;
      case downCalibration_H:
        responsePacket.push_back(0x00);
        break;
      case upCalibration_L:
        responsePacket.push_back(0x00);
        break;
      case upCalibration_H:
        responsePacket.push_back(0x00);
        break;
      case torqueEnable:
        responsePacket.push_back(0x00);
        break;
      case LED:
        responsePacket.push_back(0x00);
        break;
      case cwComplianceMargin:
        responsePacket.push_back(0x00);
        break;
      case ccwComplianceMargin:
        responsePacket.push_back(0x00);
        break;
      case cwComplianceSlope:
        responsePacket.push_back(0x20);
        break;
      case ccwComplianceSlope:
        responsePacket.push_back(0x20);
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
        responsePacket.push_back(0x00);
        break;
    }
  }
}

byte ShoulderTiltStepper::calcCheckSum()
{
  unsigned int parameterSum = 0;
  
  for (byte i=2; i<outLength; i++)
  {
    parameterSum += responsePacket.getElement(i+3);
  }
  
  unsigned int checkSum = getID() + outLength + parameterSum;
  
  return 255 - ( (getID() + outLength + parameterSum) % 256 );
}

void ShoulderTiltStepper::reset()
{
    inLength = outLength = 0;
    inParameters.clear();
    responsePacket.clear();
}

void ShoulderTiltStepper::moveToGoalPosition()
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

int ShoulderTiltStepper::convertFromStepsToDynamixelUnits(int steps)
{
  /* 
   * Each step is 0.1158 degrees.
   * Steps * 0.1158 * 1023 / 300
   */
  return (int)((float)steps * 0.39498);
}

int ShoulderTiltStepper::convertFromDynamixelUnitsToSteps(int dynamixelUnits)
{
  return (int)((float)dynamixelUnits / 0.39498);
}

void ShoulderTiltStepper::incrementPosition()
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
     PORTA |= 0x40;
     
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
      PORTA &= 0xBF;
   }
   
   if (stepBit && --stepsToDo == 0)
   {
     disableTimer();
   }
}

void ShoulderTiltStepper::enableTimer()
{
    PORTA |= 0x40; // Set the clock pin to HIGH
    //PORTC |= 0x80; // Set the enable pin to HIGH
    
    TCNT4 = 0;
    
    /*
     * A precaler or 64 & a OCRF3A count of 250 equates to an angular velocity of approximately 1 rad/s
     */
    TCCR4A = 0; // set entire TCCR4A register to 0
    OCR4A = 1000; // 8 ms pulse
    //OCR4A = 250; // 2 ms pulse
    TCCR4B = 0x0B; // start timer, 64 x prescaler, CTC mode.
    bitWrite(TIMSK4, OCIE4A, 1); // Enable timer 4 OC interrupt A
}

void ShoulderTiltStepper::disableTimer()
{
    TCCR4B = 0;
    TIMSK4 &= 0xFD;     // Disable timer 3 OC interrupt A
    //PORTC &= 0x7F;      // Disable enable pin - active HIGH
    PORTA |= 0x40;      // Set the clock pin to HIGH
    TCNT4 = 0;
    isMoving = false;
}

void ShoulderTiltStepper::moveToHomePosition()
{
    /*
     * We should never hit this position in the normal operation of the motor - other than during a reset. 
     * 
     */
     
    disableTimer(); // stop the motor.
    
   /*
    * For the shoulder pan motor, move the turret fully CCW.
    * For this motor, the min (CCW) is 0, the centre is 733 and the max (CW) 1466
    */
    detachInterrupt(1);
    PORTA |= 0x40; // Set the clock pin to HIGH
    PORTC |= 0x80; // Set the enable pin to HIGH
    PORTC |= 0x20; // Set the direction pint to HIGH (CW)  
    clockwise = false;   
 
    stepsToDo = referencePositionFromCentreInSteps;
    currentPosition = centrePositionInSteps + referencePositionFromCentreInSteps;
    goalPosition = centrePositionInDynamixelUnits;
    enableTimer();
}

void ShoulderTiltStepper::moveToReferencePosition()
{

    attachInterrupt(1, shoulderTiltLimitReached, LOW);
    PORTA |= 0x40; // Set the clock pin to HIGH
    PORTC |= 0x80; // Set the enable pin to HIGH
    PORTC &= 0xBF; // Set the direction pint to LOW (CCW)
 
    stepsToDo = 10000; // keep going until we hit the reference position!
    enableTimer();
}


