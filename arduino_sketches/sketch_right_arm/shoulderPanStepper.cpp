#include "shoulderPanStepper.h"

extern void shoulderPanLimitReached();
extern void shoulderTiltLimitReached();

ShoulderPanStepper::ShoulderPanStepper(byte ID) : Stepper (ID)
{
  isMoving = false;
  stepsToDo = 0;
  stepBit = true;
  
  clockPin = 22;
  enablePin = 24;
  directionPin = 26;
  
  centrePositionInDynamixelUnits = 512;
  centrePositionInSteps = 432; // 50 degrees. 
  referencePositionFromCentreInSteps = 740; 
  
  goalPosition = centrePositionInDynamixelUnits;
  currentPosition = centrePositionInSteps;
  
  pinMode(clockPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(directionPin, OUTPUT);
}

void ShoulderPanStepper::sendData(LList<byte> *inputData)
{
    byte i = 0;
    while (!(inputData->isEmpty()))
    {
      extractInputPacketField(inputData->pop_front(), i++);
    }
}

void ShoulderPanStepper::getResponse()
{
  setResponsePacket();
  
  for (byte i=0; i<responsePacket.size(); i++)
  {
    Serial.write(responsePacket.getElement(i));
  }
  
  reset();
}

void ShoulderPanStepper::doOperation(byte operation, LList<byte> *inputData)
{
  switch (operation)
  {
    case 0x1E: // goal position
      goalPosition = inputData->getElement(8) + (256 * inputData->getElement(9));
      moveToGoalPosition();
      break;
  }
}

void ShoulderPanStepper::extractInputPacketField(byte field, byte index)
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

void ShoulderPanStepper::initResponsePacket()
{
  responsePacket.push_back(0xFF);
  responsePacket.push_back(0xFF);
  responsePacket.push_back(getID());
  responsePacket.push_back(outLength);
  responsePacket.push_back(0x00); //error code
}

void ShoulderPanStepper::setResponsePacket()
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

void ShoulderPanStepper::addResponsePacketParameters()
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
        responsePacket.push_back(SHOULDER_PAN_MOTOR);
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

byte ShoulderPanStepper::calcCheckSum()
{
  unsigned int parameterSum = 0;
  
  for (byte i=2; i<outLength; i++)
  {
    parameterSum += responsePacket.getElement(i+3);
  }
  
  unsigned int checkSum = getID() + outLength + parameterSum;
  
  return 255 - ( (getID() + outLength + parameterSum) % 256 );
}

void ShoulderPanStepper::reset()
{
    inLength = outLength = 0;
    inParameters.clear();
    responsePacket.clear();
}

void ShoulderPanStepper::moveToGoalPosition()
{
  /*
   * The goal position is relative to the centre position.It is not an absolute distance from the current position.
   * We therefore need to work out where we are relative to the centre position, then move the motor to that position.
   */
  int goalPositionInSteps = convertFromDynamixelUnitsToSteps(goalPosition - centrePositionInDynamixelUnits) + centrePositionInSteps;
  
  clockwise = currentPosition < goalPositionInSteps;
  
  /*
   * The shoulder pan stepper is now upside down, so CW for it is CCW for the shoulder turret!
   * Swap the direction for the shoulder pan Stepper therefore.
   */
   digitalWrite(directionPin, !clockwise ); 

     
  stepsToDo =  abs(currentPosition - goalPositionInSteps);
  
  if (stepsToDo)
  {
    isMoving = true;
    enableTimer();
  }
}

int ShoulderPanStepper::convertFromStepsToDynamixelUnits(int steps)
{
  /* 
   * Each step is 0.1158 degrees.
   * Steps * 0.1158 * 1023 / 300
   */
  return (int)((float)steps * 0.39498);
}

int ShoulderPanStepper::convertFromDynamixelUnitsToSteps(int dynamixelUnits)
{
  return (int)((float)dynamixelUnits / 0.39498);
}

void ShoulderPanStepper::incrementPosition()
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
      PORTA |= 0x01; // clockPin
     
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
      PORTA &= 0xFE;
   }
   
   if (stepBit && --stepsToDo == 0)
   {
     disableTimer();
   }
}

void ShoulderPanStepper::enableTimer()
{
    PORTA |= 0x01; // Set the clock pin to HIGH
    PORTA |= 0x04; // Set the enable pin to HIGH
    
    TCNT3 = 0;
    
    /*
     * A precaler or 64 & a OCRF3A count of 250 equates to an angular velocity of approximately 1 rad/s
     */    
    TCCR3A = 0; // set entire TCCR3A register to 0
    OCR3A = 500; // 2 ms pulse
    //OCR3A = 250; // 1 ms pulse
    TCCR3B = 0x0B; // start timer, 64 x prescaler, CTC mode.
    bitWrite(TIMSK3, OCIE3A, 1); // Enable timer 3 OC interrupt A
}

void ShoulderPanStepper::disableTimer()
{
    TCCR3B = 0;
    TIMSK3 &= 0xFD;     // Disable timer 3 OC interrupt A
    PORTA &= 0xFB;      // Disable enable pin - active HIGH
    PORTA |= 0x01;      // Set the clock pin to HIGH
    TCNT3 = 0;
    isMoving = false;
}

void ShoulderPanStepper::moveToHomePosition()
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
    detachInterrupt(0);
    PORTA |= 0x01; // Set the clock pin to HIGH
    PORTA |= 0x04; // Set the enable pin to HIGH
    PORTA |= 0x10; // Set the direction pin to HIGH (CW)
    clockwise = false;
    
    stepsToDo = referencePositionFromCentreInSteps;
    currentPosition = centrePositionInSteps + referencePositionFromCentreInSteps;
    goalPosition = centrePositionInDynamixelUnits;
    enableTimer();
}

void ShoulderPanStepper::moveToReferencePosition()
{

    attachInterrupt(0, shoulderPanLimitReached, LOW);
    PORTA |= 0x01; // Set the clock pin to HIGH
    PORTA |= 0x04; // Set the enable pin to HIGH
    PORTA &= 0xEF; // Set the direction pin to LOW (CCW)
 
    stepsToDo = 10000; // keep going until we hit the reference position!
    enableTimer();
}


