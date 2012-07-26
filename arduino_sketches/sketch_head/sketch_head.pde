/* 
 */

#define TIME_OUT  10

void setup()
{ 
  pinMode(9, OUTPUT);
  //pinMode(13, OUTPUT);
  
  digitalWrite(9, HIGH);
  //digitalWrite(13, LOW);
   
  Serial.begin (1000000);
  Serial1.begin(1000000); 
}

void loop()
{
  unsigned int byteCount = 0;
  unsigned int length = 0;
  unsigned int inByte;
  
  digitalWrite(9, HIGH);
  
  while (Serial.available())
  {
    byteCount++;
    
    inByte = Serial.read();
    
    if (byteCount == 4)
    {
      length = inByte;
    }
    
    Serial1.write(inByte);
    //digitalWrite(13, HIGH);
    
    if (byteCount > (length + 3))
    {  
      delayMicroseconds(400);
      digitalWrite(9, LOW); //read
      
      get_serial1_response();
      break;
    }
  } 
}

void get_serial1_response(void)
{
	unsigned int Time_Counter = 0;
	while(Serial1.available() || Time_Counter < TIME_OUT)
        {  // Wait for Data
		Time_Counter++;
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
