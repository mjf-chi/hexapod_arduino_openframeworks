#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 180
#define SERVOMAX 585

uint8_t servonum = 0;

const byte PACKET_BOUNDARY_BYTE = 255;
const int NUM_BYTES_EXPECTED = 18;
byte buffer[NUM_BYTES_EXPECTED] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

byte bufferIndex = 0;

const int NUM_SERVO_PINS = NUM_BYTES_EXPECTED-2;
const byte SERVO_PINS[NUM_SERVO_PINS] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
const int SERVO_MINS[NUM_BYTES_EXPECTED] = {185, 190, 185, 190, 175, 180, 180, 180, 185, 185, 175, 185, 180, 195, 190, 180, 195, 185};
const int SERVO_MAXS[NUM_BYTES_EXPECTED] = {590, 575, 585, 585, 555, 580, 585, 580, 595, 595, 555, 570, 570, 575, 585, 575, 605, 600};

Servo servo17;
Servo servo18;

byte servoValues[NUM_SERVO_PINS] = {90, 0, 0, 90, 0, 0, 90, 0, 0, 90, 0, 0, 90, 0, 0, 90};

int prevVal[NUM_BYTES_EXPECTED] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int updatedVal[NUM_BYTES_EXPECTED] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int counter = 0;

/*
int sensorPin1 = A8;
int sensorPin2 = A9;
int sensorPin3 = A10;
int sensorPin4 = A11;
int sensorPin5 = A12;
int sensorPin6 = A13;

int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int sensorValue4 = 0;
int sensorValue5 = 0;
int sensorValue6 = 0;
*/

int photoResistPins[6] = {A8, A9, A10, A11, A12, A13};
int photoResistVals[6] = {0, 0, 0, 0, 0, 0};

boolean receivedData = false;

byte returnValue = 1;

int value = 0;

void setup()
{
  Serial.begin(9600);
  
  for(int i = 0; i<sizeof(photoResistPins); i++)
  {
    pinMode(photoResistPins[i], INPUT);
  }
  
  /*
  pinMode(sensorPin1, INPUT);
   pinMode(sensorPin2, INPUT);
    pinMode(sensorPin3, INPUT);
     pinMode(sensorPin4, INPUT);
      pinMode(sensorPin5, INPUT);
       pinMode(sensorPin6, INPUT);
  */
  
 //establishContact();
  
  servo17.attach(8);
  servo18.attach(9);
  
  pwm.begin();
  pwm.setPWMFreq(60);
}

void loop()
{
  while(Serial.available() > 0)
  {
    byte inByte = Serial.read();
  
    if(inByte == 255)
    {
      if(bufferIndex == NUM_BYTES_EXPECTED)
      {
        for(int i = 0; i < NUM_SERVO_PINS; i++)
        {
          servoValues[i] = buffer[i];
        }
      
        servoValues[16] = buffer[16];
        servoValues[17] = buffer[17];
      }
    
      else
      {
      }
    
       bufferIndex = 0;
       receivedData = true;
    }
  
    else
    {
      if(bufferIndex < NUM_BYTES_EXPECTED)
      {
        buffer[bufferIndex] = inByte;
        bufferIndex++;
      }
      else
      {
        bufferIndex = 0;
      }
    }
  }
  
  for(int i = 0; i< NUM_SERVO_PINS; i++)
  {
    if(servoValues[i] > 179)
    {
      servoValues[i] = 179;
    }
    
    if(servoValues[i] < 0)
    {
      servoValues[i] = 0;
    }
    int setValue = map(servoValues[i], 0, 180, SERVO_MINS[i], SERVO_MAXS[i]);
    pwm.setPWM(SERVO_PINS[i], 0, setValue);
  }
  
  servo17.write(servoValues[16]);
  servo18.write(servoValues[17]);
  
  //counter ++; 
  for(int i = 0; i<sizeof(photoResistVals); i++)
  {
        photoResistVals[i] = analogRead(photoResistPins[i]);
      
        if(i >= 0 && i < 6)
        {
        //Serial.print(photoResistVals[i]);
        //Serial.print(", ");
        
        if(photoResistVals[i] > 120)
        {
           returnValue = 0;
        }
      }    
  }
  
  //Serial.println(returnValue);
 
  if(receivedData)
  {    
    Serial.write(returnValue);
    
    receivedData = false;
  }
  
  returnValue = 1;
}

void establishContact() {
  while (Serial.available() <= 0) {
    //Serial.print(0);   // send a capital A
    delay(300);
  }
}
