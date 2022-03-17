#include "PID.h"
#include <Wire.h>
#include <Servo.h>

// Declare the Servo pin 
int servoPin = 3; 
// Create a servo object 
Servo Servo1;
 
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;

double PIDstructure[11];
#define ControlFrequency 250.0
#define Setpoint 0.0

long timestart = 0;
int counter = 0;
#define rep 1500
 
void setup(){
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(19200);
PIDInit(PIDstructure, Setpoint, ControlFrequency);
Servo1.attach(servoPin); 
timestart = millis();
Serial.print("\nStart Measuring\n");

}
void loop(){

Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);
 
x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

//delay(400);

char outputBuffer[150];

  //Get the current sensor reading and compute PID
  double pv = x;
  PIDcalculate(PIDstructure, pv);
  
  int Sangle = map(PIDstructure[8],-255, 255, 0,180);
  
  //sprintf(outputBuffer, "Pv: %d \t Error: %d \t Output: %d\t Intrgral: %d \tServo: %d\n", (int)pv, (int)PIDstructure[7], (int)PIDstructure[8], (int)PIDstructure[10], Sangle); 
  //Serial.print(outputBuffer);

  Servo1.write(Sangle); 
  delay(10);
  counter++;
  if(counter > rep)
  {
    long timenow = millis() - timestart;
    while(1){
      Serial.print(timenow);
      Serial.print("\n");
      delay(5000);
    }
  }

}
