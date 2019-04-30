/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").
Contact information
-------------------
Kristian Lauszus, TKJ Electronics
Web : http://www.tkjelectronics.com
e-mail : kristianl@tkjelectronics.com
*/

#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
//using integer
double before = 0;
typedef struct circle{
  double start;
  double fin;
  int data;
}circle;
circle c[13]={{0,15,0},{15,45,1},{45,75,1},{75,105,1},{105,135,1},{135,165,1},{165,195,1},{195,225,1},{225,255,1},{255,285,1},{285,315,1},{315,345,1},{345,0,0}};
/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter
void digital_show(int inupt);
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
int return_direction(double accXangle) ;
void setup() {
  Serial.begin(9600);
   pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  Wire.begin();
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while(i2cWrite(0x19,i2cData,4,false)); // Write to all four registers at once
  while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode
  while(i2cRead(0x75,i2cData,1));
  if(i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    //Serial.print(F("Error reading sensor"));
    while(1);
  }
  
  delay(100); // Wait for sensor to stabilize
  
  /* Set kalman and gyro starting angle */
  while(i2cRead(0x3B,i2cData,6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  
  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compAngleX = accXangle;
  compAngleY = accYangle;
  
  timer = micros();
}
int k;
int sum = 0;
void loop() {
  /* Update all the values */
  while(i2cRead(0x3B,i2cData,14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);
  
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = -((double)gyroY/131.0);
  gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter
  gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
  //gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);
  
  compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle); // Calculate the angle using a Complimentary filter
  compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);
  
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
  timer = micros();
  
  temp = ((double)tempRaw + 12412.0) / 340.0;
  
  /* Print Data */
  display_formatted_float(accX, 5, 0, 3, false);
  display_formatted_float(accY, 5, 0, 3, false);
  display_formatted_float(accZ, 5, 0, 3, false);
  display_formatted_float(gyroX, 5, 0, 3, false);
  display_formatted_float(gyroY, 5, 0, 3, false);
  display_formatted_float(gyroZ, 5, 0, 3, false);

  //Serial.print("\t");

  display_formatted_float(accXangle, 5, 2, 3, false);
  display_formatted_float(gyroXangle, 5, 2, 3, false);
  display_formatted_float(compAngleX, 5, 2, 3, false);
  display_formatted_float(kalAngleX, 5, 2, 3, false);

  //Serial.print("\t");

  display_formatted_float(accYangle, 5, 2, 3, false);
  display_formatted_float(gyroYangle, 5, 2, 3, false);
  display_formatted_float(compAngleY, 5, 2, 3, false);
  display_formatted_float(kalAngleY, 5, 2, 3, false);

  //Serial.print(temp);Serial.print("\t");
  k = return_direction(kalAngleX);
  digital_show(k);
  Serial.println(k);
  /*Serial.println(k);
  Serial.print("\r\n");
  Serial.println(k);
  Serial.print("\r\n");*/
  delay(20);
}

void display_formatted_float(double val, int characteristic, int mantissa, int blank, boolean linefeed) {
  char outString[16];
  int len;

  dtostrf(val, characteristic, mantissa, outString);
  len = strlen(outString);
  for(int i = 0; i < ((characteristic+mantissa+blank)-len); i++) //Serial.print(F(" "));
  //Serial.print(outString);
  if(linefeed)
    /*Serial.print(F("\n"))*/;
}
int i;    
circle now;
circle prev = {0,0,0};
int return_direction(double accXangle) 
{
  if(accXangle >= before)
  {
    if(before != accXangle)
    {
      for(i=0; i<13; i++){
        if(c[i].start <= accXangle && accXangle < c[i].fin)
          now = c[i];
      if(now.fin != prev.fin)
            sum = sum+now.data;

        }
      }
}
  
  if(accXangle <= before)
  {
    if(before != accXangle)
    {
      for(i=0; i<13; i++)
      if(c[i].start <= accXangle && accXangle < c[i].fin)
        now = c[i];
      if(now.fin != prev.fin)
        sum = sum-now.data;
    }
  }
  before = accXangle;
  prev = now;

  if(sum > 0)
    return 1;
  if(sum < 0)
    return -1;
  if(sum == 0)
    return 0;
  
}
      
void digital_show(int input){
  int a = Serial.read();
  Serial.println(a);                
  if(a == 1){
    digitalWrite(3,LOW);
    shiftOut(2,4,MSBFIRST,B11111111);
    digitalWrite(3,HIGH);
    digitalWrite(6,LOW);
    shiftOut(5,7,MSBFIRST,B11111111);
    digitalWrite(6,HIGH);
    digitalWrite(9,LOW);
    shiftOut(8,10,MSBFIRST,B11111111);
    digitalWrite(9,HIGH);
    delay(500);
    digitalWrite(6,LOW);
    shiftOut(5,7,MSBFIRST,B00000000);
    digitalWrite(6,HIGH);
    digitalWrite(9,LOW);
    shiftOut(8,10,MSBFIRST,B00000000);
    digitalWrite(9,HIGH);
    digitalWrite(3,LOW);
    shiftOut(2,4,MSBFIRST,B00000000);
    digitalWrite(3,HIGH);
    delay(500);
    return;
  }
  if(a == 2){
    digitalWrite(3,LOW);
    shiftOut(2,4,MSBFIRST,B10110100);
    digitalWrite(3,HIGH);
    digitalWrite(6,LOW);
    shiftOut(5,7,MSBFIRST,B11001111);
    digitalWrite(6,HIGH);
    digitalWrite(9,LOW);
    shiftOut(8,10,MSBFIRST,B10001011);
    digitalWrite(9,HIGH);
    delay(300);
    digitalWrite(6,LOW);
    shiftOut(5,7,MSBFIRST,B11011101);
    digitalWrite(6,HIGH);
    digitalWrite(9,LOW);
    shiftOut(8,10,MSBFIRST,B11001010);
    digitalWrite(9,HIGH);
    digitalWrite(3,LOW);
    shiftOut(2,4,MSBFIRST,B11000101);
    digitalWrite(3,HIGH);
    delay(300);
    return;
  }
  if(input < 0){
    digitalWrite(3,LOW);
  shiftOut(2,4,MSBFIRST,B10101010);
  digitalWrite(3,HIGH);
  digitalWrite(6,LOW);
  shiftOut(5,7,MSBFIRST,B10101010);
  digitalWrite(6,HIGH);
  digitalWrite(9,LOW);
  shiftOut(8,10,MSBFIRST,B10101010);
  digitalWrite(9,HIGH);
  delay(300);
  digitalWrite(6,LOW);
  shiftOut(5,7,MSBFIRST,B00000000);
  digitalWrite(6,HIGH);
  digitalWrite(9,LOW);
  shiftOut(8,10,MSBFIRST,B00000000);
  digitalWrite(9,HIGH);
  digitalWrite(3,LOW);
  shiftOut(2,4,MSBFIRST,B00000000);
  digitalWrite(3,HIGH);
  delay(300);
  }
  if(input > 0){
      digitalWrite(3,LOW);
  shiftOut(2,4,MSBFIRST,B01010101);
  digitalWrite(3,HIGH);
  digitalWrite(6,LOW);
  shiftOut(5,7,MSBFIRST,B01010101);
  digitalWrite(6,HIGH);
  digitalWrite(9,LOW);
  shiftOut(8,10,MSBFIRST,B01010101);
  digitalWrite(9,HIGH);
  delay(300);
  digitalWrite(6,LOW);
  shiftOut(5,7,MSBFIRST,B00000000);
  digitalWrite(6,HIGH);
  digitalWrite(9,LOW);
  shiftOut(8,10,MSBFIRST,B00000000);
  digitalWrite(9,HIGH);
  digitalWrite(3,LOW);
  shiftOut(2,4,MSBFIRST,B00000000);
  digitalWrite(3,HIGH);
  delay(300);
  }
}
