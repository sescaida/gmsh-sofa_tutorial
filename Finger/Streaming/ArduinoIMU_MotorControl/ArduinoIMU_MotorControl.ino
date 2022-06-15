/*!
  * imu_show.ino
  *
  * Download this demo to show attitude on [imu_show](https://github.com/DFRobot/DFRobot_IMU_Show)
  * Attitude will show on imu_show
  *
  * Product: http://www.dfrobot.com.cn/goods-1860.html
  * Copyright   [DFRobot](http://www.dfrobot.com), 2016
  * Copyright   GNU Lesser General Public License
  *
  * version  V1.0
  * date  07/03/2019
  */

#include <Servo.h>

Servo myservo [3]; //instances of the servo object
int ServoPin []={9, 10, 11};
int pos [] = {0, 0, 0};
int header;


#include "DFRobot_BNO055.h"
#include "Wire.h"

typedef DFRobot_BNO055_IIC    BNO;    // ******** use abbreviations instead of full names ********

BNO   bno(&Wire, 0x28);    // input TwoWire interface and IIC address

// show last sensor operate status
void printLastOperateStatus(BNO::eStatus_t eStatus)
{
  switch(eStatus) {
  case BNO::eStatusOK:    Serial.println("everything ok"); break;
  case BNO::eStatusErr:   Serial.println("unknow error"); break;
  case BNO::eStatusErrDeviceNotDetect:    Serial.println("device not detected"); break;
  case BNO::eStatusErrDeviceReadyTimeOut: Serial.println("device ready time out"); break;
  case BNO::eStatusErrDeviceStatus:       Serial.println("device internal status error"); break;
  default: Serial.println("unknow status"); break;
  }
}

void setup()
{
  for (int i = 0; i < 3; i++)
  {
    myservo[i].attach(ServoPin[i]);
  }
  Serial.begin(115200);
  bno.reset();
  while(bno.begin() != BNO::eStatusOK) {
    Serial.println("bno begin faild");
    printLastOperateStatus(bno.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bno begin success");
}

void loop()
{
  BNO::sQuaAnalog_t   sQuat;
  sQuat = bno.getQua();  
  Serial.print(sQuat.x, 4); 
  Serial.print(","); 
  Serial.print(sQuat.y, 4); 
  Serial.print(","); 
  Serial.print(sQuat.z, 4); 
  Serial.print(","); 
  Serial.print(sQuat.w, 4); 
  Serial.println(",");
  delay(5);
  
  // Reading servo commands
  int value1;
  int value2;
  int value3;
  
  if(Serial.available()>1)
  {
    value1 = Serial.parseInt();
    value2 = Serial.parseInt();
    value3 = Serial.parseInt();
    myservo[0].write(value1);     
    myservo[1].write(value2);     
    myservo[2].write(value3);     
  
//  if(Serial.available())
//  {
//  header=Serial.read();
//  while (header!=255)
//    { 
//      while(!Serial.available());
//      header=Serial.read();     
////      Serial.print("header + ");
//    }
//  for (int i = 0; i < 3; i++)
//  {
//    while(!Serial.available());
//    pos [i] = Serial.read();
////    Serial.print(pos[i]);
////    Serial.print(", ");
//  }  
//  for (int i = 0; i < 3; i++)
//  {
//    myservo[i].write(pos[i]);     
//  }
//  Serial.println("was here!");
  }
}
