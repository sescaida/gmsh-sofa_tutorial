#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HerkulexServo.h>

#define PIN_SW_RX 8
#define PIN_SW_TX 9
#define SERVO_ID  7

SoftwareSerial   servo_serial(PIN_SW_RX, PIN_SW_TX);
HerkulexServoBus herkulex_bus(servo_serial);
HerkulexServo    my_servo(herkulex_bus, SERVO_ID);

unsigned long last_update = 0;
unsigned long now = 0;
int DesiredAngle = 0;
bool toggle = false;


void setup() {
  Serial.begin(115200);
  servo_serial.begin(115200);
  delay(500);
  my_servo.setTorqueOn();  // turn power on
}

int Deg2Bin(int Deg)
{
  float ConversionFactor = 0.325;
  int ZeroOffset = 512;
  int Bin = ZeroOffset + round(Deg/ConversionFactor);
  return Bin;
}

float Bin2Deg(int Bin)
{
  float Deg = 0;
  float ZeroOffset = 512;
  float ConversionFactor = 0.325;
  Deg = (Bin-ZeroOffset) * ConversionFactor; 
  return Deg;
}
void loop() {
  herkulex_bus.update();

  now = millis();
  

  if (Serial.available())
  {
     DesiredAngle = Serial.parseInt();
     int BinAngle = Deg2Bin(DesiredAngle);
//     Serial.print("Going to the following Angle (deg, bin): ");
//     Serial.print(DesiredAngle);
//     Serial.print(", ");
//     Serial.println(BinAngle);
     my_servo.setPosition(BinAngle, 50, HerkulexLed::Green);
     while(Serial.available())
     {
      Serial.read();
//      Serial.println("Debug");
     }
  }
  
  uint16_t CurrentPosition = my_servo.getPosition();
//  Serial.println(CurrentPosition);
  Serial.println(Bin2Deg(CurrentPosition));
  
//  if ( (now - last_update) > 1000) {
//    // called every 1000 ms
//    if (toggle) {
//      // move to -90° over a duration of 560ms, set LED to green
//      // 512 - 90°/0.325 = 235
//      my_servo.setPosition(512, 50, HerkulexLed::Green);
//    } else {
//      // move to +90° over a duration of 560ms, set LED to blue
//      // 512 + 90°/0.325 = 789
//      my_servo.setPosition(512, 50, HerkulexLed::Blue);
//    }
//
//    last_update = now;
//    toggle = !toggle;
//  }
}
