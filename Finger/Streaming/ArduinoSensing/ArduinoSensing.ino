#include <Wire.h>

float SumA0 = 0;
float SumA1 = 0;
float SumA2 = 0;
float SumA3 = 0;
float OffsetA0 = 0;
float OffsetA1 = 0;
float OffsetA2 = 0;
float OffsetA3 = 0;

int Counter = 0;

//---------------
// Task 1
//---------------

float AD2V = 5.0/1023; //what is the conversion factor between analog to digital value and voltage, when the range 0V-5V is coded using 10 bits, i.e. values between 0 and 1023?
float V2kPa = 1000.0/90; // What is the conversion factor between the voltage and the pressure in kPa. What information can you get from the datasheet? (MPXV7025 and MPX5100)
float V2kPa5100 = 1000.0/45; // What is the conversion factor between the voltage and the pressure in kPa. What information can you get from the datasheet? (MPXV7025 and MPX5100)

//---------------
// Variables for Task 2
//---------------

bool Beginning = true;
unsigned long CurrentTimestamp, InitTimestamp; // in ms
unsigned long InitWait = 3000; // in ms
float MeanWindowSizeBegin = 100;
float MeanWindowSizeNormal = 25;
float MeanWindowSize = MeanWindowSizeBegin;

// Stuff for capacitive sensor board
int LED_PIN = 13;
byte Button4Value,Button4ValueOld;
unsigned long I2C_Timestamp = 0;
int I2C_Interval = 50; //ms
byte DetectionThreshold = 60;

byte capValue(byte Val)
{
  if (Val > 127)
  {
    Val = 0;
  }
  return Val;
}

void getButton4Value()
{
   unsigned long CurrentTimestamp = millis();
   
   if (CurrentTimestamp-I2C_Timestamp > I2C_Interval)
   {
     Wire.requestFrom(0x28, 1);    // 
     while (Wire.available()) { // slave may send less than requested
       Button4Value = capValue(Wire.read()); // receive a byte as character
//       Serial.println(Button4Value,BIN);         // print the character
       if (Button4Value > DetectionThreshold)
       {
        digitalWrite(LED_PIN, HIGH);  
       }
       else
       {
        digitalWrite(LED_PIN, LOW);
       }
     }
     I2C_Timestamp = CurrentTimestamp;
   }
}



void initCapSensor()
{
    
  Wire.begin();
  int ScalingFactor = 0x07;

  Wire.beginTransmission(0x28); // DM160227 default's address is 0x28
  Wire.write(0x80); // 0x25 -> set register for reading button 4 values  
  Wire.write(0x15); // 0x25 -> set register for reading button 4 values
  Wire.endTransmission();    // stop transmitting
  

  Wire.beginTransmission(0x28); // DM160227 default's address is 0x28
  Wire.write(0x81); // 0x25 -> set register for reading button 4 values  
  Wire.write(ScalingFactor); // 0x25 -> set register for reading button 4 values
  Wire.endTransmission();    // stop transmitting
  
  Wire.beginTransmission(0x28); // DM160227 default's address is 0x28
  Wire.write(0x82); // 0x25 -> set register for reading button 4 values  
  Wire.write(ScalingFactor); // 0x25 -> set register for reading button 4 values  
  Wire.endTransmission();    // stop transmitting
  
  Wire.beginTransmission(0x28); // DM160227 default's address is 0x28
  Wire.write(0x83); // 0x25 -> set register for reading button 4 values  
  Wire.write(ScalingFactor); // 0x25 -> set register for reading button 4 values
  Wire.endTransmission();    // stop transmitting
  
  Wire.beginTransmission(0x28); // DM160227 default's address is 0x28
  Wire.write(0x84); // 0x25 -> set register for reading button 4 values
  Wire.write(ScalingFactor); // 0x25 -> set register for reading button 4 values
  Wire.endTransmission();    // stop transmitting
  
  Wire.beginTransmission(0x28); // DM160227 default's address is 0x28
  Wire.write(0x85); // 0x25 -> set register for reading button 4 values
  Wire.write(ScalingFactor+2); // 0x25 -> set register for reading button 4 values
  Wire.endTransmission();    // stop transmitting
  
  Wire.beginTransmission(0x28); // DM160227 default's address is 0x28
  Wire.write(0x01); // 0x25 -> set register for reading button 4 values
  Wire.endTransmission();    // stop transmitting
  // -------------------

  // wait for 5 secs before doing anything in order to let the values of the capacitive sensor settle
  digitalWrite(LED_PIN, HIGH);  
  delay(1000);
  digitalWrite(LED_PIN, LOW);  

}

void setup()
{
  Serial.begin(115200);
//  Serial.println("P0,P1,P2,P3");
  delay(100);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  initCapSensor();

  
  InitTimestamp = millis();
}

void loop()
{
  float AZero = analogRead(A0);
  float AOne = analogRead(A1);
  float ATwo = analogRead(A2);
  float AThree = analogRead(A3);

  SumA0 += AZero;
  SumA1 += AOne;
  SumA2 += ATwo;
  SumA3 += AThree;

  if (Counter >= MeanWindowSize)
  {
    float MeanA0 = (SumA0 / MeanWindowSize) - OffsetA0;
    float MeanA1 = (SumA1 / MeanWindowSize) - OffsetA1;
    float MeanA2 = (SumA2 / MeanWindowSize) - OffsetA2;
    float MeanA3 = (SumA3 / MeanWindowSize) - OffsetA3;

    //---------------
    // Task2: implement calculation of Offsets when initializing
    //---------------
    CurrentTimestamp = millis();
    if (((CurrentTimestamp - InitTimestamp) > InitWait) && Beginning)
    {
      OffsetA0 = MeanA0;
      OffsetA1 = MeanA1;
      OffsetA2 = MeanA2;
      OffsetA3 = MeanA3;
      Beginning = false;
      MeanWindowSize = MeanWindowSizeNormal;
//      Serial.println("Offsets set!");
//      Serial.println("P0,P1,P2,P3"); 
    }

    float P0 = MeanA0 * AD2V * V2kPa;
    float P1 = MeanA1 * AD2V * V2kPa;
    float P2 = MeanA2 * AD2V * V2kPa;
    float P3 = MeanA3 * AD2V * V2kPa5100;
    Serial.print(P0);
    Serial.print(",");
    Serial.print(P1);
    Serial.print(",");
    Serial.print(P2);
    Serial.print(",");
    Serial.print(P3);
    Serial.print(",");
    Serial.println(Button4Value);
    
    // Make sure integration and counters are set back to 0!
    Counter = 0;
    SumA0 = 0;
    SumA1 = 0;
    SumA2 = 0;
    SumA3 = 0;
  }
  Counter++;
  getButton4Value();
}
