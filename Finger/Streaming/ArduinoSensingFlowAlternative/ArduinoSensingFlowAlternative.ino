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

bool DoIntegration = false;
bool ResetIntegration = true;
unsigned long TimeStepStart=0, TimeStepEnd;
//---------------
// Task 1
//---------------

float AD2V = 5.0/1023; //what is the conversion factor between analog to digital value and voltage, when the range 0V-5V is coded using 10 bits, i.e. values between 0 and 1023?
float LperMin2LperSec = 1.0/60.0;
float V2Flow = (0.1/2.0)*LperMin2LperSec ; // datasheet 2V==0.1L/min. 
float Volume = 0;
//---------------
// Variables for Task 2
//---------------

bool Beginning = true;
unsigned long CurrentTimestamp, InitTimestamp; // in ms
unsigned long InitWait = 3000; // in ms
float MeanWindowSizeBegin = 100;
float MeanWindowSizeNormal = 25;
float MeanWindowSize = MeanWindowSizeBegin;




void setup()
{
  Serial.begin(115200);
//  Serial.println("P0,P1,P2,P3");
  delay(100);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  //Serial.println("Flow, Volume");
  
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

    float F0 = MeanA0 * AD2V * V2Flow;
    float P1 = 0;
    float P2 = 0;
    float P3 = 0;

    // Do integration
       
    if (F0 <-0.00005)
    {
      Volume = 0;
    }


    float ThresholdForIntegration = 0.000005; // in L/s
    if (F0 > ThresholdForIntegration)
    {
      TimeStepEnd = millis();
      unsigned long DeltaT = TimeStepEnd - TimeStepStart;
      Volume += DeltaT*F0; // During integration we work in mL in order not to get to small floats
    }    


    TimeStepStart=millis();


    Serial.print(F0,8);
    Serial.print(",");
    float EmpiricalCalibrationFactor = 1.15;
    Serial.println(Volume*EmpiricalCalibrationFactor/1000,8); //output is in L!
    
    // Make sure integration and counters are set back to 0!
    Counter = 0;
    SumA0 = 0;
    SumA1 = 0;
    SumA2 = 0;
    SumA3 = 0;
  }
  Counter++;  
}
