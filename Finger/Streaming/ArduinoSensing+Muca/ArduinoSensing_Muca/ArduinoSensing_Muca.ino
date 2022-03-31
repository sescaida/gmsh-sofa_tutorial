#include <Muca.h>

Muca muca;

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
///float V2kPa5100 = 1000.0/45; // What is the conversion factor between the voltage and the pressure in kPa. What information can you get from the datasheet? (MPXV7025 and MPX5100)

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




void printMucaData() {
  for(int row = 11; row <=12; row++) { // NUM_ROWS 21
    for(int col = 9; col<= 12; col++) { // NUM_COLUMNS 12
      if(col+row !=20)
        Serial.print(",");
      int val = muca.getRawData(col,row);
       Serial.print(val);       
   //     muca.getRawData(i,j);
    }
  }
  Serial.println();
 // Serial.println(muca.getRawData(5, 10)); // Get the point at the Column 5, Row 10 
//  /delay(100);
}
void initMuCa()
{
  muca.init();
  muca.useRawData(true); // If you use the raw data, the interrupt is not working

  delay(50);
  muca.setGain(2);

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

  initMuCa();

  
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
    float P3 = MeanA3 * AD2V * V2kPa;
    Serial.print(P0);
    Serial.print(",");
    Serial.print(P1);
    Serial.print(",");
    Serial.print(P2);
    Serial.print(",");    
    Serial.print(P3);
    Serial.print(",");
    printMucaData();
    // Make sure integration and counters are set back to 0!
    Counter = 0;
    SumA0 = 0;
    SumA1 = 0;
    SumA2 = 0;
    SumA3 = 0;
  }
  Counter++;  
}
