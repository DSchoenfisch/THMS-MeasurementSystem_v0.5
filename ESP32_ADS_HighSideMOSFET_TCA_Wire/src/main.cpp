#include <Arduino.h>
#include <Wire.h>
#include <Linear_Regression.h>
#include <Adafruit_ADS1015_DS.h>

#define SOFTWARE_VERSION         ("v1.0.0") // Version of Software
#define VERSION_INFO             ("Sampling timing auto; Array Length 40; Gain Eight; DR_5; S-Interval 7900us. M-Interval 10s") // Version of Software
#define ARRAY_LENGTH             (40) //  ARRAY_LENGTH = 300 000 us / SAMPLE_INTERVAL_US
#define LREG_START_POINT         (6)  // Linear Regression start point of measurement array
#define PULSE_PIN1               (25) 
#define PULSE_PIN2               (26)
#define PULSE_PIN3               (27)
#define PULSE_PIN4               (18)   // ! 9 Keine Funktion / Anderweitg belegt
#define PULSE_PIN5               (23)   // ! 10 Keine Funktion / Anderweitg belegt
#define PULSE_PIN6               (13)
#define PULSE_PIN7               (5)
#define PULSE_PIN8               (2)
#define SAMPLE_INTERVAL_US       (7900) // in us ; 1sec/ADS_DATARATE ;DR_3=31250; DR_4 = 15625; DR_5=7900; DR_6=4000; DR_7 = 2105
#define BATTERY_ADC_PIN          A0 //microseconds
#define TCA_ADDRESS              (0x70)
#define MEASUREMENT_INTERVAL_S   (10)
#define INVERT_BRIDGE_V          true
#define ADS_GAIN                 GAIN_EIGHT  // Gain 8: +-512mV ; Gain 16=  +/- 0.256V -> 1 bit = 0.0078125mV
#define ADS_DATARATE             DR_5     // ADS1115/ADS1115 [Sps]: DR_3=32/490;DR_4=64/920; DR_5=128/1600 (Default); DR_6=250/2400 ; DR_7 = 475/3300
#define MULTIPLIER				 0.015625F  //=Gain / 16/12Bit ; e.g. GAIN_EIGHT: +-512mV/16Bit = 0.015625F


byte pulsePin[9] = { PULSE_PIN8, //PIN1 = pulsePin[1]
                     PULSE_PIN1,
                     PULSE_PIN2,
                     PULSE_PIN3,
                     PULSE_PIN4,
                     PULSE_PIN5,
                     PULSE_PIN6,
                     PULSE_PIN7,
                     PULSE_PIN8};


bool a = true;

float   multiplier = MULTIPLIER;    // ADS1015 @ +/- 6.144V gain (12-bit results) ->0.25F; 
int16_t measurement_no = 0;

Adafruit_ADS1115_DS ads;
Linear_Regression LinReg;



void setup() {
  multiplier = INVERT_BRIDGE_V ? (-multiplier) : multiplier;
  //pinMode(LED_BUILTIN, OUTPUT);
  for(byte i = 1 ; i <= 8 ; i++){
     pinMode(pulsePin[i], OUTPUT);
     digitalWrite(pulsePin[i], HIGH);//active LOW 
  }                                   
  //while(!Serial);
  Wire.begin(); 
  Serial.begin(115200);   
  Serial.print("Software Version: ");Serial.println(SOFTWARE_VERSION);
  Serial.print("Info: ");Serial.println(VERSION_INFO);
  ads.setGain(ADS_GAIN);   
  ads.change_I2C_address(ADDR_TO_VDD);
  ads.setDataRate(ADS_DATARATE);
  //ads.begin();
}

void setTCAChannel(byte sensor_channel){
  Wire.beginTransmission(TCA_ADDRESS);
  byte sendbyte = (sensor_channel) ? (1 << (sensor_channel-1)) : (0xFF) ; // sensor_channel 0 = Broadcast
  Wire.write(sendbyte);
  Wire.endTransmission();
}

void performMeasurement(byte sensorNo){
  setTCAChannel(sensorNo);
  float resultArray[ARRAY_LENGTH];
  float sqrtTimeArray[ARRAY_LENGTH]; //in us
  int16_t sampelInterval_us;
  uint32_t t;  
  float slope = 0.0;
  float B = 0.0;
  float Rpow2 = 0.0;
  //START MEASUREMEN.
  uint8_t errorHandler = 0;
  errorHandler = ads.startContinuous_Differential_0_1();
  if(!errorHandler){
   //digitalWrite(LED_BUILTIN, HIGH);
   delayMicroseconds(SAMPLE_INTERVAL_US-(micros()%SAMPLE_INTERVAL_US)); // Start der Messung zu stets gleichem Zeitpunkt
   t = micros();
   digitalWrite(pulsePin[sensorNo], LOW);
   for (int16_t i=0;i<ARRAY_LENGTH;i++)
   {
      delayMicroseconds(SAMPLE_INTERVAL_US-(micros()%SAMPLE_INTERVAL_US)); // Warten vor dem Lesen des Messwerts, damit genug Zeit zum durchschalten des Transistors
      resultArray[i] = ads.readMeasVal_Contiuous(); 
   }
   digitalWrite(pulsePin[sensorNo], HIGH);
   //digitalWrite(LED_BUILTIN, LOW);
   t = micros() - t;
   sampelInterval_us = t/ARRAY_LENGTH;
   ads.powerDown();

   // ---Start: Serail print of measurement values --- 
      Serial.print("Measurement Time: ");Serial.print(t/1000);Serial.println("ms");
      Serial.print("Sample Rate: ");Serial.print(1000000/(t/ARRAY_LENGTH));Serial.println("SPS");
      Serial.print("Sample Interval: ");Serial.print(sampelInterval_us);Serial.println("us");
      Serial.print("Time Array [s^0.5]: ");
      for (int16_t i=0;i<ARRAY_LENGTH;i++) {
         sqrtTimeArray[i] = sqrt(((float)i*sampelInterval_us)/1000000); //in s^0.5
      }
      for (int16_t i=0;i<ARRAY_LENGTH;i++) {
         Serial.print(sqrtTimeArray[i],3);
         Serial.print(" ; ");
      }
      Serial.println();
      Serial.print("Bridge Voltage Array [mV]: ");
      for (int16_t i=0;i<ARRAY_LENGTH;i++) {
         resultArray[i]=resultArray[i]*multiplier; // in mV
         Serial.print(resultArray[i],3);
         Serial.print(" ; ");
      }
      Serial.println();

      t = micros();
      LinReg.calculate_simpleLR(&resultArray[LREG_START_POINT],&sqrtTimeArray[LREG_START_POINT],ARRAY_LENGTH-LREG_START_POINT,&slope,&B,&Rpow2);
      t = micros() - t;
      Serial.print("Regression Start Point: ");Serial.println(LREG_START_POINT);
      Serial.print("Slope [(s^0.5)/V]: ");Serial.println(slope*1000,10);
      Serial.print("B [(s^0.5)]: ");Serial.println(B,10);
      Serial.print("R² [ ]: ");Serial.println(Rpow2,10);
      Serial.print("Regression Time: ");Serial.print(t/1000,2);Serial.println("ms");
   // ---End: Serail print of measurement values --- *
  }
  else
  {
      Serial.print("I2C communication error. Code: "); Serial.println(errorHandler);
  }

}

void loop() {
  for(byte i = 1 ; i <= 8 ; i++){
     digitalWrite(pulsePin[i], HIGH); 
  } 
  for(uint8_t i = 1 ; i <= 8; i++){
    Serial.println("<<< START >>>");
    Serial.print("Sensor: ");Serial.println(i);
    Serial.print("Measurement Number: ");Serial.println(measurement_no);
    performMeasurement(i);
    Serial.println("<<< END >>>");
  }
  uint16_t measurementDelayTime = MEASUREMENT_INTERVAL_S*1000 - (millis() % (MEASUREMENT_INTERVAL_S*1000));
  Serial.print("Measurement Delay Time: ");Serial.println(measurementDelayTime);
  Serial.print("Battery Value: ");Serial.println(analogRead(BATTERY_ADC_PIN));
  delay(measurementDelayTime);
  measurement_no++;
}

