/**
   @file I2C SCANNER FOR MUX32
   @author jul10199555
   @brief I2C Address finder.
   @version 0.2
   @date 2025-05-21
   @copyright Copyright (c) 2025
**/
#include <Wire.h>

// SYSTEM PINS
#define EN_SNS      25
#define EN_REF      33
#define EN_PER      34

#define PWR_LED     35 // LED_BUILTIN/GRN
#define STAT_LED    36 // LED_CONN/BLUE

#define LED_GREEN   PWR_LED
#define LED_ORANGE  STAT_LED // LED_BLUE

void setup(){
  pinMode(EN_SNS, OUTPUT);
  pinMode(EN_REF, OUTPUT);
  pinMode(EN_PER, OUTPUT);

  digitalWrite(EN_SNS, LOW); // Apaga VCNT
  digitalWrite(EN_REF, LOW); // Apaga VREF
  digitalWrite(EN_PER, HIGH); // Enciende VEN(3v3_2)

  delay(300);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_ORANGE, LOW);

  Wire.begin();

  Serial.begin(115200);
  while(!Serial);  // Wait for serial monitor
  Serial.println("\nI2C Scanner");
}

void loop(){
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ){
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if(error == 0){
      Serial.print("I2C device found at address 0x");
      if(address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }else if(error==4){
      Serial.print("Unknown error at address 0x");
      if(address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if(nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
