/**
   @file MUX32 LED TEST
   @author jul10199555
   @brief Fade leds test.
   @version 0.1
   @date 2024-11-19
   @copyright Copyright (c) 2024
**/
#include <LoRaWan-RAK4630.h> //Click here to get the library: http://librarymanager/All#SX126x
// SYSTEM PINS
#define EN_SNS      25
#define EN_REF      33
#define EN_PER      34

#define PWR_LED     35 // LED_BUILTIN/GRN
#define STAT_LED    36 // LED_CONN/BLUE

#define LED_GREEN   PWR_LED
#define LED_ORANGE  STAT_LED // LED_BLUE

int brightness1 = 0;    // how bright the LED is
int brightness2 = 255;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

// the setup routine runs once when you press reset:
void setup() {
  pinMode(EN_SNS, OUTPUT);
  pinMode(EN_REF, OUTPUT);
  pinMode(EN_PER, OUTPUT);

  digitalWrite(EN_SNS, LOW); // Apaga VCNT
  digitalWrite(EN_REF, LOW); // Apaga VREF
  digitalWrite(EN_PER, LOW); // Apaga VEN(3v3_2)

  delay(300);
  
  // RAK WEIRD RUTINE TO AVOID CRASHING THE AT COMMANDS MCUs
  time_t timeout = millis();
  
  // Initialize Serial for debug output
  Serial.begin(115200);
  
  while(!Serial){
    if((millis() - timeout) < 5000){
      delay(100);
    }else{
      break;
    }
  }
  
  // declare pin 9 to be an output:
  pinMode(PWR_LED, OUTPUT);
  pinMode(STAT_LED, OUTPUT);
  
  Serial.println("Welcome! UART works! :D");
}

void loop(){
    // set the brightness of pin 9:
  analogWrite(PWR_LED, brightness1);
  analogWrite(STAT_LED, brightness2);

  // change the brightness for next time through the loop:
  brightness1 = brightness1 + fadeAmount;
  brightness2 = brightness2 - fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness1 <= 0 || brightness2 <= 0 || brightness1 >= 255 || brightness2 >= 255)
    fadeAmount = -fadeAmount;
    
  // wait for 50 milliseconds to see the dimming effect
  Serial.print("LED 1 / LED_CON ");
  Serial.print(brightness1);
  Serial.print(" - LED 2 / GRN_LED ");
  Serial.println(brightness2);
  delay(50);
  
}
