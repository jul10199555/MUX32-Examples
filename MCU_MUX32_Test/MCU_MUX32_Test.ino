/**
   @file MCU MUX 32 TEST PROGRAM
   @author jul10199555
   @brief Board, sensors and leds test.
   @version 0.1
   @date 2025-05-21
   @copyright Copyright (c) 2025
**/
#include <LoRaWan-RAK4630.h> // Click here to get the library: http://librarymanager/All#SX126x

#include <Wire.h>
#include <bq25155.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h> // Click to install library: http://librarymanager/All#Adafruit_BME680
#include <UVlight_LTR390.h> // Click here to get the library: http://librarymanager/All#RAK12019_LTR390

#include <HX711_ADC.h>

/*** ADC config ***/
#define ADC_bits 12

const uint32_t Vsyst = 3.3e6; // System Voltage
const uint32_t DIVR1 = 1e6; // Known upper resistance of voltage divider (1M)
const uint32_t DIVR2 = 1.5e6; // Known lower resistance of voltage divider (1.5M)

const uint16_t ADbits = (ADC_bits==14 ? 16384 :(ADC_bits==13 ? 8192 :(ADC_bits==12 ? 4096 :(ADC_bits==11 ? 2048 : 1024))));

float ADCons = Vsyst / ADbits;

//Real vbat
// Vread * (R1+R2)/R2
float RCons = 5.00 / 3.00; // (DIVR1 + DIVR2) / DIVR2;

uint16_t rawbat = 0;
uint32_t volbat = 0, realbat = 0;

/*** LEDa fade settings ***/
int bgt1 = 0;    // how bright the LED is
int bgt2 = 255;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

String leds, envr, HXADC, ltrs, Batt;

/*** MCU MUX 32 PINS ***/
// SYSTEM PINS
#define EN_SNS      25
#define EN_REF      33
#define EN_PER      34

#define PWR_LED     35 // LED_BUILTIN/GRN
#define STAT_LED    36 // LED_CONN/BLUE

#define LED_GREEN   PWR_LED
#define LED_ORANGE  STAT_LED // LED_BLUE

// HX711
#define HX_DAT      15
#define HX_SCK      16
#define HX_DRT      17

// SHIMADZU ADC READINGS
#define SHMDZ_LOAD  4 // AIN2 (No usar si se mide carga con shimadzu)
#define SHMDZ_DISP  31 // AIN7 (No usar si se mide carga con shimadzu)

// SPI
// J8
// GND - 3V3_2 - 3VO - DP_CS - DP_RST - DP_DC - SCK - MOSI
// GND - 3V3_2 -  NC -  CS3  - DP_RST - MISO  - SCK - MOSI

//#define MISO      29  // AIN5 (No usar como AIN si SPI está en uso)
//#define SCK       3   // AIN1 (No usar como AIN si SPI está en uso)
//#define MOSI      30  // AIN6 (No usar como AIN si SPI está en uso)

// ADS1220 ADC
#define AD_CS       9
#define AD_DRDY     10

// MCP23S17 GPIO
#define MC_CS       21

//SSD1327Z OLED
#define CS_3        19
#define DP_CS       26
#define DP_DC       28 // AIN4

// microSD card
#define SD_CS       24

// I2C
//#define I2C_SDA   13
//#define I2C_SCL   14

// bq25155

// CHEN pin = 2
#define BQ_CHEN     2  // AIN0
// Charge Enable.
// Drive CE low or leave disconnected to enable charging when VIN is valid.
// Drive CE high to disable charge when VIN is present.
// CE is pulled low internally with 900-kΩ resistor.
// CE has no effect when VIN is not present.

// INT pin = 5
#define VBAT_BQINT  5  // AIN3 (Monitoreo de batería o Fault INT de BMS)
// Interruption pin.
// INT is an open-drain output that signals fault interrupts. 
// When a fault occurs, a 128-µs pulse is sent out as an interrupt for the host.
// INT is enabled/disabled using the MASK_INT bit in the control register.

// LPM pin = 20
#define BQ_LPM      20 // HIGH to allow I2C communication when VIN is not present
// Low Power Mode Enable.
// Drive this pin low to set the device in low power mode when powered by the battery. 
// This pin must be driven high to allow I2C communication when VIN is not present.
// LP is pulled low internally with 900-kΩ resistor.
// This pin has no effect when VIN is present

/*** SENSORS INIT ***/
// AD5241A 0X2C AD1->GND AD0->GND
// AD5241B 0X2E AD1->VCC AD0->GND
// LTR390U 0X53 FIXED
// bq25155 0X6B FIXED
// BME680A 0X76 SDO->GND

bq25155 charger;

// BME680
byte BME_ADDR = 0x76;
Adafruit_BME680 bme;

// Might need adjustments
#define SEALEVELPRESSURE_HPA (1010.0)

// LTR390
UVlight_LTR390 ltr = UVlight_LTR390();

/*
   For device under tinted window with coated-ink of flat transmission rate at 400-600nm wavelength,
   window factor  is to  compensate light  loss due to the  lower  transmission rate from the coated-ink.
      a. WFAC = 1 for NO window / clear window glass.
      b. WFAC >1 device under tinted window glass. Calibrate under white LED.
*/

//HX711 scale
HX711_ADC Scale(HX_DAT, HX_SCK);

unsigned long t = 0;
const int serialPrintInterval = 500;  //increase value to slow down serial print activity
unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
boolean _tare = true;                 //set this to false if you don't want tare to be performed in the next step
float calibrationValue = 696.0;       // Value of calibration

void setup(){
  pinMode(EN_SNS, OUTPUT);
  pinMode(EN_REF, OUTPUT);
  pinMode(EN_PER, OUTPUT);

  digitalWrite(EN_SNS, LOW); // Apaga VCNT
  digitalWrite(EN_REF, LOW); // Apaga VREF
  digitalWrite(EN_PER, HIGH); // Enciende VEN(3v3_2)

  delay(300);

  pinMode(HX_DRT, OUTPUT);
  digitalWrite(HX_DRT, LOW); // DR-> LOW=10Hz, HIGH=80Hz
  
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);
  
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_ORANGE, HIGH);
  
  // RAK WEIRD RUTINE TO AVOID CRASHING THE AT COMMANDS MCUs
  time_t timeout = millis();
  
  Serial.begin(115200);
  
  while(!Serial){
    if((millis() - timeout) < 5000){
      delay(100);
    }else{
      break;
    }
  }

  Serial.println("Welcome! UART works! :D");
  
  Wire.begin();
  
  // Initialize control pins: CHEN, INT, LPM
  if(!charger.begin(2, 5, 20)){
    Serial.println("bq25155 not found!");
    while (1);
  }
  
  if(!bme.begin(BME_ADDR)){
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    return;
  }else{
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }

  if(!ltr.init()){
    Serial.println("Couldn't find LTR sensor!");
    while(1)
      delay(10);
  }else{
    Serial.println("Found LTR390 sensor!");
    
    //if set LTR390_MODE_ALS, get ambient light data, if set LTR390_MODE_UVS,get ultraviolet light data.
    ltr.setMode(LTR390_MODE_ALS); //LTR390_MODE_UVS
    
    if(ltr.getMode() == LTR390_MODE_ALS)
      Serial.println("In ALS mode");
    else
      Serial.println("In UVS mode");

    ltr.setGain(LTR390_GAIN_3);

    Serial.print("Gain : ");
    
    switch(ltr.getGain()){
      case LTR390_GAIN_1:
        Serial.println(1);
        break;
      case LTR390_GAIN_3:
        Serial.println(3);
        break;
      case LTR390_GAIN_6:
        Serial.println(6);
        break;
      case LTR390_GAIN_9:
        Serial.println(9);
        break;
      case LTR390_GAIN_18:
        Serial.println(18);
        break;
      default:
        Serial.println("Failed to set gain");
        break;
    }
    
    ltr.setResolution(LTR390_RESOLUTION_16BIT);
    
    Serial.print("Integration Time (ms): ");
    switch(ltr.getResolution()){
      case LTR390_RESOLUTION_13BIT:
        Serial.println(13);
        break;
      case LTR390_RESOLUTION_16BIT:
        Serial.println(16);
        break;
      case LTR390_RESOLUTION_17BIT:
        Serial.println(17);
        break;
      case LTR390_RESOLUTION_18BIT:
        Serial.println(18);
        break;
      case LTR390_RESOLUTION_19BIT:
        Serial.println(19);
        break;
      case LTR390_RESOLUTION_20BIT:
        Serial.println(20);
        break;
      default:
        Serial.println("Failed to set Integration Time");
        break;
    }
  
    ltr.setThresholds(100, 1000); //Set the interrupt output threshold range for lower and upper.
    
    if(ltr.getMode() == LTR390_MODE_ALS)
      ltr.configInterrupt(true, LTR390_MODE_ALS); //Configure the interrupt based on the thresholds in setThresholds()
    else
      ltr.configInterrupt(true, LTR390_MODE_UVS);
  }

  Scale.begin();

  //Scale.setReverseOutput();

  Scale.start(stabilizingtime, _tare);
  
  if(Scale.getTareTimeoutFlag())
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  else{
    Scale.setCalFactor(calibrationValue); // set calibration factor (float)
    Serial.println("Startup is complete");
  }
  
  while(!Scale.update());
  
  Serial.print("HX711 Calibration value: ");
  Serial.println(Scale.getCalFactor());

  Serial.print("HX711 measured conversion time ms: ");
  Serial.println(Scale.getConversionTime());
  
  Serial.print("HX711 measured sampling rate HZ: ");
  Serial.println(Scale.getSPS());

  Serial.print("HX711 measured settlingtime ms: ");
  Serial.println(Scale.getSettlingTime());

  Serial.println("Note that the settling time may increase significantly if you use delay() in your sketch!");

  if(Scale.getSPS() < 7)
    Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
  else if (Scale.getSPS() > 100)
    Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");

  analogReadResolution(ADC_bits);
}

void loop(){
  static boolean newDataReady = 0;
  
  bme.performReading();

  // check for new data/start next conversion:
  if(Scale.update())
    newDataReady = true;

  // get smoothed value from the dataset:
  if(newDataReady){
    if(millis() > t + serialPrintInterval){
      HXADC = "HX711 RAW:"+ String(Scale.getData()) +", HX711 weight:"+ String(Scale.getData()) +" g";
      
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if(Serial.available() > 0){
    char inByte = Serial.read();
    if(inByte == 't')
      Scale.tareNoDelay();
  }

  // check if last tare operation is complete:
  if(Scale.getTareStatus() == true)
    Serial.println("Tare complete");
  
  rawbat = analogRead(VBAT_BQINT);
  volbat = rawbat * ADCons;
  realbat = volbat * RCons;
  Serial.println("ADCons: " + String(ADCons) + ", RCons: " + String(RCons));

  // set the brightness of pin 9:
  analogWrite(LED_GREEN, bgt1);
  analogWrite(LED_ORANGE, bgt2);

  // change the brightness for next time through the loop:
  bgt1 = bgt1 + fadeAmount;
  bgt2 = bgt2 - fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if(bgt1 <= 0 || bgt2 <= 0 || bgt1 >= 255 || bgt2 >= 255)
    fadeAmount = -fadeAmount;
    
  // wait for 200 milliseconds to see the dimming effect
  leds = "LED 1: " + String(bgt1) + ", LED 2: " + String(bgt2);
  envr = "Temp: " + String(bme.temperature) + ", Hum: " + String(bme.humidity) + ", Presr: " + String(bme.pressure / 100.0) + ", Gas: " + String(bme.gas_resistance / 1000.0);
  if(ltr.newDataAvailable()){
    if(ltr.getMode() == LTR390_MODE_ALS)
      ltrs = "Lux: " + String(ltr.getLUX()) + ", Raw ALS: " + String(ltr.readALS());
    else
      ltrs = "UVX: " + String(ltr.getUVI()) + ", Raw UVS: " + String(ltr.readUVS());
  }
  
  Batt = "VBAT: " + String(realbat) + ", VADC: " + String(volbat) + ", RAW: " + String(rawbat);
  
  Serial.println(" ---- ");
  Serial.println(leds);
  Serial.println(" ---- ");
  Serial.println(envr);
  Serial.println(" ---- ");
  Serial.println(HXADC);
  Serial.println(" ---- ");
  Serial.println(ltrs);
  Serial.println(" ---- ");
  Serial.println(Batt);
  Serial.println(" ---- ");
  
  delay(500);
}
