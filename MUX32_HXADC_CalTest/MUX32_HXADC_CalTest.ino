/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

/*
   Settling time (number of samples) and data filtering can be adjusted in the config.h file
   For calibration and storing the calibration value in eeprom, see example file "Calibration.ino"

   The update() function checks for new data and starts the next conversion. In order to acheive maximum effective
   sample rate, update() should be called at least as often as the HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS.
   If you have other time consuming code running (i.e. a graphical LCD), consider calling update() from an interrupt routine,
   see example file "Read_1x_load_cell_interrupt_driven.ino".

   This is an example sketch on how to use this library
*/
#include <LoRaWan-RAK4630.h> //Click here to get the library: http://librarymanager/All#SX126x
#include <HX711_ADC.h>

// SYSTEM PINS
#define EN_SNS      25
#define EN_REF      33
#define EN_PER      34

#define PWR_LED     35 // LED_BUILTIN/GRN
#define STAT_LED    36 // LED_CONN/BLUE

#define LED_GREEN   PWR_LED
#define LED_ORANGE  STAT_LED // LED_ORANGE

// HX711
#define HX_DAT      15
#define HX_SCK      16
#define HX_DRT      17

//HX711 constructor:
HX711_ADC LoadCell(HX_DAT, HX_SCK);

unsigned long t = 0;
unsigned long stabilizingtime = 1000; // tare preciscion can be improved by adding a few seconds of stabilizing time
const int serialPrintInterval = 250; // increase value to slow down serial print activity

bool _tare = true; //set to false if not tare is required at beginning
bool set80SPS = false; // DR-> false=10Hz, true=80Hz

float lastSPS = 0.0;
float wght_g = 0.0, load_N = 0.0; // Store vals

// Serial menu constants
const char CALIBRATION_MODE = 'c';
const char TARE_MODE = 't';
const char MAIN_MENU_MODE = 'm';
const char EXIT_MODE = 'x';
const int MIN_CALFACTOR = 0;
const int MAX_CALFACTOR = 3000;

// Global variables
char smode = MAIN_MENU_MODE;

float CALIBRATION_STEP = 0.1;
float calibrationValue = 1104.10; // start calibration value

void setup() {
  pinMode(EN_SNS, OUTPUT);
  pinMode(EN_REF, OUTPUT);
  pinMode(EN_PER, OUTPUT);

  digitalWrite(EN_SNS, LOW); // Apaga VCNT
  digitalWrite(EN_REF, LOW); // Apaga VREF
  digitalWrite(EN_PER, HIGH); // Enciende VEN(3v3_2)

  delay(300);

  pinMode(HX_DRT, OUTPUT);
  digitalWrite(HX_DRT, set80SPS ? HIGH : LOW);
  
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);
  
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_ORANGE, HIGH);
  
  // RAK WEIRD RUTINE TO AVOID CRASHING THE AT COMMANDS MCUs
  time_t timeout = millis();
  
  Serial.begin(115200);
  
  while(!Serial){
    if((millis() - timeout) < 5000)
      delay(100);
    else
      break;
  }

  Serial.println("Welcome to HX711_ADC Test! :D");

  LoadCell.begin();
  
  LoadCell.setReverseOutput(); // Comment if arrow points down
  LoadCell.start(stabilizingtime, _tare);
  
  if(LoadCell.getTareTimeoutFlag())
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  else{
    LoadCell.setCalFactor(calibrationValue); // set calibration factor (float)
    Serial.println("Startup is complete");
  }

  while(!LoadCell.update());
  
  Serial.print("Calibration value: ");
  Serial.println(LoadCell.getCalFactor());

  Serial.print("HX711 measured conversion time ms: ");
  Serial.println(LoadCell.getConversionTime());

  lastSPS = LoadCell.getSPS();
  
  Serial.print("HX711 measured sampling rate HZ: ");
  Serial.println(lastSPS);

  Serial.print("HX711 measured settling time ms: ");
  Serial.println(LoadCell.getSettlingTime());

  Serial.println("Note that the settling time may increase significantly if you use delay() in your sketch!");

  if(lastSPS < 7)
    Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
  else if (lastSPS > 100)
    Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");
}

void loop(){
  // check for new data/start next conversion:
  if (LoadCell.update()) {
    wght_g = LoadCell.getData();
    load_N = wght_g * 0.00980665;
  }
  
  // get smoothed value from the dataset:
  if (millis() > t + serialPrintInterval) {
    Serial.print("Load_cell units: ");
    Serial.print(wght_g, 3);
    Serial.print(" g, Load_cell units: ");
    Serial.print(load_N, 3);
    Serial.println(" N");
    t = millis();
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available()) {
    char chrs = Serial.read();
    // Main menu
    if (smode == MAIN_MENU_MODE) {
      if (chrs == CALIBRATION_MODE) {
        smode = CALIBRATION_MODE;
        Serial.println("Entering Calibration Mode.");
        
      } else if (chrs == TARE_MODE) {
        LoadCell.tareNoDelay();
        Serial.println("Tare requested.");
      }
      
    } else if (smode == CALIBRATION_MODE) {
      if (chrs == '+' || chrs == 'a')
        calibrationValue += CALIBRATION_STEP;
      else if (chrs == '-' || chrs == 'z')
        calibrationValue -= CALIBRATION_STEP;
      
      LoadCell.setCalFactor(calibrationValue); // Set the calibration factor
      
      Serial.print("Cal. Value: ");
      Serial.println(calibrationValue, 2);
      
      if(chrs == EXIT_MODE){
        smode = MAIN_MENU_MODE;
        Serial.println("Exiting Calibration Mode. Returning to Main Menu.");
      }
    }
  }

  // check if requested tare is complete:
  if(LoadCell.getTareStatus())
    Serial.println("Tare complete");
}
