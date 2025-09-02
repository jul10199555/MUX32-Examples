#include <Wire.h>
#include <bq25155.h>

/*** MCU MUX 32 PINS ***/
// SYSTEM PINS
#define EN_SNS      25
#define EN_REF      33
#define EN_PER      34

#define PWR_LED     35 // LED_BUILTIN/GRN
#define STAT_LED    36 // LED_CONN/BLUE

#define LED_GREEN   PWR_LED
#define LED_ORANGE  STAT_LED // LED_BLUE

// bq25155
// CHEN pin
#define BQ_CHEN     2  // AIN0
// Charge Enable.
// Drive CE low or leave disconnected to enable charging when VIN is valid.
// Drive CE high to disable charge when VIN is present.
// CE is pulled low internally with 900-kΩ resistor.
// CE has no effect when VIN is not present.

// INT pin
#define BQ_INT  5  // AIN3 (Monitoreo de batería o Fault INT de BMS)
// Interruption pin.
// INT is an open-drain output that signals fault interrupts.
// When a fault occurs, a 128-µs pulse is sent out as an interrupt for the host.
// INT is enabled/disabled using the MASK_INT bit in the control register.

// LPM pin
#define BQ_LPM      20
// Low Power Mode Enable.
// Drive this pin:
// -LOW to set the device in low power mode when powered by the battery.
// -HIGH to allow I2C communication when VIN is not present
// This pin must be driven high to allow I2C communication when VIN is not present.
// LP is pulled low internally with 900-kΩ resistor.
// This pin has no effect when VIN is present

#define INPT_CURRENT_mA  500
#define BAT_VOLTAGE_mV 4170
#define CHG_CURRENT_uA  250000
#define PCHG_CURRENT_uA  25000
#define SAFETY_TIMER  15

bq25155 charger;

uint16_t VBAT = 0;

bool is_Vin_PG = false, is_PG_En = false, is_CHG_DONE = false;

void setup() {
  pinMode(EN_SNS, OUTPUT);
  pinMode(EN_REF, OUTPUT);
  pinMode(EN_PER, OUTPUT);

  digitalWrite(EN_SNS, LOW); // Apaga VCNT
  digitalWrite(EN_REF, LOW); // Apaga VREF
  digitalWrite(EN_PER, HIGH); // Enciende VEN(3v3_2)

  delay(300);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_ORANGE, HIGH);

  // RAK WEIRD RUTINE TO AVOID CRASHING THE AT COMMANDS MCUs
  time_t timeout = millis();

  Serial.begin(115200);

  while (!Serial) {
    if ((millis() - timeout) < 5000) {
      delay(100);
    } else {
      break;
    }
  }

  // Initialize with control pins: CHEN, INT, LPM
  if (!charger.begin(BQ_CHEN, BQ_INT, BQ_LPM)) {
    Serial.println("bq25155 not found!");
    while (1);
  } else {
    Serial.print(charger.getDeviceIDString());
    Serial.println(" found!");
  }

  // Configure charging profile
  if (charger.initCHG(
        BAT_VOLTAGE_mV,   // Target charge voltage in mV
        true,     // Enable fast charging
        CHG_CURRENT_uA,   // Charge current in uA
        PCHG_CURRENT_uA,  // Precharge current in uA
        INPT_CURRENT_mA,  // Input current limit in mA
        SAFETY_TIMER      // Safety timer in tenths of an hour (e.g., 15 = 1.5h, 30 = 3h)
      )) {
    charger.setPGasGPOD();
    charger.EnablePG();

    Serial.println("Charger configured.");
  } else {
    Serial.println("Error while configuring the charger.");
  }

}

void loop() {
  VBAT = charger.readVBAT(1);
  is_CHG_DONE = charger.is_CHARGE_DONE();
  is_Vin_PG = charger.is_VIN_PGOOD();
  is_PG_En = charger.isPGEnabled();

  if (is_Vin_PG != is_PG_En) {
    if (is_Vin_PG) {
      if (!is_CHG_DONE)
        charger.EnableCharge();
      charger.EnablePG();
    } else {
      if (is_CHG_DONE)
        charger.DisableCharge();
      charger.DisablePG();
    }
  }

  if (is_CHG_DONE || VBAT > BAT_VOLTAGE_mV) {
    Serial.println("Charge complete!");
    Serial.println("Battery Voltage: " + String(VBAT));
  } else {
    if (is_Vin_PG) {
      Serial.println("INPUT Voltage Good!");
    } else {
      Serial.println("INPUT Voltage not Good! :(");
    }

    Serial.print("Input Voltage: ");
    Serial.println(charger.readVIN(1));
    Serial.print("Input Current: ");
    Serial.println(charger.readIIN(2));

    Serial.println("Battery Voltage: " + String(VBAT));
    Serial.print("Charging Current: ");
    Serial.println(charger.readICHG(2));
  }

  Serial.println("--------------------");

  delay(2000);
}
