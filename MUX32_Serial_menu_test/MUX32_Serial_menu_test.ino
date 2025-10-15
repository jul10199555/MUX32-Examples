/*
 * @brief         MUX32 Serial Dummy
 * @note          Implements a low power system for 
 *                Multiplexing, Reading, Processing
 *                and saving data coming from a 
 *                21p CNT_GFW sensor
 * @version       1.0
 * @creation date 2025-07-07
 * @updated date  2025-10-15
 * @author        by jul10199555
 * @copyright     Copyright (c) 2025
 * 
 */

#include <stdint.h>

#include <LoRaWan-RAK4630.h> // Library: http://librarymanager/All#SX126x
#include <bq25155.h>

#include <Adafruit_MCP23X17.h>
//#include <ADS1220_WE.h>      // Library: http://librarymanager/All#ADS1220_WE
#include <AD5242.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h> // Click to install library: http://librarymanager/All#Adafruit_BME680
#include <UVlight_LTR390.h> // Click here to get the library: http://librarymanager/All#RAK12019_LTR390

#include <HX711_ADC.h>

#include <TimeLib.h>

#include <SD.h>

//----------------------------------------------------------------------------------------------------------------------
// System Pins
//----------------------------------------------------------------------------------------------------------------------
#define EN_SNS      25
#define EN_REF      33
#define EN_PER      34

#define PWR_LED     35 // LED_BUILTIN/GRN
#define STAT_LED    36 // LED_CONN/BLUE

#define LED_GREEN   PWR_LED
#define LED_ORANGE  STAT_LED // LED_BLUE

#define GNLED_BRIGHTNESS   18
#define OGLED_BRIGHTNESS   72

//----------------------------------------------------------------------------------------------------------------------
// Date & Time parameters
//----------------------------------------------------------------------------------------------------------------------

// Default if microSD has not time yet.
int INIT_Y  = 2025;
int INIT_MO = 10;
int INIT_D  = 15;
int INIT_H  = 3;
int INIT_MI = 0;
int INIT_S  = 0;

uint32_t save_dtt_sd = 30;

int Y, Mo, D, H, Mi, S;
uint16_t lastSavedYMD = 0; // YYYY*10000 + MM*100 + DD
time_t lastPrint = 0;

//----------------------------------------------------------------------------------------------------------------------
// nRF ADC parameters
//----------------------------------------------------------------------------------------------------------------------
#define ADC_bits 12

const uint32_t Vsyst = 3.3e6;
const uint16_t ADbits = (ADC_bits==14 ? 16384 :(ADC_bits==13 ? 8192 :(ADC_bits==12 ? 4096 :(ADC_bits==11 ? 2048 : 1024))));

float ADCons = Vsyst / ADbits;

//----------------------------------------------------------------------------------------------------------------------
// SPI Peripherals
//----------------------------------------------------------------------------------------------------------------------
//#define MISO      29  // AIN5 (No usar como AIN si SPI está en uso)
//#define SCK       3   // AIN1 (No usar como AIN si SPI está en uso)
//#define MOSI      30  // AIN6 (No usar como AIN si SPI está en uso)

//----------------------------------------------------------------------------------------------------------------------
// MCP23S17 (IC6) + (ADG732 [MUX1] (IC9) + ADG732 [MUX2] (IC10))
//----------------------------------------------------------------------------------------------------------------------
#define MC_CS       21

Adafruit_MCP23X17 mcp;

// MCP_GPIO -> PORT -> ADG732 PIN -> IC PIN
// 0        -> GPA0 -> MUX2_EN    -> 21
//             ....
// 7        -> GPA7 -> MUX2_A0    -> 28
// 8        -> GPB0 -> MUX1_A0    -> 1
//             ....
// 15       -> GPB7 -> MUX1_EN    -> 8

// Define Port A (ADG732 [MUX2]) bitmasks
#define GPA_EN  (1 << 0)
#define GPA_WR  (1 << 1)
#define GPA_CS  (1 << 2)
#define GPA_A4  (1 << 3)
#define GPA_A3  (1 << 4)
#define GPA_A2  (1 << 5)
#define GPA_A1  (1 << 6)
#define GPA_A0  (1 << 7)

// Define Port B (ADG732 [MUX1]) bitmasks
#define GPB_A0  (1 << 0)
#define GPB_A1  (1 << 1)
#define GPB_A2  (1 << 2)
#define GPB_A3  (1 << 3)
#define GPB_A4  (1 << 4)
#define GPB_CS  (1 << 5)
#define GPB_WR  (1 << 6)
#define GPB_EN  (1 << 7)

bool M1A0, M1A1, M1A2, M1A3, M1A4;
bool M2A0, M2A1, M2A2, M2A3, M2A4;

uint8_t portA = 0, portB = 0;
uint16_t gpioAB = 0, MCP_delay = 1;

// ##-GFW Sensors Channels
#define ADS_READINGS 40

// Array of the 40 mappings
struct MuxPair { uint8_t ch1, ch2; };
const MuxPair GFW_chs[ADS_READINGS] = {
  {1, 1},  {1, 3},  {2, 4},  {3, 1},  {3, 5},  {4, 2},  {4, 6},  {5, 3},
  {5, 7},  {6, 4},  {6, 8},  {7, 5},  {7, 9},  {8, 6},  {8,10},  {9, 7},
  {9,11},  {10, 8}, {10,12}, {11, 9}, {11,13}, {12,10}, {12,14}, {13,11},
  {13,15}, {14,12}, {14,16}, {15,13}, {15,17}, {16,14}, {16,18}, {17,15},
  {17,19}, {18,16}, {18,20}, {19,17}, {19,21}, {20,18}, {21,19}, {21,21}
};

//----------------------------------------------------------------------------------------------------------------------
// ADS1220 (IC7)
//----------------------------------------------------------------------------------------------------------------------
#define AD_CS       9
#define AD_DRDY     10

//ADS1220_WE ads = ADS1220_WE(AD_CS, AD_DRDY);

// Store readings in microvolts as 32-bit integers
int32_t readings_uV[ADS_READINGS];

float ADSVRef_V = 0.0, ADSReading_mV = 0.0, ADSTemp = 25.0;

//----------------------------------------------------------------------------------------------------------------------
// microSD card (J7)
//----------------------------------------------------------------------------------------------------------------------
#define SD_CS       24

File DataSD;
File DTTSettings;

const char* setts_file = "/settings.txt";
static uint32_t g_lastSdSaveMs   = 0;

bool isSDins = false, is1stIt = true;

uint8_t fid = 1;
uint32_t uSD_row = 1;

//----------------------------------------------------------------------------------------------------------------------
// SSD1327Z OLED (J8)
// GND - 3V3_2 - 3VO - DP_CS - DP_RST - DP_DC - SCK - MOSI
// GND - 3V3_2 -  NC -  CS3  - DP_RST - MISO  - SCK - MOSI
//----------------------------------------------------------------------------------------------------------------------
#define CS_3        19
#define DP_CS       26
#define DP_DC       28 // AIN4
#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128

//----------------------------------------------------------------------------------------------------------------------
// I2C Peripherals
//----------------------------------------------------------------------------------------------------------------------
//#define I2C_SDA   13
//#define I2C_SCL   14

//----------------------------------------------------------------------------------------------------------------------
// bq25155 (IC4)
//----------------------------------------------------------------------------------------------------------------------
// bq25155 0X6B FIXED

// Charge Enable (CHEN) pin
// Drive CE low or leave disconnected to enable charging when VIN is valid.
// Drive CE high to disable charge when VIN is present.
// CE is pulled low internally with 900-kΩ resistor.
// CE has no effect when VIN is not present.
#define BQ_CHEN     2  // AIN0

// Interruption (INT) pin.
// INT is an open-drain output that signals fault interrupts. 
// When a fault occurs, a 128-µs pulse is sent out as an interrupt for the host.
// INT is enabled/disabled using the MASK_INT bit in the control register.
#define BQ_INT  5  // AIN3 (Monitoreo de batería o Fault INT de BMS)

// Low Power Mode (LPM) Enable pin.
// Drive this pin low to set the device in low power mode when powered by the battery. 
// This pin must be driven high to allow I2C communication when VIN is not present.
// LP is pulled low internally with 900-kΩ resistor.
// This pin has no effect when VIN is present
#define BQ_LPM      20 // HIGH to allow I2C communication when VIN is not present

bq25155 charger;

#define INPT_CURRENT_mA  500
#define BAT_VOLTAGE_mV 4175
#define CHG_CURRENT_uA  100000
#define PCHG_CURRENT_uA  15000
#define SAFETY_TIMER  15

bool is_CHG_Set = false, is_CHG_DONE = false;
bool is_Vin_PG = false, is_PG_En = false;

uint16_t INVo = 0, VBAT = 0;
uint32_t INCu = 0, ICHG = 0;

//----------------------------------------------------------------------------------------------------------------------
// AD5242#1 (IC11) + AD5242#2 (IC12)
//----------------------------------------------------------------------------------------------------------------------
// AD5241A 0X2C AD1->GND AD0->GND
// AD5241B 0X2E AD1->VCC AD0->GND

#define MAX_RESISTANCE_OHMS 1000000  // 1 MΩ
#define MAX_POT_STEPS 255.0

AD5242 POT1(0x2C);  //  AD1->GND AD0->GND
AD5242 POT2(0x2E);  //  AD1->VCC AD0->GND

//----------------------------------------------------------------------------------------------------------------------
// BME680 (IC13)
//----------------------------------------------------------------------------------------------------------------------
// BME680A 0X76 SDO->GND
uint8_t BME_ADDR = 0x76;
#define SEALEVELPRESSURE_HPA (1010.0)

Adafruit_BME680 bme;

uint32_t BME_press = 0, BME_gas = 0;
float BME_temp = 0.0, BME_humr = 0.0;

//----------------------------------------------------------------------------------------------------------------------
// LTR390 (IC14)
//----------------------------------------------------------------------------------------------------------------------
// LTR390U 0X53 FIXED
UVlight_LTR390 ltr = UVlight_LTR390();

uint32_t LTR_XXS = 0;
float LTR_IDX = 0.0;

//----------------------------------------------------------------------------------------------------------------------
// HX711 (IC8)
//----------------------------------------------------------------------------------------------------------------------
#define HX_DAT      15
#define HX_SCK      16
#define HX_DRT      17

HX711_ADC LoadCell(HX_DAT, HX_SCK); // Constructor

unsigned long t = 0;
unsigned long HX_stabilize_t = 1000; // tare preciscion can be improved by adding a few seconds of stabilizing time
const int serialPrintInterval = 250; // increase value to slow down serial print activity

bool HX_iniTare = true; //set to false if not tare is required at beginning
bool set80SPS = false; // DR-> false=10Hz, true=80Hz

uint32_t settTime = 0;
float lastSPS = 0.0, convTime = 0.0;
float HX_weight = 0.0, HX_load = 0.0; // Store vals

uint16_t HX_Cell_Capacity = 2000; // Load Cell Capacity in g
float calibrationValue = 1104.10; // start calibration value

// Shimadzu ADC Readings
#define SHMDZ_LOAD  4 // AIN2 (No usar si se mide carga con shimadzu)
#define SHMDZ_DISP  31 // AIN7 (No usar si se mide carga con shimadzu)

// Shimadzu resistors
uint32_t VD_PU = 100000, VD_PD = 150000;

//----------------------------------------------------------------------------------------------------------------------
// App Timings
//----------------------------------------------------------------------------------------------------------------------
const unsigned long DEFAULT_DATA_INTERVAL = 1000;  // ms

//----------------------------------------------------------------------------------------------------------------------
// System Command Codes
//----------------------------------------------------------------------------------------------------------------------
// Board Menu/Modes
#define MUX32_INIT_SETTINGS   (0x01)
#define MUX32_SEND_DATA       (0x02)
#define MUX32_CONFIG_DATA     (0x03)

#define MUX32_OPT_NA          'N'

#define MUX_PAY_DEL           ','

enum Mode {
  MODE_IDLE,
  MODE_CONFIG,
  MODE_DATAREQ
};

Mode currentMode = MODE_IDLE;

// Board Models
#define MUX08_A               "8"
#define MUX08_B               "MUX08"

#define MUX32_A               "32"
#define MUX32_B               "MUX32"

// Machines
#define MUX_SHIMADZU          'S'
#define MUX_MTS               'T'
#define MUX_MINI_SMDZ         'M'
#define MUX_FGR_BEND          'F'
#define MUX_OAX_STR           'O'

// Materials
#define CNT_GFW               'C'
#define GS_GFW                'G'
#define MWCNT                 'M'
#define MXENE                 'X'
#define CX_ALPHA              'A'

// Global Commands
#define MUX32_ALIVE           '0'
#define MUX32_SETUP           '1'
#define MUX32_CH_EN           "ce"
#define MUX32_CH_DIS          "cd"
#define MUX32_CH_VBAT         "vb"
#define MUX32_CH_STAT         "cs"
#define MUX32_DT_STAT         "dt"
#define MUX32_TARE_HX         't'

// OnTest Commands
#define MUX32_MANUAL_REQ      'r'
#define MUX32_PAUSE_TEST      'p'
#define MUX32_END_OF_TEST     "end"

// Board Errors Codes
#define MUX32_SUCCESS         (0x00)
#define MUX32_INVALID_COMMAND (0x01)
#define MUX32_INVALID_PAYLOAD (0x02)

// Peripheral Errors
#define bq25155_ERROR_INIT    (0x03)
#define ADS1220_ERROR_INIT    (0x04)

#define POT1_ERROR_INIT       (0x05)
#define POT2_ERROR_INIT       (0x06)
#define MCP23_ERROR_INIT      (0x07)

#define HX711_ERROR_INIT      (0x08)

// Sensors & Memory Warnings
#define BME_WARNING           (0x09)
#define LTR_WARNING           (0x0A)

#define DTT_WARNING           (0x0E)
#define SDCARD_WARNING        (0x0F)

//----------------------------------------------------------------------------------------------------------------------
// System State Variables
//----------------------------------------------------------------------------------------------------------------------
bool paused = false;
unsigned long dataInterval = DEFAULT_DATA_INTERVAL;
unsigned long sensInterval = DEFAULT_DATA_INTERVAL;
unsigned long lastDataTime = 0, lastSensTime = 0;

// --- CONFIGURATION VARIABLES (largest superset needed) ---

uint8_t MUXBoard = 0;

// Sensors configuration
// BME
bool TempEn = false;
//1 = C
//2 = F
uint8_t TempUnits = 1;
bool RHumEn = false;
bool PresEn = false;
//1 = hPa
//2 = mBar
//3 = mmHg
uint8_t PresUnits = 1;
bool GasEn = false;
//1 = Kohms
//2 = TVoC
uint8_t GasUnits = 1;
// LTR
bool LUXEn = false;
//1 = ALS
//2 = UVS
uint8_t LUXType = 1;
uint8_t LUXBits = 16;

// Test settings
char machineType = 0;
uint32_t totalCycles = 0;
uint16_t nominalRPM = 0;

// Variation 1 displacement
bool dispAvailable = false;
uint8_t dispMaxVoltage = 0;
uint16_t maxDistance = 0;
//1 = mm
//2 = cm
//3 = in
uint8_t distanceUnits = 1;

// Variation 1 load
bool loadAvailable = false;
uint8_t loadMaxVoltage = 0;
uint32_t loadCapacity = 0;
//1 = g
//2 = N
//3 = kg
//4 = kN
uint8_t loadUnits = 0;

// Variation 1 external loadcell
bool extLoadAvailable = false;
uint32_t extLoadCapacity = 0;
uint8_t extLoadUnits = 0;

// Variation 2 variable speed
bool varSpeedAvailable = false;
float speedStart = 0, speedEnd = 0, speedStep = 0;

// Variation 2 motor angle
float motorAngle = 0;
bool varAngleAvailable = false;
float angleStart = 0, angleEnd = 0, angleStep = 0;

// Common material/test fields
char materialType = 0;
char testType = 0;
uint16_t matLength = 0, matWidth = 0, matHeight = 0;
bool hasDelam = false;
uint16_t sampleNumber = 0;
uint16_t totalColumns = 0, totalRows = 0;
uint16_t totalChannels = 0;

String channelHeader;

// Variation 3 strain
uint32_t maxStrain = 0;
uint8_t strainUnits = 0;

//----------------------------------------------------------------------------------------------------------------------
// System Helper Functions
//----------------------------------------------------------------------------------------------------------------------
// Split str by delimiter into arr[0..maxParts-1], return actual count
int split(const String &str, char delim, String arr[], int maxParts) {
  int cnt = 0, start = 0, idx;
  while (cnt < maxParts - 1 && (idx = str.indexOf(delim, start)) != -1) {
    arr[cnt++] = str.substring(start, idx);
    start = idx + 1;
  }
  arr[cnt++] = str.substring(start);
  return cnt;
}

// Parse the initial‐configuration payload; return true on success
bool parseConfig(const String &payload) {
  const int MAX_FIELDS = 16;
  String fields[MAX_FIELDS];
  int n = split(payload, MUX_PAY_DEL, fields, MAX_FIELDS);

  if (n < 1) return false;
  
  // Determine MUX board on first field
  String b = fields[0];
  b.trim();
  
  if (b.equals(MUX32_A) || b.equals(MUX32_B) || b.equalsIgnoreCase("32"))
    MUXBoard = 32;
  else if (b.equals(MUX08_A) || b.equals(MUX08_B) || b.equalsIgnoreCase("08"))
    MUXBoard = 8;
  else
    return false;

  // Determine Environmental sensor on second field
  String env[4];
  int sbm = split(fields[1], '_', env, 4);

  if (sbm < 1 || sbm > 4) return false;

  // Trim each token
  for (int i = 0; i < sbm; ++i) env[i].trim();
  
  // --- Temperature ---
  char tm = env[0].length() ? env[0].charAt(0) : MUX32_OPT_NA;
  if (tm == MUX32_OPT_NA)
    TempEn = false;
  else {
    TempEn = true;
    if (env[0].equalsIgnoreCase("C"))      TempUnits = 1; //1 = C
    else if (env[0].equalsIgnoreCase("F")) TempUnits = 2; //2 = F
    else TempEn = false;
  }
  
  // --- Humidity ---
  RHumEn = !(env[1].length() && env[1].charAt(0) == MUX32_OPT_NA);
  
  // --- Pressure ---
  char ps = env[2].length() ? env[2].charAt(0) : MUX32_OPT_NA;
  if (ps == MUX32_OPT_NA)
    PresEn = false;
  else {
    PresEn = true;
    if      (env[2].equalsIgnoreCase("hPa"))  PresUnits = 1; //1 = hPa
    else if (env[2].equalsIgnoreCase("mBar")) PresUnits = 2; //2 = mBar
    else if (env[2].equalsIgnoreCase("mmHg")) PresUnits = 3; //3 = mmHg
    else PresEn = false;
  }

  // --- Gas ---
  char gs = env[3].length() ? env[3].charAt(0) : MUX32_OPT_NA;
  if (gs == MUX32_OPT_NA)
    GasEn = false;
  else {
    GasEn = true;
    if      (env[3].equalsIgnoreCase("KOhms")) GasUnits = 1; //1 = KOhms
    else if (env[3].equalsIgnoreCase("TVoC"))  GasUnits = 2; //2 = TVoC
    else GasEn = false;
  }
  
  // Determine LUX sensor on third field
  String lux[2];
  int slx = split(fields[2], '_', lux, 2);
  
  if (slx < 1 || slx > 2) return false;
  
  // Trim each token
  for (int i = 0; i < slx; ++i) lux[i].trim();

  if (slx == 2) {
    LUXEn = true;
    
    if      (lux[0].equalsIgnoreCase("ALS")) LUXType = 1; //1 = ALS
    else if (lux[0].equalsIgnoreCase("UVS")) LUXType = 2; //2 = UVS
    else LUXEn = false;

    LUXBits = (uint8_t) lux[1].toInt();
    
    if (LUXBits < 13)
      LUXBits = 13;
    else if (LUXBits > 20)
      LUXBits = 20;
  } else
    LUXEn = false;
  
  // Determine variation by fourth field
  char m = fields[3].charAt(0);
  
  if ((m==MUX_SHIMADZU || m==MUX_MTS || m==MUX_MINI_SMDZ) && n==14) { // Variation 1
    machineType = m;

    String cy = fields[4];
    cy.trim();
    
    unsigned long tc = cy.toInt();
    if (tc < 1UL || tc > 9999999UL) return false;

    totalCycles = (uint32_t)tc;
    
    // D_/N_ sub‐fields
    String sub5[4];
    split(fields[5], '_', sub5, 4);
    char ds = sub5[0].length() ? sub5[0].charAt(0) : MUX32_OPT_NA;
    
    if (ds == MUX32_OPT_NA)
      dispAvailable = false;
    else {
      dispAvailable = true;
      
      dispMaxVoltage = (uint8_t) sub5[1].toInt();
      maxDistance    = (uint16_t) sub5[2].toInt();
      distanceUnits  = (uint8_t) sub5[3].toInt(); //1 = mm, 2 = cm, 3 = in
    }
    
    // L_/N_ sub‐fields
    String sub6[4];
    split(fields[6], '_', sub6, 4);
    char ld = sub6[0].length() ? sub6[0].charAt(0) : MUX32_OPT_NA;
    
    if (ld == MUX32_OPT_NA)
      loadAvailable = false;
    else {
      loadAvailable = true;
      
      loadMaxVoltage   = (uint8_t) sub6[1].toInt();
      loadCapacity     = (uint32_t) sub6[2].toInt();
      loadUnits        = (uint8_t) sub6[3].toInt(); //1 = g, 2 = N, 3 = kg, 4 = kN
    }

    // H_/N_ sub‐fields
    String sub7[3];
    split(fields[7], '_', sub7, 3);
    char h7 = sub7[0].length() ? sub7[0].charAt(0) : MUX32_OPT_NA;
    
    if (h7 == MUX32_OPT_NA)
      extLoadAvailable = false;
    else {
      extLoadAvailable = true;
      
      extLoadCapacity  = (uint32_t) sub7[1].toInt();
      extLoadUnits     = (uint8_t) sub7[3].toInt(); //1 = g, 2 = N, 3 = kg
    }
    
    materialType     = fields[8].charAt(0);
    testType         = fields[9].charAt(0);
    
    String sub10[3];
    split(fields[10], '_', sub10, 3);
    matLength        = (uint16_t) sub10[0].toInt();
    matWidth         = (uint16_t) sub10[1].toInt();
    matHeight        = (uint16_t) sub10[2].toInt();

    if (fields[11].charAt(0) == MUX32_OPT_NA)
      hasDelam       = false;
    else
      hasDelam       = true;
    
    sampleNumber     = (uint16_t) fields[12].toInt();
    totalChannels    = (uint16_t) fields[13].toInt();

    if(materialType == MWCNT || materialType == MXENE || materialType == CX_ALPHA){
      String sub14[2];
      split(fields[14], '_', sub14, 2);
      totalColumns     = (uint16_t) sub14[0].toInt();
      totalRows        = (uint16_t) sub14[1].toInt();
    }
    
    return true;
    
  } else if (m == MUX_FGR_BEND && n == 13) { // Variation 2
    machineType      = m;

    String cy = fields[4];
    cy.trim();
    
    unsigned long tc = cy.toInt();
    if (tc < 1UL || tc > 9999999UL) return false;

    totalCycles = (uint32_t)tc;

    // Motor Speed sub‐fields
    String sub5[4];
    split(fields[5], '_', sub5, 4);
    
    char sm = sub5[0].length() ? sub5[0].charAt(0) : MUX32_OPT_NA;
    
    if (sm == 'R') {
      varSpeedAvailable = true;
      
      nominalRPM = sub5[2].toFloat();
    } else if (sm == 'S') {
      varSpeedAvailable = true;
      
      speedStart       = sub5[1].toFloat();
      speedEnd         = sub5[2].toFloat();
      speedStep        = sub5[3].toFloat();
    } else
      varSpeedAvailable = false;

    // Angle sub‐fields
    String sub6[4];
    split(fields[6], '_', sub6, 4);
    
    char va = sub6[0].length() ? sub6[0].charAt(0) : MUX32_OPT_NA;
    
    if (va == 'A') {
      varAngleAvailable = true;
      
      motorAngle = sub6[2].toFloat();
    } else if (va == 'V') {
      varAngleAvailable = true;
      
      angleStart       = sub6[1].toFloat();
      angleEnd         = sub6[2].toFloat();
      angleStep        = sub6[3].toFloat();
    } else
      varAngleAvailable = false;

    // H_/N_ sub‐fields
    String sub7[3];
    split(fields[7], '_', sub7, 3);
    char h7 = sub7[0].length() ? sub7[0].charAt(0) : MUX32_OPT_NA;
    
    if (h7 == MUX32_OPT_NA)
      extLoadAvailable = false;
    else {
      extLoadAvailable = true;
      
      extLoadCapacity  = (uint32_t) sub7[1].toInt();
      extLoadUnits     = (uint8_t) sub7[3].toInt(); //1 = g, 2 = N, 3 = kg
    }
    
    materialType     = fields[8].charAt(0);
    testType         = fields[9].charAt(0);
        
    String sub10[3];
    split(fields[10], '_', sub10, 3);
    matLength        = (uint16_t) sub10[0].toInt();
    matWidth         = (uint16_t) sub10[1].toInt();
    matHeight        = (uint16_t) sub10[2].toInt();

    if (fields[11].charAt(0) == MUX32_OPT_NA)
      hasDelam       = false;
    else
      hasDelam       = true;
    
    sampleNumber     = (uint16_t) fields[12].toInt();
    totalChannels    = (uint16_t) fields[13].toInt();

    if(materialType == MWCNT || materialType == MXENE || materialType == CX_ALPHA){
      String sub14[2];
      split(fields[14], '_', sub14, 2);
      totalColumns     = (uint16_t) sub14[0].toInt();
      totalRows        = (uint16_t) sub14[1].toInt();
    }
    
    return true;
    
  } else if (m == MUX_OAX_STR && n == 11) { // Variation 3
    machineType      = m;
    totalCycles      = (uint32_t) fields[1].toInt();
    
    String sub3[2];
    split(fields[2], '_', sub3, 2);
    maxStrain        = (uint32_t) sub3[0].toInt();
    strainUnits      = (uint8_t) sub3[1].toInt();
    
    String sub4[3];
    split(fields[3], '_', sub4, 3);
    if (sub4[0]!="H" && sub4[0]!="N") return false;
    extLoadAvailable = sub4[0].charAt(0);
    extLoadCapacity  = (uint32_t) sub4[1].toInt();
    extLoadUnits     = (uint8_t) sub4[2].toInt();
    materialType     = fields[4].charAt(0);
    testType         = fields[5].charAt(0);
    
    String sub7[3];
    split(fields[6], '_', sub7, 3);
    matLength        = (uint16_t) sub7[0].toInt();
    matWidth         = (uint16_t) sub7[1].toInt();
    matHeight        = (uint16_t) sub7[2].toInt();
    
    if (fields[7].charAt(0) == MUX32_OPT_NA)
      hasDelam       = false;
    else
      hasDelam       = true;
    
    sampleNumber     = (uint16_t) fields[8].toInt();
    
    String sub9[2];
    split(fields[9], '_', sub9, 2);
    totalColumns     = (uint16_t) sub9[0].toInt();
    totalRows        = (uint16_t) sub9[1].toInt();
    
    totalChannels    = (uint16_t) fields[10].toInt();
    
    return true;
  }
  
  return false;
}

/*** Default Functions Headers ***/
//void scanMUX(uint8_t start = 1, uint8_t end = 21, uint16_t delay_ms = 10);

void setup() {
  pinMode(EN_SNS, OUTPUT);
  digitalWrite(EN_SNS, LOW); // Apaga VCNT
  
  pinMode(EN_REF, OUTPUT);
  digitalWrite(EN_REF, LOW); // Apaga VREF
  
  pinMode(EN_PER, OUTPUT);
  digitalWrite(EN_PER, HIGH); // Enciende VEN(3v3_2)

  // Deselect All SPI devices
  pinMode(AD_CS, OUTPUT);
  digitalWrite(AD_CS, HIGH);
  
  pinMode(CS_3, OUTPUT);
  digitalWrite(CS_3, HIGH);
  
  pinMode(DP_CS, OUTPUT);
  digitalWrite(DP_CS, HIGH);
  
  pinMode(MC_CS, OUTPUT);
  digitalWrite(MC_CS, HIGH);
  
  //pinMode(SD_CS, OUTPUT);
  //digitalWrite(SD_CS, HIGH);

  delay(300);

  pinMode(HX_DRT, OUTPUT);
  digitalWrite(HX_DRT, set80SPS ? HIGH : LOW);
  
  pinMode(LED_GREEN, OUTPUT);
  analogWrite(LED_GREEN, GNLED_BRIGHTNESS);
  
  pinMode(LED_ORANGE, OUTPUT);
  analogWrite(LED_ORANGE, OGLED_BRIGHTNESS);
  
  // RAK WEIRD RUTINE TO AVOID CRASHING THE AT COMMANDS MCUs
  time_t timeout = millis();
  
  Serial.begin(115200);
  
  while(!Serial){
    if((millis() - timeout) < 5000)
      delay(100);
    else
      break;
  }

  // Try init microSD and restore RTC
  restoreRTC();

  // MCP23S17 Init
  if (!mcp.begin_SPI(MC_CS)) {
    Serial.println(MCP23_ERROR_INIT);
  } else {
    for(int i = 0; i < 16; i++)
      mcp.pinMode(i, OUTPUT);
  }
  
  // bq25155 Init
  if (!charger.begin(BQ_CHEN, BQ_INT, BQ_LPM))
    Serial.println(bq25155_ERROR_INIT);
  else{
    charger.DisableRST14sWD();
    charger.setPGasGPOD();
    charger.DisablePG();
  }
  
  // AD5242#1 Init
  if(!POT1.begin(1000000))
    Serial.println(POT1_ERROR_INIT);

  // AD5242#2 Init
  if(!POT2.begin(1000000))
    Serial.println(POT2_ERROR_INIT);
  
  // BME680 Init
  if(!bme.begin(BME_ADDR)){
    Serial.println(BME_WARNING);
    return;
  }else{
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }
  
  // LTR390 Init
  if(!ltr.init()){
    Serial.println(LTR_WARNING);
  }
  
  // HX711 Init
  LoadCell.begin();
  
  LoadCell.setReverseOutput(); // Comment if arrow Cell points down
  LoadCell.start(HX_stabilize_t, HX_iniTare);
  
  if (LoadCell.getTareTimeoutFlag())
    Serial.println(HX711_ERROR_INIT);
  else {
    LoadCell.setCalFactor(calibrationValue);

    while(!LoadCell.update());

    convTime = LoadCell.getConversionTime();
    lastSPS = LoadCell.getSPS();
    settTime = LoadCell.getSettlingTime();

    if(lastSPS < 7 || lastSPS > 100)
      Serial.println(HX711_ERROR_INIT);
  }

  /* // ADS1220 Init
  if(!ads.init())
    Serial.println(ADS1220_ERROR_INIT);
  else {
    // The voltages to be measured need to be between negative VREF + 0.2 V and positive
    // VREF -0.2 V if PGA is enabled. For this example I disable PGA, to be on the safe side.
     
    // ads.bypassPGA(true);
    ads.setAvddAvssAsVrefAndCalibrate();
    ads.setCompareChannels(ADS1220_MUX_1_2);
  }*/
  
  analogReadResolution(ADC_bits);
}

void loop() {

  // Update sensor variables
  unsigned long nowsense = millis();
  if (nowsense - lastDataTime >= sensInterval) {
    is_CHG_Set = charger.isChargeEnabled();
    is_CHG_DONE = charger.is_CHARGE_DONE();
    is_Vin_PG = charger.is_VIN_PGOOD();
    is_PG_En = charger.isPGEnabled();

    INVo = charger.readVIN(1);
    INCu = charger.readIIN(2);
    VBAT = charger.readVBAT(1);
    ICHG = charger.readICHG(2);

    readBME(TempEn, RHumEn, PresEn, GasEn);

    if (LUXEn)
      readLTR(LUXType == 2 ? true : false);

    readHX7();
    
    lastSensTime = nowsense;
  }

  if (Serial.available()) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.length() > 0) {
      if (in.length() == 1 && in.charAt(0) == MUX32_ALIVE) {
        // Ping request
        Serial.println(MUX32_SUCCESS);

      } else if (sscanf(in.c_str(), "%d_%d_%d_%d_%d_%d", &Y, &Mo, &D, &H, &Mi, &S) == 6) {
        setTime(H, Mi, S, D, Mo, Y);

        Serial.print("RTC set to: ");
        Serial.println(fmtTS(now()));

        // Save to SD
        Serial.println(uSD_DTT_update());

      } else if (in.equalsIgnoreCase(MUX32_DT_STAT)) {
        // DTT request
        Serial.println(fmtTS(now()));

      } else if (in.equalsIgnoreCase(MUX32_CH_EN)) {
        // Charge enable
        if (charger.initCHG(
          BAT_VOLTAGE_mV,   // Target charge voltage in mV
          true,             // Enable fast charging?
          CHG_CURRENT_uA,   // Charge current in uA
          PCHG_CURRENT_uA,  // Precharge current in uA
          INPT_CURRENT_mA,  // Input current limit in mA
          SAFETY_TIMER      // Safety timer in tenths of an hour (e.g., 15 = 1.5h, 30 = 3h)
        )) {
          charger.DisableRST14sWD();
          charger.DisablePG();

          Serial.println(MUX32_SUCCESS);
        } else {
          charger.DisableCharge();
          charger.EnablePG();
          Serial.println(bq25155_ERROR_INIT);
        }

      } else if (in.equalsIgnoreCase(MUX32_CH_DIS)) {
        // Charge disabled
        if (charger.DisableCharge()) {
          charger.DisablePG();
          Serial.println(MUX32_SUCCESS);
        } else {
          Serial.println(bq25155_ERROR_INIT);
        }

      } else if (in.equalsIgnoreCase(MUX32_CH_VBAT)) {
        // Vbat only
        VBAT = charger.readVBAT(1);
        Serial.println(VBAT);
        
      } else if (in.equalsIgnoreCase(MUX32_CH_STAT)) {
        // Charger status
        Serial.print("ChSet:"+String(is_CHG_Set)+",");
        Serial.print("ChDn:"+String(is_CHG_DONE)+",");
        Serial.print("VinOK:"+String(is_Vin_PG)+",");
        Serial.print("PGEn:"+String(is_PG_En)+",");
        Serial.print("INVo:"+String(INVo)+",");
        Serial.print("INuA:"+String(INCu)+",");
        Serial.print("BatV:"+String(VBAT)+",");
        Serial.println("CHuA:"+String(ICHG));
        
      } else switch (currentMode) {
        case MODE_IDLE:
          if (in.length() == 1 && in.charAt(0) == MUX32_SETUP) {
            Serial.println(MUX32_SUCCESS);
            currentMode = MODE_CONFIG;
            lastDataTime = millis();
          } else
            Serial.println(MUX32_INVALID_COMMAND);
          break;

        case MODE_CONFIG:
          if (in.length() == 3 && in.equalsIgnoreCase(MUX32_END_OF_TEST)) {
            // End test command
            currentMode = MODE_IDLE;
            Serial.println(MUX32_SUCCESS);
          } else {
            if (parseConfig(in)) {
              if (totalChannels == 1) {
                channelHeader = F("Resistance (6001)");
              }else if (totalChannels == 8) {
                channelHeader = F("1001 <R1> (OHM), 1002 <R2> (OHM), 1003 <R3> (OHM), 1004 <R4> (OHM), "
                                  "1006 <C1> (OHM), 1007 <C2> (OHM), 1008 <C3> (OHM), 1009 <C4> (OHM)");
              }else if (totalChannels == 10) {
                channelHeader = F("1001 <R1> (OHM), 1002 <R2> (OHM), 1003 <R3> (OHM), "
                                  "1004 <R4> (OHM), 1005 <R5> (OHM), "
                                  "1006 <C1> (OHM), 1007 <C2> (OHM), 1008 <C3> (OHM), "
                                  "1009 <C4> (OHM), 1010 <C5> (OHM)");
              }else if (totalChannels == 21) {
                channelHeader = F("1-1p (6001),1-3p (6002),2-4p (6003),3-1p (6004),3-5p (6005),"
                                  "4-2p (6006),4-6p (6007),5-3p (6008),5-7p (6009),6-4p (6010),"
                                  "6-8p (6011),7-5p (6012),7-9p (6013),8-6p (6014),8-10p (6015),"
                                  "9-7p (6016),9-11p (6017),10-8p (6018),10-12p (6019),"
                                  "11-9p (6020),11-13p (6021),12-10p (6022),12-14p (6023),"
                                  "13-11p (6024),13-15p (6025),14-12p (6026),14-16p (6027),"
                                  "15-13p (6028),15-17p (6029),16-14p (6030),16-18p (6031),"
                                  "17-15p (6032),17-19p (6033),18-16p (6034),18-20p (6035),"
                                  "19-17p (6036),19-21p (6037),20-18p (6038),21-19p (6039),21-21p (6040)");
              }//else
              //  Serial.println(MUX32_INVALID_PAYLOAD);
              Serial.print(F("5001 <LOAD> (VDC),5021 <DISP> (VDC),"));
              Serial.println(channelHeader);
            
              currentMode = MODE_DATAREQ;
              lastDataTime = millis();
            } else
              Serial.println(MUX32_INVALID_PAYLOAD);
          }
          break;
        case MODE_DATAREQ:
          if (tolower(in.charAt(0)) == MUX32_PAUSE_TEST) {
            // pause/resume test
            paused = !paused;
            Serial.println(MUX32_SUCCESS);
          } else if (tolower(in.charAt(0)) == MUX32_MANUAL_REQ) {
            // manual request
            sendData();
          } else if (tolower(in.charAt(0)) == MUX32_TARE_HX) {
            // tare comand
            if (extLoadAvailable == 'H') {
              
              LoadCell.tareNoDelay();
              if (taredLoadcell())
                Serial.println(MUX32_SUCCESS);
              else
                Serial.println(HX711_ERROR_INIT);
            } else {
              Serial.println(MUX32_INVALID_COMMAND);
            }
          } else if (in.equalsIgnoreCase(MUX32_END_OF_TEST)) {
            // End test command
            currentMode = MODE_IDLE;
            Serial.println(MUX32_SUCCESS);
          }
          break;
      }
    }
  }

  // Data burst (if auto send config)
  if (currentMode == MODE_DATAREQ && !paused) {
    unsigned long nowdata = millis();
    if (nowdata - lastDataTime >= dataInterval) {
      // sendData(); // disabled for now
      lastDataTime = nowdata;
    }
  }

  // Autosave Date&Time
  if (sdDueToSave())
    int dttsaved = uSD_DTT_update();
}

//----------------------------------------------------------------------------------------------------------------------
// Channels Management
//----------------------------------------------------------------------------------------------------------------------

void initMUXes(){
  // Set WR & CS HIGH to initialize MUXes
  portA = GPA_CS | GPA_WR;
  portB = GPB_CS | GPB_WR;
  
  gpioAB = ((uint16_t)portB << 8) | portA;
  
  mcp.writeGPIOAB(gpioAB);
  delayMicroseconds(MCP_delay);
}

void selMUXes(){
  // Clear Both CS & WR to enable Ch selection
  portA &= ~(GPA_CS | GPA_WR);
  portB &= ~(GPB_CS | GPB_WR);
  
  gpioAB = ((uint16_t)portB << 8) | portA;
  
  mcp.writeGPIOAB(gpioAB);
  delayMicroseconds(MCP_delay);
}

void setMUXChannels(uint8_t chMux1, uint8_t chMux2){
  // Clamp Channels to 1–32 range
  if (chMux1 > 32) chMux1 = 32;
  if (chMux1 < 1) chMux1 = 1;
  
  if (chMux2 > 32) chMux2 = 32;
  if (chMux2 < 1) chMux2 = 1;
  
  // Convert to 0-based indexes for ADG732
  chMux1 = chMux1 - 1;
  chMux2 = chMux2 - 1;

  // Extract A0–A4 address bits
  M1A0 = chMux1 & 0x01;
  M1A1 = chMux1 & 0x02;
  M1A2 = chMux1 & 0x04;
  M1A3 = chMux1 & 0x08;
  M1A4 = chMux1 & 0x10;
  
  M2A0 = chMux2 & 0x01;
  M2A1 = chMux2 & 0x02;
  M2A2 = chMux2 & 0x04;
  M2A3 = chMux2 & 0x08;
  M2A4 = chMux2 & 0x10;
  
  // Clear address bits before writing
  portA &= ~(GPA_A0 | GPA_A1 | GPA_A2 | GPA_A3 | GPA_A4);
  portB &= ~(GPB_A0 | GPB_A1 | GPB_A2 | GPB_A3 | GPB_A4);

  // Write new bits
  portA |= (M2A0 ? GPA_A0 : 0) | (M2A1 ? GPA_A1 : 0) | (M2A2 ? GPA_A2 : 0) | 
           (M2A3 ? GPA_A3 : 0) | (M2A4 ? GPA_A4 : 0);
  portB |= (M1A0 ? GPB_A0 : 0) | (M1A1 ? GPB_A1 : 0) | (M1A2 ? GPB_A2 : 0) | 
           (M1A3 ? GPB_A3 : 0) | (M1A4 ? GPB_A4 : 0);
           
  gpioAB = ((uint16_t)portB << 8) | portA;

  mcp.writeGPIOAB(gpioAB);
  delayMicroseconds(MCP_delay);
}

void latchMUXes(){
  // Set CS & WR HIGH, everything else LOW (latch)
  portA = GPA_CS | GPA_WR;
  portB = GPB_CS | GPB_WR;
  gpioAB = ((uint16_t)portB << 8) | portA;

  mcp.writeGPIOAB(gpioAB);
  delayMicroseconds(MCP_delay);
}

void allOff() {
  // Write ADG732_ALLOFF (EN pin HIGH, everything else = 0)
  portA = GPA_EN;
  portB = GPB_EN;
  
  gpioAB = ((uint16_t)portB << 8) | portA;
  mcp.writeGPIOAB(gpioAB);
  delayMicroseconds(MCP_delay);
}

//----------------------------------------------------------------------------------------------------------------------
// Bridge Parameters Management
//----------------------------------------------------------------------------------------------------------------------

// Converts pot value (0–255) to actual resistance in ohms
float potToOhms(uint8_t potVal) {
  return (potVal / MAX_POT_STEPS) * MAX_RESISTANCE_OHMS;
}

// Calculates RU (upper right) in ohms
float UpperWheatstoneBridge(uint8_t R1, uint8_t R2, uint8_t RL, int32_t VWB, uint32_t V_REF) {
  if (R1 == 0 || R2 == 0 || RL == 0 || V_REF == 0) return -1;

  float R1f = potToOhms(R1);
  float R2f = potToOhms(R2);
  float RLf = potToOhms(RL);

  float left_ratio = R1f / (R1f + R2f);
  
  float VWB_ratio = (float)VWB / V_REF;
  
  float right_ratio = VWB_ratio + left_ratio;

  if (right_ratio <= 0.0 || right_ratio >= 1.0) return -1;

  float RU = (RLf * right_ratio) / (1.0 - right_ratio);
  return RU;  // In ohms
}

// Calculates RL (lower right) in ohms
float LowerWheatstoneBridge(uint8_t R1, uint8_t R2, uint8_t RU, int32_t VWB, uint32_t V_REF) {
  if (R1 == 0 || R2 == 0 || RU == 0 || V_REF == 0) return -1;

  float R1f = potToOhms(R1);
  float R2f = potToOhms(R2);
  float RUf = potToOhms(RU);

  float left_ratio = R1f / (R1f + R2f);
  
  float VWB_ratio = (float)VWB / V_REF;
  
  float right_ratio = VWB_ratio + left_ratio;

  if (right_ratio <= 0.0 || right_ratio >= 1.0) return -1;

  float RL = (RUf * (1.0 - right_ratio)) / right_ratio;
  return RL;  // In ohms
}

void setDigitPots(uint8_t DP1R1, uint8_t DP1R2, uint8_t DP2R3){
    POT1.write(0, DP1R1);
    POT1.write(1, DP1R2);
    
    POT2.write(0, DP2R3);
    POT2.write(1, DP2R3);
}

// read ADC
void readADC(uint32_t deluSecs){
// Find a way to set all resistances voltage values to zero bridge (only first init)
// an array to store each R4 of wheatstone bridge?
  uint8_t PR1 = 255;
  uint8_t PR2 = 255;
  uint8_t PR3 = 255;

  setDigitPots(PR1, PR2, PR3);
  
  initMUXes();
  
  for (uint8_t i = 0; i < ADS_READINGS; i++) {
    setMUXChannels(GFW_chs[i].ch1, GFW_chs[i].ch2);

    if (!i)
      selMUXes();
    
    delayMicroseconds(deluSecs);

    // read ADS1220
    //float ADS_mV = analogRead(SHMDZ_LOAD);
    //int32_t ADS_uV = (int32_t)roundf(ADS_mV * 1000.0f);
    int32_t ADS_uV = analogRead(SHMDZ_LOAD);
    
    readings_uV[i] = ADS_uV;
  }
  
  allOff();
}

//----------------------------------------------------------------------------------------------------------------------
// Sensors Data Management
//----------------------------------------------------------------------------------------------------------------------

// Ambient sensor read
static bool readBME(bool temp, bool rhum, bool atmpr, bool gas) {
  if (bme.performReading()) {
    
    BME_temp = (temp ? bme.temperature : 0.0); // in C
    BME_humr = (rhum ? bme.humidity : 0.0); // in rH%
    BME_press = (atmpr ? bme.pressure : 0.0); // in Pa, divide by 100 to hPa
    BME_gas = (gas ? bme.gas_resistance : 0.0); // in Ohms, divide by 1000 to kOhms
    
    return true;
  } else
    return false;
}

// Lux sensor read
static bool readLTR(bool UVI) {
  
  // if set LTR390_MODE_UVS, get UV light data.
  // if set LTR390_MODE_ALS, get ambient light data
  if (UVI)
    ltr.setMode(LTR390_MODE_UVS);
  else
    ltr.setMode(LTR390_MODE_ALS);
  
  // perform data reading:
  if (ltr.newDataAvailable()) {
    if(ltr.getMode() == LTR390_MODE_ALS) {
      LTR_XXS = ltr.readALS();
      LTR_IDX = ltr.getLUX();
    } else {
      LTR_XXS = ltr.readUVS();
      LTR_IDX = ltr.getUVI();
    }

    return true;
  } else
    return false;
}

// Loadcell ADC read
static bool readHX7() {
  // check for new data/start next conversion:
  if (LoadCell.update()) {
    HX_weight = LoadCell.getData();
    HX_load = HX_weight * 0.00980665;
    
    return true;
  } else
    return false;
}

// Loadcell tare check
static bool taredLoadcell() {
  if(!LoadCell.getTareStatus())
    return true;
  else
    return false;
}

static void restoreRTC() {
  if (SD.begin(SD_CS)) {
    // Check for file first
    if (SD.exists(setts_file)) {
      // Load from SD and set RTC
      uint32_t e = 0;
      if (sdLoadEpoch(e) && e > 1000000000UL) {
        setTime(e);
        lastSavedYMD = packYMD(year(), month(), day());
      } else
        defaultRTC();
    } else {
      // Use the initial defaults
      tmElements_t tm;
      tm.Year   = CalendarYrToTm(INIT_Y);
      tm.Month  = INIT_MO;
      tm.Day    = INIT_D;
      tm.Hour   = INIT_H;
      tm.Minute = INIT_MI;
      tm.Second = INIT_S;

      time_t t0 = makeTime(tm);

      defaultRTC();

      // Try to create the file
      if (!sdSaveEpoch(t0))
        Serial.println(DTT_WARNING);
    }

    SD.end();

  } else
    defaultRTC();
}

static void defaultRTC() {
  // Just set the RTC from user defaults
  setTime(INIT_H, INIT_MI, INIT_S, INIT_D, INIT_MO, INIT_Y);
  lastSavedYMD = packYMD(INIT_Y, INIT_MO, INIT_D);
}

static uint8_t uSD_DTT_update() {
  if (SD.begin(SD_CS)) {
    if (sdSaveEpoch(now())) {
      SD.end();

      return MUX32_SUCCESS;
    } else {
      SD.end();

      return DTT_WARNING;
    }
  } else
    return SDCARD_WARNING;
}

static bool sdLoadEpoch(uint32_t &outEpoch) {
  DTTSettings = SD.open(setts_file, FILE_READ);
  if (!DTTSettings) return false;

  // Read small file into buffer
  char buf[96] = {0};
  
  size_t n = DTTSettings.readBytes(buf, sizeof(buf) - 1);
  DTTSettings.close();
  
  if (n == 0) return false;

  // UNIX= first
  uint32_t e = 0;
  if (sscanf(buf, "EPOCH=%lu", &e) == 1 && e > 1000000000UL) {
    outEpoch = e;
    return true;
  }

  // HUMAN=YYYY_MM_DD_HH_MM_SS
  int Y,Mo,D,H,Mi,S;
  if (strstr(buf, "HUMAN=") && sscanf(buf, "HUMAN=%d_%d_%d_%d_%d_%d", &Y,&Mo,&D,&H,&Mi,&S) == 6) {
    tmElements_t tm;
    tm.Year = CalendarYrToTm(Y);
    tm.Month = Mo;
    tm.Day = D;
    tm.Hour = H;
    tm.Minute = Mi;
    tm.Second = S;
    outEpoch = makeTime(tm);
    return true;
  }
  
  return false;
}

static bool sdSaveEpoch(time_t t) {
  // Open for write (will overwrite existing file)
  DTTSettings = SD.open(setts_file, FILE_WRITE);
  if (!DTTSettings) return false;

  // Since there is not support truncate(), we must remove & recreate
  DTTSettings.close();
  SD.remove(setts_file);
  
  DTTSettings = SD.open(setts_file, FILE_WRITE);
  if (!DTTSettings) return false;

  // Write both machine- and human-friendly versions
  DTTSettings.print("EPOCH=");
  DTTSettings.println((uint32_t)t);
  DTTSettings.print("HUMAN=");
  DTTSettings.println(fmtTS(t));

  DTTSettings.flush();
  DTTSettings.close();
  
  lastSavedYMD = packYMD(Y,Mo,D);

  g_lastSdSaveMs = millis();
  return true;  // always true if we reached here
}

// Save guard for periodic SD writes
static inline bool sdDueToSave() {
  const uint32_t periodMs = (uint32_t)save_dtt_sd * 60000UL;
  return (millis() - g_lastSdSaveMs) >= periodMs;
}

// pack Y/M/D into a single int for easy comparison
static uint16_t packYMD(int y, int m, int d) {
  return uint16_t(y % 100) * 10000 + uint16_t(m) * 100 + uint16_t(d);
}

// format epoch as "YYYY_MM_DD_HH_MM_SS" 
static String fmtTS(time_t t) {
  char b[20];
  snprintf(b, sizeof(b), "%04d_%02d_%02d_%02d_%02d_%02d", year(t), month(t), day(t), hour(t), minute(t), second(t));
  return String(b);
}

bool save2SD(){
  uint8_t fnum = 1;
  
  String extn = ".txt";
  String fname = formNum(fnum)+extn;
  
  if (isSDins) {
    if (is1stIt) {
      while (SD.exists(fname)) {
        fnum++;
        fname = formNum(fnum)+extn;
      }
      
      is1stIt = false;
      uSD_row = 1;
    }

    //------- Write SD file --------
    DataSD = SD.open(fname, FILE_WRITE);
    if(uSD_row==1){
      DataSD.print("Scan,Time,5001 <LOAD> (VDC),5021 <DISP> (VDC),");
      DataSD.print("6001 (OHM),6002 (OHM),6003 (OHM),6004 (OHM),6005 (OHM),6006 (OHM),6007 (OHM),6008 (OHM),6009 (OHM),6010 (OHM),");
      DataSD.print("6011 (OHM),6012 (OHM),6013 (OHM),6014 (OHM),6015 (OHM),6016 (OHM),6017 (OHM),6018 (OHM),6019 (OHM),6020 (OHM),");
      DataSD.print("6021 (OHM),6022 (OHM),6023 (OHM),6024 (OHM),6025 (OHM),6026 (OHM),6027 (OHM),6028 (OHM),6029 (OHM),6030 (OHM),");
      DataSD.println("6031 (OHM),6032 (OHM),6033 (OHM),6034 (OHM),6035 (OHM),6036 (OHM),6037 (OHM),6038 (OHM),6039 (OHM),6040 (OHM)");
    }
    
    DataSD.print(String(uSD_row)+",");
    DataSD.print(String(uSD_row * 100)+",");
    DataSD.print(String(charger.readVBAT(1))+",");
    DataSD.print(String(charger.readVBAT(1))+",");
    
    for (uint8_t i = 0; i < ADS_READINGS; i++) {
      DataSD.print(readings_uV[i]);
      if (i<(ADS_READINGS-1))
        DataSD.print(","); // Add a comma between vals
    }
    
    DataSD.println(); // Add a newline
    DataSD.close();
  }
}

// Double digit format
String formNum(uint8_t nmbr){
  // Format to have leading zeros
  if (nmbr < 10)
    return "0" + String(nmbr);
  else
    return String(nmbr);
}

// Data Payload
void sendData() {
  /*
  readADC(20);
  // make uid universal
  save2SD();
  
  Serial.print(String(uSD_row)+",");
  Serial.print(String(uSD_row * 100)+",");
  Serial.print(String(charger.readVIN(1))+",");
  Serial.print(String(charger.readVBAT(1))+",");
  
  for (uint8_t i = 0; i < ADS_READINGS; i++) {
    Serial.print(readings_uV[i]);
    if (i<(ADS_READINGS-1))
      Serial.print(","); // Add a comma between vals
  }
  */
  
  Serial.print(","+String(HX_weight));
  
  Serial.print(","+String(BME_temp));
  Serial.print(","+String(BME_humr));
  Serial.print(","+String(BME_press/100));
  Serial.print(","+String(BME_gas));
  
  Serial.print(","+String(LTR_XXS));
  Serial.print(","+String(LTR_IDX));
  Serial.print(","+String(charger.readVBAT(1)));
  
  Serial.println();
}
