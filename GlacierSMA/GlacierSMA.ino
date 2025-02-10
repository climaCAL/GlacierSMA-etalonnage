/*
    Title:    Cryologger Automatic Weather Station
    Date:     February 24, 2023
    Author:   Adam Garbo
    Version:  0.4

    Description:
    - Code configured for automatic weather stations to be deployed in Igloolik, Nunavut.

    Components:
    - Rock7 RockBLOCK 9603
    - Maxtena M1621HCT-P-SMA antenna (optional)
    - Adafruit Feather M0 Adalogger
    - Adafruit Ultimate GPS Featherwing
    - Adafruit BME280 Temperature Humidity Pressure Sensor
    - Adafruit LSM303AGR Accelerometer/Magnetomter
    - Pololu 3.3V 600mA Step-Down Voltage Regulator D36V6F3
    - Pololu 5V 600mA Step-Down Voltage Regulator D36V6F5
    - Pololu 12V 600mA Step-Down Voltage Regulator D36V6F5
    - SanDisk Industrial XI 8 GB microSD card

    Sensors:
    - RM Young 05103L Wind Monitor
    - Vaisala HMP60 Humidity and Temperature Probe

    Comments:
    - Sketch uses 98720 bytes (37%) of program storage space. Maximum is 262144 bytes.
    - Power consumption in deep sleep is ~625 uA at 12.5V

*/

// ----------------------------------------------------------------------------
// Libraries
// ----------------------------------------------------------------------------
#include <Adafruit_BME280.h>        // https://github.com/adafruit/Adafruit_BME280 (v2.2.2)
#include <Adafruit_LSM303_Accel.h>  // https://github.com/adafruit/Adafruit_LSM303_Accel (v1.1.4)
#include <Adafruit_Sensor.h>        // https://github.com/adafruit/Adafruit_Sensor (v1.1.4)
#include <Arduino.h>                // Required for new Serial instance. Include before <wiring_private.h>
#include <ArduinoLowPower.h>        // https://github.com/arduino-libraries/ArduinoLowPower (v1.2.2)
#include <IridiumSBD.h>             // https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library (v3.0.6)
#include <RTCZero.h>                // https://github.com/arduino-libraries/RTCZero (v1.6.0)
#include <SdFat.h>                  // https://github.com/greiman/SdFat (v2.1.2)
#include <sensirion.h>              // https://github.com/HydroSense/sensirion
#include <Statistic.h>              // https://github.com/RobTillaart/Statistic (v1.0.0)
#include <TimeLib.h>                // https://github.com/PaulStoffregen/Time (v1.6.1)
#include <TinyGPS++.h>              // https://github.com/mikalhart/TinyGPSPlus (v1.0.3)
#include <Wire.h>                   // https://www.arduino.cc/en/Reference/Wire
#include <wiring_private.h>         // Required for creating new Serial instance
#include "src/Adafruit_VEML7700.h"  // Customized version of Adafruit VEML7700 library

// ----------------------------------------------------------------------------
// Define unique identifier
// ----------------------------------------------------------------------------
#define CRYOLOGGER_ID   "PART-IT-01"

// ----------------------------------------------------------------------------
// Data logging
// ----------------------------------------------------------------------------
#define LOGGING         false  // Log data to microSD

// ----------------------------------------------------------------------------
// Debugging macros
// ----------------------------------------------------------------------------
#define DEBUG           false // Output debug messages to Serial Monitor
#define DEBUG_GNSS      false // Output GNSS debug information
#define DEBUG_IRIDIUM   false // Output Iridium debug messages to Serial Monitor
#define INSOMNIAC       true  // Prevent sleeping
#define CALIBRATE       false // Enable sensor calibration code

#if DEBUG
#define DEBUG_PRINT(...)          SERIAL_PORT.print(__VA_ARGS__)
#define DEBUG_PRINTF(x, ...)      SERIAL_PORT.print(F(x), ##__VA_ARGS__)
#define DEBUG_PRINTLN(...)        SERIAL_PORT.println(__VA_ARGS__)
#define DEBUG_PRINTFLN(x, ...)    SERIAL_PORT.println(F(x), ##__VA_ARGS__)
#define DEBUG_WRITE(x)            SERIAL_PORT.write(x)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTF(x, ...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINTFLN(x, ...)
#define DEBUG_WRITE(x)
#endif

#define DEBUG_PRINT_HEX(x)        DEBUG_PRINT(x, HEX)
#define DEBUG_PRINTLN_HEX(x)      DEBUG_PRINTLN(x, HEX)
#define DEBUG_PRINT_DEC(x, y)     DEBUG_PRINT(x, y)
#define DEBUG_PRINTLN_DEC(x, y)   DEBUG_PRINTLN(x, y)

// ----------------------------------------------------------------------------
// I2C address definitions
// ----------------------------------------------------------------------------
#define BME280_EXT_ADDR     BME280_ADDRESS            // Defined in Adafruit Library = 0x77 - Used for the outside sensor.
#define BME280_INT_ADDR     BME280_ADDRESS_ALTERNATE  // Defined in Adafruit Library = 0x76 - Used for the inside sensor.
#define BRIDGE_SENSOR_SLAVE_ADDR 0x66                 // WindSensor module I2C address declaration
#define VEML_ADDR           0x10 // According to datasheet page 6 (https://www.vishay.com/docs/84286/veml7700.pdf)

// ----------------------------------------------------------------------------
// Pin definitions
// ----------------------------------------------------------------------------
#define PIN_VBAT            A0
#define PIN_WIND_SPEED      A1  // Used by Young 5103L and Davis 7911
#define PIN_WIND_DIR        A2  // Used by Young 5103L and Davis 7911
#define PIN_HUMID           A3  // Used by HMP60 module
#define PIN_TEMP            A4  // Used by HMP60 module
#define PIN_GNSS_EN         A5
#define PIN_MICROSD_CS      4
#define PIN_12V_EN          5   // 12 V step-up/down regulator
#define PIN_5V_EN           6   // 5V step-down regulator
#define PIN_MICROSD_CD      7   // Detects if the microSD card is present (currently unused)
#define PIN_LED_GREEN       8   // Green LED
#define PIN_IRIDIUM_RX      10  // Pin 1 RXD (Yellow)
#define PIN_IRIDIUM_TX      11  // Pin 6 TXD (Orange)
#define PIN_IRIDIUM_SLEEP   12  // Pin 7 OnOff (Grey)
#define PIN_LED_RED         13

// Unused
#define PIN_SOLAR           7   // Used by SP212
#define PIN_SENSOR_PWR      7   // Used by Davis 7911
#define PIN_RFM95_CS        7   // LoRa "B"
#define PIN_RFM95_RST       7   // LoRa "A"
#define PIN_RFM95_INT       7   // LoRa "D"

// ------------------------------------------------------------------------------------------------
// Port configuration
// ------------------------------------------------------------------------------------------------
// Create a new UART instance and assign it to pins 10 (RX) and 11 (TX).
// For more information see: https://www.arduino.cc/en/Tutorial/SamdSercom
Uart Serial2 (&sercom1, PIN_IRIDIUM_RX, PIN_IRIDIUM_TX, SERCOM_RX_PAD_2, UART_TX_PAD_0);

#define SERIAL_PORT   Serial
#define GNSS_PORT     Serial1
#define IRIDIUM_PORT  Serial2

// Attach interrupt handler to SERCOM for new Serial instance
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

// ----------------------------------------------------------------------------
// Object instantiations
// ----------------------------------------------------------------------------
Adafruit_BME280                 bme280Ext;
Adafruit_BME280                 bme280Int;
//Adafruit_VEML7700               veml = Adafruit_VEML7700(); // Initialized when read because it leaks memory if not destroyed after reading.
Adafruit_LSM303_Accel_Unified   lsm303 = Adafruit_LSM303_Accel_Unified(54321); // I2C address: 0x1E
IridiumSBD                      modem(IRIDIUM_PORT, PIN_IRIDIUM_SLEEP);
RTCZero                         rtc;
SdFs                            sd;           // File system object
FsFile                          logFile;      // Log file
TinyGPSPlus                     gnss;
sensirion                       sht(20, 21);  // (data, clock). Pull-up required on data pin

// Custom TinyGPS objects to store fix and validity information
// Note: $GPGGA and $GPRMC sentences produced by GPS receivers (PA6H module)
// $GNGGA and $GNRMC sentences produced by GPS/GLONASS receivers (PA161D module)
TinyGPSCustom gnssFix(gnss, "GPGGA", 6); // Fix quality
TinyGPSCustom gnssValidity(gnss, "GPRMC", 2); // Validity

// ----------------------------------------------------------------------------
// Statistics objects
// ----------------------------------------------------------------------------
typedef statistic::Statistic<float,uint32_t,false> StatisticSMA;
StatisticSMA batteryStats;         // Battery voltage
StatisticSMA pressureIntStats;     // Pressure from internal sensor
StatisticSMA temperatureIntStats;  // Temperature from internal sensor
StatisticSMA humidityIntStats;     // Humidity from internal sensor
StatisticSMA pressureExtStats;     // Pressure from external sensor
StatisticSMA temperatureExtStats;  // Temperature from external sensor
StatisticSMA humidityExtStats;     // Humidity from external sensor
StatisticSMA solarStats;           // Solar radiation
StatisticSMA hauteurNeigeStats;    // Suivi hauteur de neige
StatisticSMA windSpeedStats;       // Wind speed
StatisticSMA uStats;               // Wind east-west wind vector component (u)
StatisticSMA vStats;               // Wind north-south wind vector component (v)

// ----------------------------------------------------------------------------
// User defined configuration variables
// ----------------------------------------------------------------------------
#if DEBUG
unsigned int  sampleInterval    = 1;      // Sampling interval (minutes). Values must be in range [1, 60]
unsigned int  averageInterval   = 5;      // Number of samples to be averaged in each message. Range: [1, 240]
unsigned int  transmitInterval  = 1;      // Number of messages in each Iridium transmission (340-byte limit)
unsigned int  retransmitLimit   = 5;      // Failed data transmission reattempts (340-byte limit)
unsigned int  iridiumTimeout    = 120;    // Timeout for Iridium transmission (seconds)
unsigned int  gnssTimeout       = 60;     // Timeout for GNSS signal acquisition (seconds)
float         batteryCutoff     = 3.0;    // Battery voltage cutoff threshold (V)
byte          loggingMode       = 3;      // Flag for new log file creation. 1: daily, 2: monthly, 3: yearly
unsigned int  systemRstWDTCountLimit = 5; // Nombre d'alertes WDT autorisées avant de faire un system Reset (8s par cycle)
#else
unsigned int  sampleInterval    = 1;      // Sampling interval (minutes). Default: 5 min (300 seconds)
unsigned int  averageInterval   = 12;     // Number of samples to be averaged in each message. Default: 12 (hourly)
unsigned int  transmitInterval  = 1;      // Number of messages in each Iridium transmission (340-byte limit)
unsigned int  retransmitLimit   = 5;      // Failed data transmission reattempts (340-byte limit)
unsigned int  iridiumTimeout    = 240;    // Timeout for Iridium transmission (seconds)
unsigned int  gnssTimeout       = 120;    // Timeout for GNSS signal acquisition (seconds)
float         batteryCutoff     = 11.0;   // Battery voltage cutoff threshold (V)
byte          loggingMode       = 2;      // Flag for new log file creation. 1: daily, 2: monthly, 3: yearly
unsigned int  systemRstWDTCountLimit = 15;// Nombre d'alertes WDT autorisées avant de faire un system Reset (8s par cycle)
#endif
//TODO Verify that these values are within range -- see iridium.ino for acceptable ranges.

// ----------------------------------------------------------------------------
// Sensors correction factor and offsets -- to modify -- 
// ----------------------------------------------------------------------------
//BME280 -- Exterior sensor
float tempBmeEXT_CF             = 1.0;      // Correction factor for exterior temperature acquisition.
float tempBmeEXT_Offset         = 0.0;      // Offset for exterior temperature acquisition.
float humBmeEXT_CF              = 1.0;      // Correction factor for exterior humidity acquisition.
float humBmeEXT_Offset          = 0.0;      // Offset for exterior humidity acquisition.
float presBmeEXT_CF             = 1.0;      // Correction factor for exterior pressure acquisition.
float presBmeEXT_Offset         = 0.0;      // Offset for exterior pressure acquisition.

//BME280 -- Interior sensor
float tempBmeINT_CF             = 1.0;      // Correction factor for interior temperature acquisition.
float tempBmeINT_Offset         = 0.0;      // Offset for interior temperature acquisition.
float humBmeINT_CF              = 1.0;      // Correction factor for interior humidity acquisition.
float humBmeINT_Offset          = 0.0;      // Offset for interior humidity acquisition.
float presBmeINT_CF             = 1.0;      // Correction factor for interior pressure acquisition.
float presBmeINT_Offset         = 0.0;      // Offset for interior pressure acquisition.

//VEML7700
float veml_CF                   = 22.045;   // Correction factor for light intensity acquisition. Ref: ÉtalonnageVEML7700_H24.xlsx
float veml_Offset               = -372.06;  // Offset for light intensity acquisition.

// ----------------------------------------------------------------------------
// Error codes and values
// ----------------------------------------------------------------------------

// Cas d'erreurs des valeurs du Stevenson:
const int16_t temp_ERRORVAL  = -25500;   //temperature
const int16_t hum_ERRORVAL   = -25500;   //humidite
const int16_t pres_ERRORVAL  = -2550;    //pression atmospherique
const uint16_t lux_ERROVAL   = 0;        //Luminosité
const uint16_t HN_ERRORVAL   = 0xFFFF;   //Hauteur de neige

// ----------------------------------------------------------------------------
// Global variable declarations
// ----------------------------------------------------------------------------
volatile bool alarmFlag         = false;  // Flag for alarm interrupt service routine
volatile bool wdtFlag           = false;  // Flag for Watchdog Timer interrupt service routine
volatile int  wdtCounter        = 0;      // Watchdog Timer interrupt counter
volatile int  revolutions       = 0;      // Wind speed ISR counter
bool          firstTimeFlag     = true;   // Flag to determine if program is running for the first time
bool          resetFlag         = false;  // Flag to force system reset using Watchdog Timer
uint8_t       moSbdBuffer[340];           // Buffer for Mobile Originated SBD (MO-SBD) message (340 bytes max)
uint8_t       mtSbdBuffer[270];           // Buffer for Mobile Terminated SBD (MT-SBD) message (270 bytes max)
size_t        moSbdBufferSize;
size_t        mtSbdBufferSize;
char          logFileName[50]   = "";     // Log file name
char          dateTime[20]      = "";     // Datetime buffer
byte          retransmitCounter = 0;      // Counter for Iridium 9603 transmission reattempts
byte          transmitCounter   = 0;      // Counter for Iridium 9603 transmission intervals
byte          currentLogFile    = 0;      // Variable for tracking when new microSD log files are created
byte          currentDay        = 0;      // Variable for tracking when the date changes
int           transmitStatus    = 0;      // Iridium transmission status code
unsigned int  iterationCounter  = 0;      // Counter for program iterations (zero indicates a reset)
unsigned int  failureCounter    = 0;      // Counter of consecutive failed Iridium transmission attempts
unsigned long previousMillis    = 0;      // Global millis() timer
unsigned long alarmTime         = 0;      // Global epoch alarm time variable
unsigned long unixtime          = 0;      // Global epoch time variable
unsigned int  cutoffCounter     = 0;      // Battery voltage cutoff sleep cycle counter
unsigned long sampleCounter     = 0;      // Sensor measurement counter
unsigned long samplesSaved      = 0;      // Log file sample counter
long          rtcDrift          = 0;      // RTC drift calculated during sync

float         pressureInt       = 0.0;    // Internal pressure (hPa)
float         temperatureInt    = 0.0;    // Internal temperature (°C)
float         humidityInt       = 0.0;    // Internal hunidity (%)
float         pressureExt       = 0.0;    // External pressure (hPa)
float         temperatureExt    = 0.0;    // External temperature (°C)
float         humidityExt       = 0.0;    // External humidity (%)
float         pitch             = 0.0;    // Pitch (°)
float         roll              = 0.0;    // Roll (°)
float         solar             = 0.0;    // Solar radiation (lx)
float         hauteurNeige      = 0.0;    // Mesure de la hauteur de neige, en mm
float         temperatureHN     = 0.0;    // Temperature au moment de la mesure de la hauteur de neige (en C, 1C pres)
float         windSpeed         = 0.0;    // Wind speed (m/s)
float         windDirection     = 0.0;    // Wind direction (°)
float         windGustSpeed     = 0.0;    // Wind gust speed  (m/s)
float         windGustDirection = 0.0;    // Wind gust direction (°)
int           windDirectionSector = 0.0;  // Wind direction indicator (ref to DFRWindSpeed() for details)
float         voltage           = 0.0;    // Battery voltage (V)
float         latitude          = 0.0;    // GNSS latitude (DD)
float         longitude         = 0.0;    // GNSS longitude (DD)
byte          satellites        = 0;      // GNSS satellites
float         hdop              = 0.0;    // GNSS HDOP
tmElements_t  tm;                         // Variable for converting time elements to time_t

// ----------------------------------------------------------------------------
// Unions/structures
// ----------------------------------------------------------------------------

// DFRWindSensor (CAL) struc to store/retreive data
const int regMemoryMapSize = 10;
union sensorsDataRaw {
  struct {
    uint16_t angleVentReg = 0; // direction vent en degrés (0-360)
    uint16_t dirVentReg   = 0; // direction vent en secteur (0-15)
    uint16_t vitVentReg   = 0; // vitesse vent en m/s *10
    uint16_t HNeigeReg    = 0; // hauteur de neige en mm
    uint16_t tempHNReg    = 0; // temperature de reference pour la mesure hauteur de neige, en Celcius, resolution de 1C
    uint16_t tempExtReg   = 0; // temperature du BME280 dans le Stevenson
    uint16_t humExtReg    = 0; // humidite du BME280 dans le Stevenson
    uint16_t presExtReg   = 0; // pression du BME280 dans le Stevenson
    uint16_t luminoReg    = 0; // Luminosite du VEML7700 dans le Stevenson
    uint16_t stvsnErrReg  = 0;
  } __attribute__((packed));
  uint16_t regMemoryMap[regMemoryMapSize];
};
static_assert(sizeof(sensorsDataRaw) == regMemoryMapSize * sizeof(uint16_t));

struct sensorsData {
  float angleVentFloat = 0.0;
  uint16_t directionVentInt = 0;
  float vitesseVentFloat = 0.0;
  float hauteurNeige = 0.0;              //V0.6
  float temperatureHN = 0.0;             //V0.6
  float temperatureExt = 0.0;            //V0.7
  float humiditeExt = 0.0;               //V0.7
  float presAtmospExt = 0.0;             //V0.7
  float luminoAmbExt = 0.0;              //V0.7
  uint16_t stvsnErrCode = 0;             //V0.8
};

// Union to store Iridium Short Burst Data (SBD) Mobile Originated (MO) messages
typedef union
{
  struct
  {
    uint32_t  unixtime;           // UNIX Epoch time                (4 bytes)
    int16_t   temperatureInt;     // Internal temperature (°C)      (2 bytes)   * 100
    uint16_t  humidityInt;        // Internal humidity (%)          (2 bytes)   * 100
    uint16_t  pressureExt;        // External pressure (hPa)        (2 bytes)   - 400 * 100
    int16_t   temperatureExt;     // External temperature (°C)      (2 bytes)   * 100
    uint16_t  humidityExt;        // External humidity (%)          (2 bytes)   * 100
    int16_t   pitch;              // Pitch (°)                      (2 bytes)   * 100
    int16_t   roll;               // Roll (°)                       (2 bytes)   * 100
    uint32_t  solar;              // Solar illuminance (lx)         (4 bytes)   * 10000
    uint16_t  windSpeed;          // Mean wind speed (m/s)          (2 bytes)   * 100
    uint16_t  windDirection;      // Mean wind direction (°)        (2 bytes)	  * 10
    uint16_t  windGustSpeed;      // Wind gust speed (m/s)          (2 bytes)   * 100
    uint16_t  windGustDirection;  // Wind gust direction (°)        (2 bytes)	  * 10
    int32_t   latitude;           // Latitude (DD)                  (4 bytes)   * 1000000
    int32_t   longitude;          // Longitude (DD)                 (4 bytes)   * 1000000
    uint8_t   satellites;         // # of satellites                (1 byte)
    uint16_t  hauteurNeige;       // Hauteur de neige (mm)          (2 bytes)   * 1
    uint16_t  voltage;            // Battery voltage (V)            (2 bytes)   * 100
    uint16_t  transmitDuration;   // Previous transmission duration (2 bytes)
    uint8_t   transmitStatus;     // Iridium return code            (1 byte)
    uint16_t  iterationCounter;   // Message counter                (2 bytes)
  } __attribute__((packed));                                    // Total: (48 bytes)
  uint8_t bytes[48];
} SBD_MO_MESSAGE;
static_assert(sizeof(SBD_MO_MESSAGE) <= 50, "Message structure exceeds a single credit (50 bytes).");
//static_assert(sizeof(SBD_MO_MESSAGE) * (1 + retransmitLimit) <= 340, "Retransmit limit too high for a single message (340 bytes).");
SBD_MO_MESSAGE moSbdMessage;

// Union to store received Iridium SBD Mobile Terminated (MT) message
typedef union
{
  struct
  {
    uint8_t   sampleInterval;     // 1 byte
    uint8_t   averageInterval;    // 1 byte
    uint8_t   transmitInterval;   // 1 byte
    uint8_t   retransmitLimit;    // 1 byte
    uint8_t   batteryCutoff;      // 1 byte
    uint8_t   resetFlag;          // 1 byte
  };
  uint8_t bytes[6]; // Size of message to be received in bytes
} SBD_MT_MESSAGE;

SBD_MT_MESSAGE mtSbdMessage;

// Structure to store device online/offline states and enabled/disabled flags
// Note: If initialized to `1` here, sensors will be disabled (never read);
//       Sensors set to `0` will be initialized automatically when needed.
struct struct_online
{
  bool bme280Ext = 1; // Derrière le bridge RS-485
  bool bme280Int = 0;
  bool lsm303   = 0;
  bool veml7700 = 1; // Derrière le bridge RS-485
  bool hmp60    = 1; //TODO Retirer
  bool sht31    = 1; //TODO Retirer
  bool wm5103L  = 1; //TODO Retirer
  bool di7911   = 1; //TODO Retirer
  bool sp212    = 1; //TODO Retirer
  bool dfrws    = 0; // Est le bridge RS-485 (TODO: Renommer)
  bool gnss     = INSOMNIAC;
  bool iridium  = INSOMNIAC;
  bool microSd  = 0;
} online;
const struct_online disabled;

// Structure to store function timers
struct struct_timer
{
  unsigned long readRtc;
  unsigned long readBattery;
  unsigned long readBme280Ext;
  unsigned long readBme280Int;
  unsigned long readLsm303;
  unsigned long readVeml7700;
  unsigned long readHmp60;
  unsigned long readSht31;
  unsigned long read5103L;
  unsigned long read7911;
  unsigned long readSp212;
  unsigned long readDFRWS;
  unsigned long readGnss;
  unsigned long writeMicroSd;
  unsigned long iridium;
} timer;

// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------
void setup()
{
  // Clear memory structures
  memset(&moSbdMessage, 0, sizeof(moSbdMessage));
  memset(&mtSbdMessage, 0, sizeof(mtSbdMessage));
  memset(&online, 0, sizeof(online));
  memset(&timer, 0, sizeof(timer));

  // Pin assignments
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_SENSOR_PWR, OUTPUT);
  pinMode(PIN_5V_EN, OUTPUT);
  pinMode(PIN_12V_EN, OUTPUT);
  pinMode(PIN_GNSS_EN, OUTPUT);
  pinMode(PIN_VBAT, INPUT);
  digitalWrite(PIN_LED_GREEN, LOW);   // Disable green LED
  digitalWrite(PIN_LED_RED, LOW);     // Disable red LED
  digitalWrite(PIN_SENSOR_PWR, LOW);  // Disable power to 3.3V
  digitalWrite(PIN_5V_EN, LOW);       // Disable power to Iridium 9603
  digitalWrite(PIN_12V_EN, LOW);      // Disable 12V power
  digitalWrite(PIN_GNSS_EN, HIGH);    // Disable power to GNSS

  // Configure analog-to-digital (ADC) converter
  configureAdc();

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock speed to 400 kHz

#if DEBUG || INSOMNIAC
  SERIAL_PORT.begin(115200); // Open serial port at 115200 baud
  blinkLed(PIN_LED_GREEN, 10, 500); // Non-blocking delay to allow user to open and clear Serial Monitor
#endif

  DEBUG_PRINTLN();
  DEBUG_PRINTLN();
  DEBUG_PRINTLN();
  printLine();
  DEBUG_PRINT("Cryologger - Automatic Weather Station #"); DEBUG_PRINTLN(CRYOLOGGER_ID);
  printLine();

#if CALIBRATE
  enable5V();   // Enable 5V power
  enable12V();  // Enable 12V power

  while (true)
  {
    petDog(); // Reset WDT
    int fram = freeRam();
    DEBUG_PRINT(">  Fram initial: "); DEBUG_PRINTLN(fram);

    calibrateAdc();
    DEBUG_PRINT(">  (ADC) Fram state: "); DEBUG_PRINTLN(freeRam());

    readBattery();        // Read battery voltage
    DEBUG_PRINT(">  (Bat) Fram state: "); DEBUG_PRINTLN(freeRam());

    readBme280Int();  // Read temperature and humidty sensor (external)
    DEBUG_PRINT(">  (BME280Int) Fram state: "); DEBUG_PRINTLN(freeRam());

    readBme280Ext();     // Read temperature and humidty sensor (external)
    DEBUG_PRINT(">  (BME280Ext) Fram state: "); DEBUG_PRINTLN(freeRam());

    readLsm303();
    DEBUG_PRINT(">  (LSM303) Fram state: "); DEBUG_PRINTLN(freeRam());

    readVeml7700();    // Read solar radiation
    DEBUG_PRINT(">  (VEML7700) Fram state: "); DEBUG_PRINTLN(freeRam());

    readDFRWindSensor(); //TODO Rename me
    DEBUG_PRINT(">  (DFRWS) Fram state: "); DEBUG_PRINTLN(freeRam());

    //readGnss(); // Sync RTC with the GNSS

    DEBUG_PRINT(">  Fram final change: "); DEBUG_PRINTLN(freeRam() - fram);
    myDelay(5000);
    printWakeUp(++sampleCounter);
  }
#endif

  // Configure devices
  configureRtc();       // Configure real-time clock (RTC)
  configureWdt();       // Configure Watchdog Timer (WDT)
  printSettings();      // Print configuration settings
  readBattery();        DEBUG_PRINT("Info - Battery voltage: "); DEBUG_PRINTLN(voltage);
  readGnss();           // Sync RTC with GNSS
  newDay();             // Update current date
  configureIridium();   // Configure Iridium 9603 transceiver
  configureSd();        // Configure microSD
  createLogFile();      // Create initial log file

  // Blink LED to indicate completion of setup
  for (byte i = 0; i < 10; i++)
  {
    // Blink LEDs
    digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED));
    digitalWrite(PIN_LED_GREEN, !digitalRead(PIN_LED_GREEN));
    myDelay(250);
  }

  //SERIAL_PORT.println("> STARTING");
}

// ----------------------------------------------------------------------------
// Loop
// ----------------------------------------------------------------------------
void loop()
{
  // Check if RTC alarm triggered or if program is running for first time
  if (INSOMNIAC || alarmFlag || firstTimeFlag)
  {
    // Read the RTC
    readRtc();

    // Reset WDT
    petDog();

    // Increment the sample counter
    sampleCounter++;

    // Check if program is running for the first time
    if (!firstTimeFlag)
    {
      // Wake from deep sleep
      wakeUp();

      // Print date and time
      DEBUG_PRINT("Info - Alarm triggered at "); printDateTime();
    }

    // Read battery voltage
    readBattery();

    // Check if battery voltage is above cutoff threshold
    if (voltage < batteryCutoff)
    {
      cutoffCounter++;

      // In the event that the battery voltage never recovers, force a reset of the
      // system after 1 week
      if (cutoffCounter > 7 * 24 * (60/sampleInterval))
      {
        forceReset(); // Force WDT reset
      }

      DEBUG_PRINTLN("Warning - Battery voltage cutoff exceeded. Entering deep sleep...");

      /* I understand the idea of discarding the whole dataset in this scenario, but it can lead to a scenario where
         we no longer receive data because we can't reliably complete 15 full cycles.

      // Reset sample counter
      sampleCounter = 0;

      // Clear statistics objects
      clearStats();
      */

      // Go to sleep
      setCutoffAlarm();
    }
    else
    {
      DEBUG_PRINT("Info - Battery voltage good: "); DEBUG_PRINTLN(voltage);

      cutoffCounter = 0;

      enable5V();         // Enable 5V power
      enable12V();        // Enable 12V power

      // Perform measurements
      receiveCommand();
      if (disabled.bme280Ext)
        DEBUG_PRINTLN("Info - BME280 Ext disabled");
      else
        readBme280Ext();     // Read temperature and humidty sensor (external)

      receiveCommand();
      if (disabled.bme280Int)
        DEBUG_PRINTLN("Info - BME280 Int disabled");
      else
        readBme280Int();  // Read temperature and humidty sensor (internal)

      receiveCommand();
      if (disabled.lsm303)
        DEBUG_PRINTLN("Info - LS303 disabled");
      else
        readLsm303();     // Read accelerometer

      receiveCommand();
      if (disabled.veml7700)
        DEBUG_PRINTLN("Info - VEMLM7700 disabled");
      else
        readVeml7700();

      receiveCommand();
      if (disabled.dfrws)
        DEBUG_PRINTLN("Info - DFRWS disabled");
      else
        readDFRWindSensor(); // Read anemometer and windDirection

      // Print summary of statistics
      printStats();

      // Check if number of samples collected has been reached and calculate statistics (if enabled)
      if ((sampleCounter == averageInterval) || firstTimeFlag)
      {
        calculateStats(); // Calculate statistics of variables to be transmitted
        writeBuffer();    // Write data to transmit buffer

        // Check if data transmission interval has been reached
        if ((transmitCounter == transmitInterval) || firstTimeFlag)
        {
          if (newDay()) // Check for date change
            readGnss(); // Sync RTC with the GNSS

          if (transmitData()) // Transmit data via Iridium transceiver
            printSettings(); // Print current settings (in case they changed)
        }

        clearStats(); // This also resets the sample counter
      }

      // Log data to microSD card
      logData();

      // Print function execution timers
      printTimers();

      // Set the RTC alarm
      if (!INSOMNIAC)
        setRtcAlarm();

      // Prepare for sleep
      prepareForSleep();
    }
  }
  
  // Check for WDT interrupts
  if (wdtFlag)
  {
    if (INSOMNIAC || checkAlarm())
    {
      // Blink LED to indicate WDT interrupt and nominal system operation
      blinkLed(PIN_LED_GREEN, 1, 50);

      // Reset the WDT
      petDog();
    }
    else
    {
      // Blink red LED to indicate abnormal system operation
      blinkLed(PIN_LED_RED, 1, 250);

      // Reset the RTC alarm based on current time
      setCutoffAlarm(); //TODO Clarify what cutOffAlarm should really do
    }
  }

  // Clear first-time flag after initial power-down
  if (firstTimeFlag) {
    firstTimeFlag = false;
    //SERIAL_PORT.println("> READY");
  }

  // Enter deep sleep and wait for WDT or RTC alarm interrupt
  goToSleep();
}

// ----------------------------------------------------------------------------
// Gestion des commandes
// ----------------------------------------------------------------------------
#define reply(var) _reply(var, F(#var))

#define FOREACH_STATUSVAR(M) M(sampleCounter) M(iterationCounter) M(transmitStatus)
#define FOREACH_SETTING(M) M(sampleInterval) M(averageInterval) M(transmitInterval)
#define FOREACH_PARAM(M) \
            M(tempBmeINT_Offset) M(tempBmeINT_CF) M(humBmeINT_Offset) M(humBmeINT_CF) M(presBmeINT_Offset) M(presBmeINT_CF) \
            M(tempBmeEXT_Offset) M(tempBmeEXT_CF) M(humBmeEXT_Offset) M(humBmeEXT_CF) M(presBmeEXT_Offset) M(presBmeEXT_CF)

#define _GET(var) else if (command.equalsIgnoreCase(F(#var))) { reply(var); }
#define _SET(var, val) else if (command.equalsIgnoreCase(F(#var))) { var = val; reply(var); }
#define _SET_ARG(var) _SET(var, static_cast<__typeof(var)>(arg))

template<typename T, typename S>
void _reply(T value, S* ident = nullptr) {
    SERIAL_PORT.write('>');
    SERIAL_PORT.write(' ');
    if (ident) {
        DEBUG_PRINT(ident);
        DEBUG_PRINT(' ');
        DEBUG_PRINT('=');
        DEBUG_PRINT(' ');
    }
    SERIAL_PORT.println(value);
    SERIAL_PORT.flush();
}

int receiveCommand() {
    if (SERIAL_PORT.available() <= 0)
        return 0;

    String command = SERIAL_PORT.readString();
    command.trim();
    SERIAL_PORT.print("< ");
    SERIAL_PORT.println(command); // Les fins de ligne sont optionnels après les commandes (mais attention au baud rate)

    String COMMAND = command;
    COMMAND.toUpperCase();
    if (COMMAND.startsWith("READ")) {
        command = command.substring(5);
        long arg = command.length() > 0 ? command.toInt() : 1;
        if (!arg) {
            int idx = COMMAND.indexOf('.');
            COMMAND = idx < 0 ? "INT" : COMMAND.substring(idx + 1);
            if (COMMAND == "INT")
                idx = 0;
            else if (COMMAND == "EXT")
                idx = 1;
            else {
                SERIAL_PORT.print("! INVALID ARGUMENT: ");
                SERIAL_PORT.println(command);
                return -2;
            }
            switch (toupper(command[0])) {
                case 'V': reply(voltage); break;
                case 'T': reply(idx ? temperatureExt : temperatureInt); break;
                case 'P': reply(idx ? pressureExt : pressureInt); break;
                case 'H': reply(idx ? humidityExt : humidityInt); break;
                default:
                    SERIAL_PORT.print("! INVALID ARGUMENT: ");
                    SERIAL_PORT.println(command);
                    return -2;
            }
        }
        else if ((long)sampleCounter < arg) {
            SERIAL_PORT.println("! NOT ENOUGH SAMPLES");
            return -4;
        }
        else {
            DEBUG_PRINT("Sending averaged data (");
            DEBUG_PRINT(sampleCounter);
            DEBUG_PRINTLN(" samples)");
            calculateStats();
            SERIAL_PORT.write('>');
            SERIAL_PORT.write(' ');
            SERIAL_PORT.write(moSbdMessage.bytes, sizeof(moSbdMessage));
            SERIAL_PORT.write('\n');
            SERIAL_PORT.flush();
        }
    }
    else if (COMMAND.startsWith("GET")) {
        command = command.substring(4);
        if (command.length() == 0) {
            SERIAL_PORT.println("! MISSING ARGUMENT <param>");
            return -3;
        }
        else if (command.equalsIgnoreCase("time")) {
            _reply(rtc.getEpoch(), "time");
        }
        FOREACH_STATUSVAR(_GET)
        FOREACH_SETTING(_GET)
        FOREACH_PARAM(_GET)
        else {
            SERIAL_PORT.print("! UNKNOWN PARAMETER: ");
            SERIAL_PORT.println(command);
            return -4;
        }
    }
    else if (COMMAND.startsWith("SET")) {
        command = command.substring(4);
        if (command.length() == 0) {
            SERIAL_PORT.println("! MISSING ARGUMENTS <param> <value>");
            return -3;
        }
        int idx = command.indexOf(' ');
        double arg = command.substring(idx + 1).toDouble();
        if (!arg) {
            if (idx < 0) {
                SERIAL_PORT.println("! MISSING ARGUMENT <value>");
                return -3;
            }
            else {
                SERIAL_PORT.print("! INVALID VALUE: ");
                SERIAL_PORT.println(command.substring(idx + 1));
                return -2;
            }
        }
        command = command.substring(0, max(idx, 0));
        if (command.length() == 0) {
            SERIAL_PORT.println("! MISSING ARGUMENT <param>");
            return -3;
        }
        else if (command.equalsIgnoreCase("time")) {
            rtc.setEpoch((uint32_t)arg);
            _reply((uint32_t)arg, "time");
        }
        FOREACH_SETTING(_SET_ARG)
        FOREACH_PARAM(_SET_ARG)
        else {
            SERIAL_PORT.print("! UNKNOWN PARAMETER: ");
            SERIAL_PORT.println(command);
            return -4;
        }
    }
    else if (COMMAND.equals("RESET")) {
        SERIAL_PORT.println("> RESETTING");
        forceReset();
    }
    else if (COMMAND.equals("STATUS") || COMMAND.length() == 0) {
        if (firstTimeFlag)
            SERIAL_PORT.println("> INITIALIZING");
        else
            SERIAL_PORT.println("> READY");
    }
    else {
        SERIAL_PORT.print("! COMMAND NOT RECOGNIZED: ");
        SERIAL_PORT.println(command);
        return -1;
    }
    return 1;
}
