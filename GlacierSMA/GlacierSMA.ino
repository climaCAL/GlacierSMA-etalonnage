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
#include <IridiumSBD.h>             // https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library (v3.0.5)
#include <RTCZero.h>                // https://github.com/arduino-libraries/RTCZero (v1.6.0)
#include <SdFat.h>                  // https://github.com/greiman/SdFat (v2.1.2)
#include <sensirion.h>              // https://github.com/HydroSense/sensirion
#include <Statistic.h>              // https://github.com/RobTillaart/Statistic (v1.0.0)
#include <TimeLib.h>                // https://github.com/PaulStoffregen/Time (v1.6.1)
#include <TinyGPS++.h>              // https://github.com/mikalhart/TinyGPSPlus (v1.0.3)
#include <Wire.h>                   // https://www.arduino.cc/en/Reference/Wire
#include <wiring_private.h>         // Required for creating new Serial instance
#include "Adafruit_VEML7700.h"      // Add docs

// ----------------------------------------------------------------------------
// Define unique identifier
// ----------------------------------------------------------------------------
#define CRYOLOGGER_ID "AL2"

// ----------------------------------------------------------------------------
// Data logging
// ----------------------------------------------------------------------------
#define LOGGING         true  // Log data to microSD

// ----------------------------------------------------------------------------
// Define modifed addresses
// ----------------------------------------------------------------------------
#define BME280_ADR2 0x76    // Second addresse for the BME280 - Used for the interiror sensor.

// ----------------------------------------------------------------------------
// Debugging macros
// ----------------------------------------------------------------------------
#define DEBUG           true  // Output debug messages to Serial Monitor
#define DEBUG_GNSS      false // Output GNSS debug information
#define DEBUG_IRIDIUM   false // Output Iridium debug messages to Serial Monitor
#define CALIBRATE       true // Enable sensor calibration code

#if DEBUG
#define DEBUG_PRINT(x)            SERIAL_PORT.print(x)
#define DEBUG_PRINTLN(x)          SERIAL_PORT.println(x)
#define DEBUG_PRINT_HEX(x)        SERIAL_PORT.print(x, HEX)
#define DEBUG_PRINTLN_HEX(x)      SERIAL_PORT.println(x, HEX)
#define DEBUG_PRINT_DEC(x, y)     SERIAL_PORT.print(x, y)
#define DEBUG_PRINTLN_DEC(x, y)   SERIAL_PORT.println(x, y)
#define DEBUG_WRITE(x)            SERIAL_PORT.write(x)

#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT_HEX(x)
#define DEBUG_PRINTLN_HEX(x)
#define DEBUG_PRINT_DEC(x, y)
#define DEBUG_PRINTLN_DEC(x, y)
#define DEBUG_WRITE(x)
#endif

// ----------------------------------------------------------------------------
// Pin definitions
// ----------------------------------------------------------------------------
#define PIN_VBAT            A0
#define PIN_WIND_SPEED      A1
#define PIN_WIND_DIR        A2
#define PIN_HUMID           A3
#define PIN_TEMP            A4
#define PIN_GNSS_EN         A5
#define PIN_MICROSD_CS      4
#define PIN_12V_EN          5   // 12 V step-up/down regulator
#define PIN_5V_EN           6   // 5V step-down regulator
#define PIN_LED_GREEN       8   // Green LED
#define PIN_IRIDIUM_RX      10  // Pin 1 RXD (Yellow)
#define PIN_IRIDIUM_TX      11  // Pin 6 TXD (Orange)
#define PIN_IRIDIUM_SLEEP   12  // Pin 7 OnOff (Grey)
#define PIN_LED_RED         13

// Unused
#define PIN_SOLAR           7
#define PIN_SENSOR_PWR      7
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

//WindSensor module I2C address declaration:
const uint8_t WIND_SENSOR_SLAVE_ADDR = 0x66;

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
typedef statistic::Statistic<float,uint32_t,false> StatisticCAL;
StatisticCAL batteryStats;         // Battery voltage
StatisticCAL temperatureIntStats;  // Internal temperature
StatisticCAL humidityIntStats;     // Internal humidity
StatisticCAL pressureIntStats;     // Internal pressure
StatisticCAL temperatureExtStats;  // External temperature
StatisticCAL humidityExtStats;     // External humidity
StatisticCAL solarStats;           // Solar radiation
StatisticCAL windSpeedStats;       // Wind speed
StatisticCAL uStats;               // Wind east-west wind vector component (u)
StatisticCAL vStats;               // Wind north-south wind vector component (v)

// ----------------------------------------------------------------------------
// User defined global variable declarations
// ----------------------------------------------------------------------------
#if DEBUG
unsigned long sampleInterval    = 1;      // Sampling interval (minutes). Default: 5 min (300 seconds)
#else
unsigned long sampleInterval    = 5;      // Sampling interval (minutes). Default: 5 min (300 seconds)
#endif
unsigned int  averageInterval   = 12;     // Number of samples to be averaged in each message. Default: 12 (hourly)
unsigned int  transmitInterval  = 1;      // Number of messages in each Iridium transmission (340-byte limit)
unsigned int  retransmitLimit   = 4;      // Failed data transmission reattempts (340-byte limit)
#if DEBUG_GNSS
unsigned int  gnssTimeout       = 30;     // Timeout for GNSS signal acquisition (seconds)
#else
unsigned int  gnssTimeout       = 120;    // Timeout for GNSS signal acquisition (seconds)
#endif
unsigned int  iridiumTimeout    = 180;    // Timeout for Iridium transmission (seconds)
bool          firstTimeFlag     = true;   // Flag to determine if program is running for the first time
#if DEBUG
float         batteryCutoff     = 3.0;    // Battery voltage cutoff threshold (V)
#else
float         batteryCutoff     = 11.0;    // Battery voltage cutoff threshold (V)
#endif
byte          loggingMode       = 2;      // Flag for new log file creation. 1: daily, 2: monthly, 3: yearly

// ----------------------------------------------------------------------------
// Sensors correction factor and offsets -- to modify -- 
// ----------------------------------------------------------------------------
//BME280 -- Exterior sensor
float tempBmeEXT_CF             = 1.046;    // Correction factor for exterior temperature acquisition.
float tempBmeEXT_Offset         = -0.805;   // Offset for exterior temperature acquisition.
float humBmeEXT_CF              = 1.09;     // Correction factor for exterior humidity acquisition.
float humBmeEXT_Offset          = 2.3;      // Offset for exterior humidity acquisition.

//BME280 -- Interior sensor
float tempImeINT_CF             = 1.05;     // Correction factor for interior temperature acquisition.
float tempBmeINT_Offset         = -1.07;    // Offset for interior temperature acquisition.
float humImeINT_CF              = 1.0;      // Correction factor for interior humidity acquisition.
float humBmeINT_Offset          = 0.0;      // Offset for interior humidity acquisition.

//VEML7700
float veml_CF                   = 15.172;   // Correction factor for light intensity acquisition.
float veml_Offset               = -998;     // Offset for light intensity acquisition.

// ----------------------------------------------------------------------------
// Global variable declarations
// ----------------------------------------------------------------------------
volatile bool alarmFlag         = false;  // Flag for alarm interrupt service routine
volatile bool wdtFlag           = false;  // Flag for Watchdog Timer interrupt service routine
volatile int  wdtCounter        = 0;      // Watchdog Timer interrupt counter
volatile int  revolutions       = 0;      // Wind speed ISR counter
bool          resetFlag         = false;  // Flag to force system reset using Watchdog Timer
uint8_t       moSbdBuffer[340];           // Buffer for Mobile Originated SBD (MO-SBD) message (340 bytes max)
uint8_t       mtSbdBuffer[270];           // Buffer for Mobile Terminated SBD (MT-SBD) message (270 bytes max)
size_t        moSbdBufferSize;
size_t        mtSbdBufferSize;
char          logFileName[30]   = "";     // Log file name
char          dateTime[30]      = "";     // Datetime buffer
byte          retransmitCounter = 0;      // Counter for Iridium 9603 transmission reattempts
byte          transmitCounter   = 0;      // Counter for Iridium 9603 transmission intervals
byte          currentLogFile    = 0;      // Variable for tracking when new microSD log files are created
byte          newLogFile        = 0;      // Variable for tracking when new microSD log files are created
byte          currentDate       = 0;      // Variable for tracking when the date changes
byte          newDate           = 0;      // Variable for tracking when the date changes
int           transmitStatus    = 0;      // Iridium transmission status code
unsigned int  iterationCounter  = 0;      // Counter for program iterations (zero indicates a reset)
unsigned int  failureCounter    = 0;      // Counter of consecutive failed Iridium transmission attempts
unsigned long previousMillis    = 0;      // Global millis() timer
unsigned long alarmTime         = 0;      // Global epoch alarm time variable
unsigned long unixtime          = 0;      // Global epoch time variable
unsigned int  sampleCounter     = 0;      // Sensor measurement counter
unsigned int  cutoffCounter     = 0;      // Battery voltage cutoff sleep cycle counter
unsigned long samplesSaved      = 0;      // Log file sample counter
long          rtcDrift          = 0;      // RTC drift calculated during sync
float         temperatureInt    = 0.0;    // Internal temperature (°C)
float         humidityInt       = 0.0;    // Internal hunidity (%)
float         pressureInt       = 0.0;    // Internal pressure (hPa)
float         temperatureExt    = 0.0;    // External temperature (°C)
float         humidityExt       = 0.0;    // External humidity (%)
float         pitch             = 0.0;    // Pitch (°)
float         roll              = 0.0;    // Roll (°)
float         solar             = 0.0;    // Solar radiation
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
// Attention structure prévue pour la hauteur de neige ET PAS UTILISÉE ICI
// regMemoryMap[0] = direction vent en degrés (0-360)
// regMemoryMap[1] = direction vent en secteur (0-15)
// regMemoryMap[2] = vitesse vent en m/s *10
// regMemoryMap[3] = hauteur de neige en mm
// regMemoryMap[4] = temperature de reference pour la mesure hauteur de neige, em Celcius resolution de 1C
typedef struct {
  uint16_t regMemoryMap[5] = {0,0,0,0,0};  //total 5 words = 10 bytes; utile: 3 bytes (2 derniers byte seront pour hauteur de neige, futur)
  float angleVentFloat = 0;
  uint16_t directionVentInt = 0;
  float vitesseVentFloat = 0;
  float hauteurNeige = 0;  //Futur
  float temperatureHN = 0; //Futur
} vent;

const uint8_t ventRegMemMapSize = 0x06;  //6 = 3 capteurs * 2 bytes (instruments de vent seulement)

// Union to store Iridium Short Burst Data (SBD) Mobile Originated (MO) messages
typedef union
{
  struct
  {
    uint32_t  unixtime;           // UNIX Epoch time                (4 bytes)
    int16_t   temperatureInt;     // Internal temperature (°C)      (2 bytes)   * 100
    uint16_t  humidityInt;        // Internal humidity (%)          (2 bytes)   * 100
    uint16_t  pressureInt;        // Internal pressure (hPa)        (2 bytes)   - 850 * 100
    int16_t   temperatureExt;     // External temperature (°C)      (2 bytes)   * 100
    uint16_t  humidityExt;        // External humidity (%)          (2 bytes)   * 100
    int16_t   pitch;              // Pitch (°)                      (2 bytes)   * 100
    int16_t   roll;               // Roll (°)                       (2 bytes)   * 100
    uint32_t  solar;              // Solar illuminance (Lux)        (4 bytes)   * 10
    uint16_t  windSpeed;          // Mean wind speed (m/s)          (2 bytes)   * 100
    uint16_t  windDirection;      // Mean wind direction (°)        (2 bytes)	* 10
    uint16_t  windGustSpeed;      // Wind gust speed (m/s)          (2 bytes)   * 100
    uint16_t  windGustDirection;  // Wind gust direction (°)        (2 bytes)	* 10
    int32_t   latitude;           // Latitude (DD)                  (4 bytes)   * 1000000
    int32_t   longitude;          // Longitude (DD)                 (4 bytes)   * 1000000
    uint8_t   satellites;         // # of satellites                (1 byte)
    uint16_t  hdop;               // HDOP                           (2 bytes)
    uint16_t  voltage;            // Battery voltage (V)            (2 bytes)   * 100
    uint16_t  transmitDuration;   // Previous transmission duration (2 bytes)
    uint8_t   transmitStatus;     // Iridium return code            (1 byte)
    uint16_t  iterationCounter;   // Message counter                (2 bytes)
  } __attribute__((packed));                                    // Total: (48 bytes)
  uint8_t bytes[48];
} SBD_MO_MESSAGE;

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
  bool bme280Ext = 0;
  bool bme280Int = 0;
  bool dfrWindSensor = 0;
  bool di7911   = 1; //TODO Retirer
  bool hmp60    = 1; //TODO Retirer
  bool lsm303   = 0;
  bool sht31    = 1; //TODO Retirer
  bool sp212    = 1; //TODO Retirer
  bool veml7700 = 0;
  bool wm5103L  = 1; //TODO Retirer
  bool gnss     = 0;
  bool microSd  = 0; // Set to 0 initially otherwise the SD card will not be initialized
} disabled, online;

// Structure to store function timers
struct struct_timer
{
  unsigned long readRtc;
  unsigned long readBattery;
  unsigned long configMicroSd;
  unsigned long writeMicroSd;
  unsigned long readGnss;
  unsigned long readBme280;
  unsigned long readVeml7700;
  unsigned long readLsm303;
  unsigned long readHmp60;
  unsigned long readSht31;
  unsigned long read5103L;
  unsigned long read7911;
  unsigned long readDFRWS;
  unsigned long readSp212;
  unsigned long iridium;
} timer;

// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------
void setup()
{
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

#if DEBUG
  SERIAL_PORT.begin(115200); // Open serial port at 115200 baud
  blinkLed(PIN_LED_GREEN, 4, 500); // Non-blocking delay to allow user to open Serial Monitor
#endif

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
    DEBUG_PRINT(">  (A) Fram state: ");  // monitore l'etat de la RAM
    DEBUG_PRINTLN(freeRam());  
    calibrateAdc();
    DEBUG_PRINT(">  (B1) Fram state: ");  // monitore l'etat de la RAM
    DEBUG_PRINTLN(freeRam());
    readBme280Int();  // Read temperature and humidty sensor (external)
    DEBUG_PRINT(">  (B2) Fram state: ");  // monitore l'etat de la RAM
    DEBUG_PRINTLN(freeRam());  
    readBme280Ext();     // Read temperature and humidty sensor (external)
    DEBUG_PRINT(">  (C) Fram state: ");  // monitore l'etat de la RAM
    DEBUG_PRINTLN(freeRam());  
    readLsm303();
    DEBUG_PRINT(">  (D) Fram state: ");  // monitore l'etat de la RAM
    DEBUG_PRINTLN(freeRam());
    readVeml7700();    // Read solar radiation - Attention (09/28/23 Yh) si le VEML7700 n'est pas connecté, le code bloque... corrigé. Cause: le destructeur. Donc déclaré global.
    DEBUG_PRINT(">  (E) Fram state: ");  // monitore l'etat de la RAM
    DEBUG_PRINTLN(freeRam());
    readDFRWindSensor();
    DEBUG_PRINT(">  (F) Fram state: ");  // monitore l'etat de la RAM
    DEBUG_PRINTLN(freeRam());
    //readGnss(); // Sync RTC with the GNSS
    readBattery();        // Read battery voltage

    DEBUG_PRINT(">  Fram final  : ");  // monitore l'etat de la RAM
    DEBUG_PRINTLN(freeRam());  

    myDelay(5000);
  }
#endif

  // Configure devices
  configureRtc();       // Configure real-time clock (RTC)
  readRtc();            // Read date and time from RTC
  configureWdt();       // Configure Watchdog Timer (WDT)
  readBattery();        // Read battery voltage
  configureSd();        // Configure microSD
  printSettings();      // Print configuration settings
  readGnss();           // Sync RTC with GNSS
  configureIridium();   // Configure Iridium 9603 transceiver
  createLogFile();      // Create initial log file

  // Close serial port if immediately entering deep sleep
  if (!firstTimeFlag)
  {
    disableSerial();
  }

  // Blink LED to indicate completion of setup
  for (byte i = 0; i < 10; i++)
  {
    // Blink LEDs
    digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED));
    digitalWrite(PIN_LED_GREEN, !digitalRead(PIN_LED_GREEN));
    myDelay(250);
  }
}

// ----------------------------------------------------------------------------
// Loop
// ----------------------------------------------------------------------------
void loop()
{
  // Check if RTC alarm triggered or if program is running for first time
  if (alarmFlag || firstTimeFlag)
  {
    // Read the RTC
    readRtc();

    // Print date and time
    DEBUG_PRINT("Info - Alarm trigger: "); printDateTime();

    // Reset WDT
    petDog();

    // Increment the sample counter
    sampleCounter++;

    // Check if program is running for the first time
    if (!firstTimeFlag)
    {
      // Wake from deep sleep
      wakeUp();
      printWakeUp(sampleCounter);
    }

    // Read battery voltage
    readBattery();

    // Check if battery voltage is above cutoff threshold
    if (voltage < batteryCutoff)
    {
      cutoffCounter++;

      // In the event that the battery voltage never recovers, force a reset of the
      // system after 1 week
      if (cutoffCounter > 168)
      {
        // Force WDT reset
        while (1);
      }

      DEBUG_PRINTLN("Warning - Battery voltage cutoff exceeded. Entering deep sleep...");

      // Reset sample counter
      sampleCounter = 0;

      // Clear statistics objects
      clearStats();

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
      if (disabled.bme280Ext)
        DEBUG_PRINTLN("Info - BME280 Ext disabled");
      else
        readBme280Ext();     // Read temperature and humidty sensor (external)

      if (disabled.bme280Int)
        DEBUG_PRINTLN("Info - BME280 Int disabled");
      else
        readBme280Int();  // Read temperature and humidty sensor (internal)

      if (disabled.lsm303)
        DEBUG_PRINTLN("Info - LS303 disabled");
      else
        readLsm303();     // Read accelerometer

      if (disabled.veml7700)
        DEBUG_PRINTLN("Info - VEMLM7700 disabled");
      else
        readVeml7700();

      if (disabled.dfrWindSensor)
        DEBUG_PRINTLN("Info - DFR disabled");
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
          // Check for date change
          checkDate();
          if (firstTimeFlag || (currentDate != newDate))
          {
            readGnss(); // Sync RTC with the GNSS
            currentDate = newDate;
            Serial.print("currentDate: "); Serial.println(currentDate);
            Serial.print("newDate: "); Serial.println(newDate);
          }
          #if DEBUG
          if (currentDate != newDate)
            transmitData(); // Transmit only once per day
          #else
          transmitData(); // Transmit data via Iridium transceiver
          #endif
        }
        sampleCounter = 0; // Reset sample counter
      }

      // Log data to microSD card
      logData();

      // Print function execution timers
      printTimers();

      // Set the RTC alarm
      setRtcAlarm();

      //Ok, we're done, let's shutdown things
      disable12V();      // Disable 12V power
      disable5V();       // Disable 5V power

      DEBUG_PRINTLN("Info - Entering deep sleep...");
      DEBUG_PRINTLN();

      // Prepare for sleep
      prepareForSleep();
    }
  }
  // Check for WDT interrupts
  if (wdtFlag)
  {
    petDog(); // Reset the WDT
  }

  // Blink LED to indicate WDT interrupt and nominal system operation
  blinkLed(PIN_LED_GREEN, 1, 25);

  // Enter deep sleep and wait for WDT or RTC alarm interrupt
  goToSleep();
}
