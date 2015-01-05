/* Bicycle Monitor
 Teensy 3.1
 Adafruit Ultimate GPS
 Adafruit BMP180 - Barometric Pressure / Temperature / Altitude Sensor
 By Ryan Heyser
 */

#include <Adafruit_GPS.h> //Arduino Ultimate GPS
#include <Adafruit_BMP085_U.h> //BMP180
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <LowPower_Teensy3.h>
#include <Wire.h>
#include <Time.h>
#include <Timezone.h>

//CPU
TEENSY3_LP LP = TEENSY3_LP();
HardwareSerial_LP lphwserial1 = HardwareSerial_LP();
IntervalTimer_LP lpwritingTimer;

//#define GPSSERIAL lphwserial1 //RX = Pin 0, TX = Pin 1

#define DEBUG //Debug data to serial

#ifdef DEBUG
#define DEBUG_PRINT(str) \
Serial.println(str); 
//Serial.flush();
#else
#define DEBUG_PRINT(str)
#endif

#ifdef DEBUG  
#define DEBUG_START_SERIAL() \
Serial.begin(9600); //115200
#else
#define DEBUG_START_SERIAL()
#endif

#ifdef DEBUG  
#define DEBUG_STOP_SERIAL() \
Serial.end();
#else
#define DEBUG_STOP_SERIAL()
#endif

#ifdef DEBUG  
#define SLEEP(sec) \
delay(sec*1000);
#else
#define SLEEP(sec) \
LP.DeepSleep(RTCA_WAKE, sec);
#endif

#ifdef DEBUG  
#define DEEPSLEEP() \
//delay(sec*1000);
#else
#define DEEPSLEEP() \
LP.DeepSleep(GPIO_WAKE, PIN_2, writing);
#endif

//Timer
int timerfrequency(int sec) {
  return (sec*1000000); //1000000 = 1 second
}
//#define TIMERFREQUENCY(1000000*sec);
IntervalTimer writingTimer;

//GPS
Adafruit_GPS GPS(&lphwserial1); 
bool isFixed = false;
const int timeoffset = -5; //EST

//setupdone
bool setupdone = false;

//BMP180
//SCL pin 19
//SDA pin 18
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(18000);

//SD
//MOSI pin 11
//MISO pin 12
//CLK pin 13
//CS pin 10
File logFile;
char filename [15];


#define BACK_FROM_SLEEP() \
Serial.begin(9600); \
GPS.begin(9600); \ 
bmp.begin(); \
delay(100); 
//LP.CPU(F_CPU); 
//DEBUG_PRINT("BACK FROM SLEEP");

void setup()
{
  
  //noInterrupts();
  //delay(5000);

  /* GPS INIT */
  GPS.begin(9600);

  //GPS Setup Baud to 115200
  //GPS.sendCommand(PMTK_SET_BAUD_115200);

  //GPS.end();
  //delay(1000);
  //GPS.begin(115200);

  DEBUG_START_SERIAL();
  
  //pinMode(2, INPUT_PULLUP);
  //LP.CPU(TWO_MHZ);
  lpwritingTimer.begin(writing, 1000000); //timerfrequency(10)
  noInterrupts();
 //LP.CPU(F_CPU);
  
  //delay(2000);
  //BACK_FROM_SLEEP();
  DEBUG_PRINT("Starting debug test:");

  // You can adjust which sentences to have the module emit, below

  // uncomment this to force a reboot state
  //GPS.sendCommand(PMTK_CMD_COLD_START);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data for high update rates!
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);

  // Set the update rate
  // 1 Hz update rate
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  // Turn off updates on antenna status, if the firmware permits
  GPS.sendCommand(PGCMD_NOANTENNA);

  DEBUG_PRINT("GPS Ready");

  /* SD INIT */
  strcpy(filename, "BIKELOG0000.TXT");

  pinMode(10, OUTPUT);

  if(!SD.begin(10))
  {
    DEBUG_PRINT("SD Initialization Failed");
  }  
  else
  {
    DEBUG_PRINT("SD Initialization Succeeded");
  }

  for(int i = 0; i<10000; i++)
  {
    //modify values 7-10 before overwriting
    filename[7] = '0' + i/10;
    filename[8] = '0' + i/100;
    filename[9] = '0' + i/1000;
    filename[10] = '0' + i%1000;

    if(!SD.exists(filename))
    {
      SD.open(filename, FILE_WRITE);
      logFile.close();
      DEBUG_PRINT(filename);        
      break;     
    }
  }

  /* Sensor Init */
  if(!bmp.begin())
  {
    DEBUG_PRINT("BMP180 Not Found");        
    //while(1); //delay until found
  }

  //Serial.end();
  
  DEBUG_PRINT("Starting Timer");
  
  
  //attachInterrupt(2, writing, FALLING);
  DEBUG_PRINT("SETUP DONE");
  setupdone = true;
  //interrupts();
}

void running()
{
  //DEBUG_START_SERIAL();
  //DEBUG_PRINT("RUNNING");

  char c = GPS.read();
  //if(c) Serial.print(c);
  
  if(GPS.newNMEAreceived())
  {
    if(!GPS.parse(GPS.lastNMEA())) return;
  }

  //if(GPS.fix) writing();

  //DEBUG_STOP_SERIAL(); 
}

void writing()
{
  if(!setupdone) return;
  //noInterrupts();
  LP.CPU(F_CPU);  
  //delay(5000);
  //DEBUG_START_SERIAL();
  noInterrupts();
  BACK_FROM_SLEEP();
  //delay(5000);
  
  char buff[10];
  
  char c = GPS.read();
  //if(c) Serial.print(c);
  read_GPS: while(!GPS.newNMEAreceived())
  {
    char c = GPS.read();
    //DEBUG_PRINT("No NMEA Received");
    LP.Idle();
  };
  
  if(GPS.newNMEAreceived())
  {
    /*if(!GPS.parse(GPS.lastNMEA())) {
      DEBUG_PRINT("Unable to Parse GPS NMEA");
      //interrupts();
      return;
    }*/
    if(!GPS.parse(GPS.lastNMEA())) goto read_GPS;
  }

  if(!GPS.fix) 
  {
    String satstr = "No GPS Fix, Connected Satellites: ";
    sprintf(buff, "%u", GPS.satellites);
    satstr += buff;

    char *text4 = new char[satstr.length() + 1];
    strcpy(text4, satstr.c_str());

    DEBUG_PRINT(text4);

    delete [] text4;
    //DEBUG_STOP_SERIAL();
    //interrupts();
    return;
  }

  //DEBUG_PRINT("GPS START LOGGING");

  //open SD
  logFile = SD.open(filename, FILE_WRITE);

  //WRITE GPS LOG
  setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year);
  
  time_t eastern, utc;
  TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};  //UTC - 4 hours
  TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};   //UTC - 5 hours
  Timezone usEastern(usEDT, usEST);
  utc = now();    //current time from the Time Library
  eastern = usEastern.toLocal(utc);

  setTime(eastern);  
  
  String timestr = "Time: ";
  sprintf(buff, "%02d", hour());
  timestr += buff;
  timestr += ':';
  sprintf(buff, "%02d", minute());
  timestr += buff;
  timestr += ':';
  sprintf(buff, "%02d", second());
  timestr += buff;
  timestr += "\t";
  sprintf(buff, "%02d", month());
  timestr += buff;
  timestr += '/';
  sprintf(buff, "%02d", day());
  timestr += buff;
  timestr += '/';
  sprintf(buff, "%04d", year());
  timestr += buff;
  
  char *text1 = new char[timestr.length() + 1];
  strcpy(text1, timestr.c_str());

  DEBUG_PRINT(text1);
  logFile.println(text1);

  String gpsstr = "Location: ";
  sprintf(buff, "%f", GPS.latitude);
  gpsstr += "Latitude: ";
  gpsstr += buff[0];
  gpsstr += buff[1];
  gpsstr += "\260 ";
  for(int i=2;i<10;i++) gpsstr += buff[i];
  gpsstr += "\' ";
  gpsstr += GPS.lat;
  sprintf(buff, "%f", 0.0);
  sprintf(buff, "%f", GPS.longitude);
  gpsstr += ", Longitude: ";
  gpsstr += buff[0];
  gpsstr += buff[1];
  gpsstr += "\260 ";
  for(int i=2;i<10;i++) gpsstr += buff[i];
  gpsstr += "\' ";
  gpsstr += GPS.lon;
  sprintf(buff, "%f", GPS.altitude * 0.0328084);
  gpsstr += ", Altitude (ft): ";
  gpsstr += buff;

  char *text2 = new char[gpsstr.length() + 1];
  strcpy(text2, gpsstr.c_str());

  DEBUG_PRINT(text2);
  logFile.println(text2);

  //DEBUG_PRINT("GPS LOGGING WRITTEN TO SD");
  //DEBUG_PRINT(logptr);

  //Get new sensor data
  //DEBUG_PRINT("Getting Sensor Data");
  sensors_event_t event;
  bmp.getEvent(&event);
  //DEBUG_PRINT("Getting Event Data");
  float pressure = event.pressure;
  float temperature;
  bmp.getTemperature(&temperature);
  temperature = temperature*9/5+32;
  //DEBUG_PRINT("Getting Temp Data");
  //float seaLevelPressure = 1015.0;
  //float alt = bmp.pressureToAltitude(seaLevelPressure, event.pressure, temperature);
  //DEBUG_PRINT("Converting Data");

  //WRITE SENSOR LOG
  //char sensorptr[] = {"Pressure (hPa): " , event.pressure , ", Temperature (c): " , temperature , ", Altitude: " , alt};
  //uint8_t sensorsize = strlen(sensorptr);
  //if(sensorsize != logFile.write((uint8_t *)sensorptr, sensorsize));
  //if(strstr(sensorptr, "RMC")) logFile.flush();

  String sensorstr = "";
  sensorstr += "Pressure (hPa): "; 
  sprintf(buff, "%f", pressure);
  sensorstr += buff;
  sensorstr += ", Temperature (f): ";
  sprintf(buff, "%f", temperature);
  sensorstr += buff;
  sensorstr += ", Speed: ";
  sprintf(buff, "%f", GPS.speed);
  sensorstr += buff;

  char *text3 = new char[sensorstr.length() + 1];
  strcpy(text3, sensorstr.c_str());

  DEBUG_PRINT(text3);
  logFile.println(text3);

  //DEBUG_PRINT("SENSOR LOGGING WRITTEN TO SD");
  //DEBUG_PRINT(text);
  delete [] text1;
  delete [] text2;
  delete [] text3;

  //close SD
  logFile.close(); 

  DEBUG_PRINT("");
  //DEBUG_STOP_SERIAL();
  
  //interrupts();
}

void loop()
{
  interrupts();
  //LP.Sleep();
  BACK_FROM_SLEEP();
  running();
  delay(100);
  //SLEEP(5);
}










