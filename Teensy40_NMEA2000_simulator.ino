
#define NMEA2000_TEENSYX_CAN_BUS tNMEA2000_Teensyx::CAN1
#include <Arduino.h>

#include "NMEA2000_CAN.h"  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include "N2kMessages.h"

int led = 13;   // Yellow LED on Teensy
static const int LED_red = 9;
static const int LED_green = 8;
uint32_t LED_green_ontime = 0;
uint32_t LED_red_ontime = 0;
uint32_t led_ontime = 0;

static const int SW1 = 6;
static const int SW2 = 7;

static const int P1 = 0;    // Wind speed
static const int P2 = 1;    // Wind direction
static const int P3 = 2;    // Water temperature
static const int P4 = 3;    // Engine speed
static const int P5 = 6;    // COG
static const int P6 = 7;    // SOG

// List here messages your device will transmit.
const unsigned long TemperatureMonitorTransmitMessages[] PROGMEM={130310L,130311L,130312L,0};
const unsigned long BatteryMonitorTransmitMessages[] PROGMEM={127506L,127508L,127513L,0};
const unsigned long EngineTransmitMessages[] PROGMEM={127488L,0};
const unsigned long TransmitWindMessages[] PROGMEM={130306L,0};
const unsigned long TransmitCOGSOGMessages[] PROGMEM={129026L,0};

#define DEV_TEMP 0
//#define DEV_BAT  1  //Battery voltage
#define DEV_HEAD 1
#define DEV_ENG  2  //Engine RPM
//#define DEV_RUD  3  //Rudder angle
#define DEV_WIND 3  //Wind info 

void setup() {
  pinMode(led,OUTPUT);
  pinMode(LED_red,OUTPUT);
  pinMode(LED_green,OUTPUT);
  pinMode(SW1,INPUT_PULLUP);
  pinMode(SW2,INPUT_PULLUP);
  digitalWrite(LED_green,HIGH);
  delay(1000);
  digitalWrite(LED_green,LOW);
  digitalWrite(LED_red,HIGH);

  
  Serial.println("****** Teensy 4.0 NMEA2000 simulator skpang.co.uk 2023");

  NMEA2000.SetDeviceCount(4); // Enable multi device support for 2 devices
  // Set Product information for temperature monitor
  NMEA2000.SetProductInformation("112233", // Manufacturer's Model serial code. 
                                 100, // Manufacturer's product code
                                 "Simple temperature monitor",  // Manufacturer's Model ID
                                 "1.0.0.2 (2017-06-13)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2017-06-13)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_TEMP
                                 );
  /*
  // Set Product information for battery monitor
  NMEA2000.SetProductInformation("112234", // Manufacturer's Model serial code. 
                                 100, // Manufacturer's product code
                                 "Simple battery monitor",  // Manufacturer's Model ID
                                 "1.0.0.2 (2017-06-13)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2017-06-13)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_BAT
                                 );
                                 */
  NMEA2000.SetProductInformation("112238", // Manufacturer's Model serial code. 
                                 100, // Manufacturer's product code
                                 "Simple heading monitor",  // Manufacturer's Model ID
                                 "1.0.0.2 (2017-06-13)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2017-06-13)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_HEAD
                                 );
  // Set Product information for ENGINE monitor
  NMEA2000.SetProductInformation("112235", // Manufacturer's Model serial code. 
                                 101, // Manufacturer's product code
                                 "Simple engine monitor",  // Manufacturer's Model ID
                                 "1.0.0.2 (2017-06-13)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2017-06-13)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_ENG
                                 );    

  // Set Product information for WIND monitor
  NMEA2000.SetProductInformation("00000002", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Simple wind monitor",  // Manufacturer's Model ID
                                 "1.1.0.22 (2016-12-31)",  // Manufacturer's Software version code
                                 "1.1.0.0 (2016-12-31)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_WIND
                                 );  



// Set device information for temperature monitor
  NMEA2000.SetDeviceInformation(112233, // Unique number. Use e.g. Serial number.
                                130, // Device function=Temperature. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75, // Device class=Sensor Communication Interface. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                                4, // Marine
                                DEV_TEMP
                               );
  /*
  // Set device information for battery monitor
  NMEA2000.SetDeviceInformation(112234,  // Unique number. Use e.g. Serial number.
                                170,    // Device function=Battery. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                35,     // Device class=Electrical Generation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040,    // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                                4, // Marine
                                DEV_BAT
                               );
                               */
// Set device information for heading monitor
  NMEA2000.SetDeviceInformation(112238,  // Unique number. Use e.g. Serial number.
                                170,    // Device function=Battery. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                35,     // Device class=Electrical Generation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040,    // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                                4, // Marine
                                DEV_HEAD
                               );
  // Set device information for engine monitor
  NMEA2000.SetDeviceInformation(112235,  // Unique number. Use e.g. Serial number.
                                170,    // Device function=Battery. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                35,     // Device class=Electrical Generation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040,    // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                                4, // Marine
                                DEV_ENG
                               );
  // Set device information for WIND monitor
  NMEA2000.SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                130, // Device function=Atmospheric. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                85, // Device class=External Environment. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                                4, // Marine
                                DEV_WIND
                               );
  // Uncomment 2 rows below to see, what device will send to bus. Use e.g. OpenSkipper or Actisense NMEA Reader                           
  NMEA2000.SetForwardStream(&Serial);
  // If you want to use simple ascii monitor like Arduino Serial Monitor, uncomment next line
 //NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  //NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,22);
  NMEA2000.SetMode(tNMEA2000::N2km_SendOnly,22);
  //NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  //NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  
  // Here we tell, which PGNs we transmit from temperature monitor
  NMEA2000.ExtendTransmitMessages(TemperatureMonitorTransmitMessages,DEV_TEMP);
  // Here we tell, which PGNs we transmit from battery monitor
  //NMEA2000.ExtendTransmitMessages(BatteryMonitorTransmitMessages,DEV_BAT);
  NMEA2000.ExtendTransmitMessages(EngineTransmitMessages,DEV_ENG);
  NMEA2000.ExtendTransmitMessages(TransmitCOGSOGMessages,DEV_HEAD);
  NMEA2000.ExtendTransmitMessages(TransmitWindMessages,DEV_WIND);

  NMEA2000.Open();

  Serial.println("Starting...");
  digitalWrite(LED_red,LOW);
}


void loop() {

 
 
  SendN2kTemperature();
  //SendN2kBattery();
  SendN2kEngine();
  SendN2kWind();
  SendN2kCOGSOG();
  NMEA2000.ParseMessages();

  update_led();

}
void update_led()
{

  if((millis() - LED_red_ontime) > 20 )
  {
      digitalWrite(LED_red,LOW);

  }

  if((millis() - led_ontime) > 1000 )
  {
      digitalToggle(led);
      led_ontime = millis();
  }
  

}
void blink_led(uint8_t led)
{
  if(led == LED_red)
  {
    digitalWrite(LED_red,HIGH);
    LED_red_ontime = millis();
  }
}

double ReadCabinTemp() {
  return CToKelvin(0); // Read here the true temperature e.g. from analog input
}

double ReadWaterTemp() {
  double temp;
  
  blink_led(LED_red);
  temp  =(double)(map((1023-analogRead(P3)), 0, 1023, 273, 700));

  return temp;// CToKelvin(0); // Read here the true temperature e.g. from analog input
}

#define EngineUpdatePeriod 100

void SendN2kEngine(){
  static unsigned long Updated=millis()+50;
  tN2kMsg N2kMsg;

  if ( Updated+EngineUpdatePeriod<millis() ) {
      blink_led(LED_red);
      Updated=millis();
  
      SetN2kEngineParamRapid(N2kMsg, 0,map((1023-analogRead(P4)),0,1023,0,16384), N2kDoubleNA, N2kInt8NA);
      NMEA2000.SendMsg(N2kMsg,DEV_ENG);
  }
}
double ReadCOG() {
  double COG;
  blink_led(LED_red);

  COG = (double)map((1023-analogRead(P5)), 0, 1023, 0, 360);
  return DegToRad(COG); // Read here the measured wind angle e.g. from analog input
}
double ReadSOG() {
  double SOG;
  blink_led(LED_red);

  SOG = (double)map((1023-analogRead(P6)), 0, 1023, 0, 32000);
  return DegToRad(SOG); // Read here the measured wind angle e.g. from analog input
}
#define HeadUpdatePeriod 100

void SendN2kCOGSOG(){
  static unsigned long Updated=millis()+60;
  tN2kMsg N2kMsg;

  if ( Updated+HeadUpdatePeriod<millis() ) {
    Updated=millis();
    
    SetN2kCOGSOGRapid(N2kMsg, 1,0,ReadCOG(),ReadSOG());
    NMEA2000.SendMsg(N2kMsg,DEV_HEAD);
  }

}

#define TempUpdatePeriod 100
void SendN2kTemperature() {
     
  static unsigned long Updated=millis()+60;
  tN2kMsg N2kMsg;

  if ( Updated+TempUpdatePeriod<millis() ) {
    Updated=millis();
    //SetN2kTemperature(N2kMsg, 1, 1, N2kts_MainCabinTemperature, ReadCabinTemp());
    //NMEA2000.SendMsg(N2kMsg,DEV_TEMP);
    //SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_MainCabinTemperature, 400);
    //NMEA2000.SendMsg(N2kMsg,DEV_TEMP);
    SetN2kOutsideEnvironmentalParameters(N2kMsg, 1, ReadWaterTemp());
    NMEA2000.SendMsg(N2kMsg,DEV_TEMP);
  }
}

/*
#define BatUpdatePeriod 100
void SendN2kBattery() {
  double p4;
  static unsigned long Updated=millis()+70;
  tN2kMsg N2kMsg;

  if ( Updated+BatUpdatePeriod<millis() ) {
    Updated=millis();
    p4 = ((double)analogRead(P1))/1000;
    Serial.println(p4);
    SetN2kDCBatStatus(N2kMsg,1,p4,6.2,35.12,1);
    NMEA2000.SendMsg(N2kMsg,DEV_BAT);
    //SetN2kDCStatus(N2kMsg,1,1,N2kDCt_Battery,56,92,38500,0.012, AhToCoulomb(420));
    //NMEA2000.SendMsg(N2kMsg,DEV_BAT);
    //SetN2kBatConf(N2kMsg,1,N2kDCbt_Gel,N2kDCES_Yes,N2kDCbnv_12v,N2kDCbc_LeadAcid,AhToCoulomb(420),53,1.251,75);
    //NMEA2000.SendMsg(N2kMsg,DEV_BAT);
  }
}
*/


double ReadWindAngle() {
  double angle;
  blink_led(LED_red);

  angle = (double)map((1023-analogRead(P2)), 0, 1023, 0, 360);
  return DegToRad(angle); // Read here the measured wind angle e.g. from analog input
}

double ReadWindSpeed() {
  double wind;
  blink_led(LED_red);
  wind =(double)(map((1023-analogRead(P1)), 0, 1023, 0, 0xffff)/100);

  return wind; // Read here the wind speed e.g. from analog input
}
#define WindUpdatePeriod 100

void SendN2kWind() {
  static unsigned long WindUpdated=millis()+80;
  tN2kMsg N2kMsg;

  if ( WindUpdated+WindUpdatePeriod<millis() ) {
    SetN2kWindSpeed(N2kMsg, 1, ReadWindSpeed(), ReadWindAngle(),N2kWind_Apprent);
    WindUpdated=millis();
    NMEA2000.SendMsg(N2kMsg);
  }
}
