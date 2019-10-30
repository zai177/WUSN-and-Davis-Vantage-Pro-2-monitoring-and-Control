 /******************************************************************************
 This file needs to be downloaded on a Molenet node with RTC and 5TM decagon sensor connected to it.
 */

#include "decagon_5tm.h"
#include "rv8523_clock_alarm.h"
#include <LowPower.h>
#include <RFM69.h>
#define BAUD_RATE 9600
#define INT_PIN 1



#define NODEID 1//unique for each node on same network33333
#define FREQUENCY RF69_433MHZ
#define ENCRYPTKEY "sampleEncryptKey" //key for encryption
#define NETWORKID 100 //the same on all nodes in the network
#define GATEWAYID 65 //ID of Gateway
#define RADIOPOWERLEVEL 31
#define SERIAL_BAUD 9600
#define INT_PIN 1
#define BASESTATIONID 69
// Create object of RTC class



RFM69 radio;
RV8523_RTC rtc;
bool alarmInt = false;
int minutes_alarm=1;

typedef struct {
  byte source_ID;
  byte node_id;
  byte seq_num;
  byte VWC_0;
  byte VWC_1;
  byte temperature_0;
  byte temperature_1;
  byte RSSI_0;
  byte RSSI_1;
  byte next_RSSI_0;
  byte next_RSSI_1;
}RSSIPacket;

RSSIPacket RP;

typedef struct {
  byte sink_ID;
  byte sample_rate;
  byte sleep_time;
  byte next_hop_id;
} ControlPacket;
ControlPacket CP;

typedef struct {
            byte start ;
}initial;
initial IP;

namespace Pin { enum Configuration {
      SENSOR_POWER  = 7,
      RX_5TM_Serial = 8,    // Software Serial
      TX_5TM_Serial = 11
};}

bool exciteSen = true;
int seq_num =0;
Decagon5TM sensor(Pin::SENSOR_POWER,Pin::RX_5TM_Serial, Pin::TX_5TM_Serial );
// Setup Function
void setup() {
  
   // pinMode(INT_PIN,INPUT);
  // Initialize Serial Communication
  Serial.begin(BAUD_RATE);
  while(!Serial){
    // Wait till Serial port is initialized
  }
  Serial.println("Real Time Clock Test");
radio.initialize(RF69_433MHZ,NODEID,100);
radio.Reset_Config(); 
radio.encrypt(ENCRYPTKEY);
Serial.print("Setup Done");
while(true)
{
  if(radio.receiveDone() && (int)radio.DATA[0] == 55)
  {
    Serial.println("initialized");
    break;}
}
  // Configure RTC TIME
  rtc.setTimeMode(RV8523_RTC::TM_24HOUR);
 rtc.setClockTime(50,00,14, 13,8,16);
  rtc.startRTC();
  rtc.batterySwitchOver(1);

//   node.config[StartupConfig::pos_CONTROL_2] = B01110000; // set only incremental alarm
//    node.config[StartupConfig::pos_ALARM_HOUR] = 0;
//    node.config[StartupConfig::pos_ALARM_MINUTE] = 2;
//    node.config[StartupConfig::pos_ALARM_DAY] = 0;
//    node.config[StartupConfig::pos_ALARM_WEEKDAY] = 0;


  //// Configure Alarm
//  rtc.beginAlarm(INT_PIN, B00000001 , 1, 0, 0, 0);
        
  rtc.resetCtrl();
  rtc.setAlarmTime(1,0,0,0);
  rtc.clearAllAlarm();  
  rtc.setAlarmType(RV8523_RTC::AT_MINUTE);
  rtc.activateAlarm();
  rtc.resetInterrupt();
  attachInterrupt(INT_PIN, alarmInterrupt, FALLING);    //create an interupt for the rtc alarm
  Serial.print("going for sleep");
  delay(200);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void alarmInterrupt()
{
  alarmInt = true;
}
byte x0,x1,y0,y1;
void loop() {
  
  if(alarmInt)
  {
    Serial.println("Alarm Gernerated, do some task");
    resetAlarm(10);
    unsigned long int timer  = millis();
    long unsigned int BroadCastPeriod=5000*NODEID;
    radio.initialize(RF69_433MHZ,NODEID,NETWORKID);
    radio.Reset_Config();
    radio.encrypt(ENCRYPTKEY);
    while(millis() - timer < 120000)
    {
       if(exciteSen)
    {
        sensor.exciteSensor();
        exciteSen = false;
    }
    if( sensor.newDecagon5tmDataAvailable() )
    {
//        Serial.println(sensor.rawTemperatureInt);
//        Serial.println(sensor.rawDielectricInt);
        x0= sensor.rawTemperatureInt>>8;
        x1 = sensor.rawTemperatureInt;
        y0 = sensor.rawDielectricInt>>8;
        y1 = sensor.rawTemperatureInt;
        exciteSen = true;
      
    }
      if(radio.receiveDone())
    {
      if(radio.DATALEN == sizeof(RP))
      {
        RP = *(RSSIPacket*)radio.DATA;
        Serial.print("RSSI Packet Received from :");
        Serial.print((String)RP.source_ID);
        Serial.print("\t");
        RP.node_id = NODEID;
        RP.RSSI_0 = radio.RSSI>>8;
        RP.RSSI_1 = radio.RSSI;
        Serial.print("Sleep Time");
        Serial.print((String)minutes_alarm);
        Serial.print("\n");
        delay((NODEID-1)*1000);
          if(radio.sendWithRetry(BASESTATIONID,(const void*)(&RP),sizeof(RP)))
          {
            Serial.println("RSSI Packet forwarded to Base Station");
          }
          else Serial.println("RSSI Packet could not be forwarded to Base Station");
      }
      else if(radio.DATALEN == sizeof(CP))
     {
      CP = *(ControlPacket*)radio.DATA;
      minutes_alarm = (int)CP.sleep_time;
      Serial.print("Sleep Time");
      Serial.print((String)minutes_alarm);
      Serial.print("\n");
        if(radio.ACKRequested())
        {
          radio.sendACK();
          Serial.println("ACK sent");
        }
        else Serial.println("Ack not requested");
      }
    }
    if(millis() - timer > BroadCastPeriod )
    {
//      for(int i=0;i<4;i++)
//      {
        RP.source_ID = NODEID;
        RP.seq_num = seq_num++;
        RP.temperature_0 = x0;
        RP.temperature_1 = x1;
        RP.VWC_0 = y0;
        RP.VWC_1 = y1;
        Serial.println("Broadcasting RSSI ");//- Source ID : "+int(RP.source_ID+" Sequence number : "+(int)RP.seq_num));
        radio.send(255,(const void*)(&RP),sizeof(RP),false);
        if(BroadCastPeriod<75000)BroadCastPeriod+=35000;
        else BroadCastPeriod = 130000;
     }
    }
    Serial.print("going for sleep");
    delay(200);
    radio.sleep();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
  
  Serial.println("");
  rtc.debugCurrentTime();
  rtc.debugAlarm();
   delay(1000);
}

void resetAlarm(int x)
{
    alarmInt = false;
    rtc.resetCtrl();
    rtc.setAlarmTime( ( (rtc.getCurrentMinutes()+x) % 60), 0, 0, 0);
    rtc.clearAllAlarm();
    rtc.setAlarmType(RV8523_RTC::AT_MINUTE);
    rtc.activateAlarm();
    rtc.resetInterrupt();
   
    rtc.batterySwitchOver(true);
        Serial.println("Reset Alarm");
}

