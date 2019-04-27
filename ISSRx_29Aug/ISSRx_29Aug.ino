// Sample RFM69 sender/CP sketch, with ACK and optional encryption, and Automatic Transmission Control
// Sends periodic messages of increasing length to gateway (id=1)
// It also looks for an onboard FLASH chip, if present
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
//#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)
#include<SerialCommand.h>
#include "SIM800.h"
#include"Wire.h"
//#include<DavisRFM69.h>
//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************

#define APN "internet"
#define CPID        69    //must be unique for each CP on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID     100  //the same on all CPs that talk to each other (range up to 255)
#define GATEWAYID     5
//Match frequency to the hardware version of the SimpleRadio on your Moteino (uncomment one):
#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all CPs!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define PACKET_INTERVAL 2555
#define WUSNSIZE 4

RFM69 radio;//
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote CPs that have ATC enabled to
//dial their power down to only the required level (ATC_RSSI)
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
//*********************************************************************************************

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#define SERIAL_BAUD   9600

int TRANSMITPERIOD = 200; //transmit a packet to gateway so often (in ms)

CGPRS_SIM800 gprs;
typedef struct {
  byte sink_ID;
  byte sample_rate;
  byte sleep_time;
  byte next_hop_id;
} CommandPacket;

CommandPacket CP;

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

typedef struct {
            byte start ;
}initial;
initial IP;

RSSIPacket RP;
CommandPacket CPs[WUSNSIZE];
//
//SerialCommand sCmd;
//char buff[20];
//byte sendSize=0;
//boolean requestACK = false, 
boolean Switch = false;
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)





void setup() {
  
  
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize(FREQUENCY,CPID,NETWORKID);
  //radio.Davis_setChannel(0);
  radio.Reset_Config();
  radio.encrypt(ENCRYPTKEY);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor CPs that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable CPs that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
//radio.enableAutoPower(ATC_RSSI);
radio.setPowerLevel(31);
  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  //while(!Serial);
  //Serial.println(buff);

  if (flash.initialize())
  {
    //Serial.print("SPI Flash Init OK ... UniqueID (MAC): ");
    flash.readUniqueId();
    for (byte i=0;i<8;i++)
    {
      Serial.print(flash.UNIQUEID[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
  else
    //Serial.println("SPI Flash MEM not found (is chip soldered?)...");
  radio.promiscuous(true);
#ifdef ENABLE_ATC
  //Serial.println("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

for (;;) 
  {
    //Serial.print("Resetting GSM Shield...");
    while (!gprs.init()) 
    {
      Serial.write('#');
    }
    //Serial.println("OK");
    
    //Serial.print("Setting up network...");
    byte ret = gprs.setup(APN);
    if (ret == 0)
    {
      break;
    }
    Serial.print("Error code:");
    Serial.println(ret);
    Serial.println(gprs.buffer);
}
  
  delay(3000);
  if (gprs.getOperatorName()) 
  {
    Serial.print("Operator:");
    Serial.println(gprs.buffer);
  }
  int ret = gprs.getSignalQuality();
  if (ret) 
  {
     Serial.print("Signal:");
     Serial.print(ret);
     Serial.println("dB");
  }
  String com  = postData("","134.102.188.200/RSSIlog.php");
  Serial.println(com);
  com = com.substring(com.indexOf('{')+1,com.indexOf('}'));
          String A[WUSNSIZE*4];
          int count =0;
          int firstIndex =-1;
          for (int i = 0 ;i<com.length() ;i++)
               if(com.charAt(i)==':' )
               {
               A[count] = com.substring(firstIndex+1,i);
               count++;
               firstIndex = i;
               }
          for (int i = 0 ;i<WUSNSIZE ;i++)
          {
            CP.sink_ID = byte(A[4*i].toInt());
            CP.sample_rate = byte(A[4*i+1].toInt());
            CP.sleep_time = byte(A[4*i+2].toInt());
            CP.next_hop_id = byte(A[4*i+3].toInt());
            CPs[i] = CP;
          }

IP.start = 55;
Serial.print("Sending start packet");
radio.sendWithRetry(255,(const void*)(&IP),sizeof(IP),20);
Serial.println("Setup Complete");
}

void blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}



void loop()
{
       //Serial.println("moving to weather station");
//     radio.encrypt(0);
//     radio.Reset_Config_Davis();   //RFM69 mode change to communicate with Weather Sataion
//     radio.Davis_setChannel(0);        // Set radio to first of 5 channels to receive from weather Station
    // DavisReceive();               // radio receives 7 correct packets from weather station, and sends them to web server
     //radio.Reset_Config();       // rfm69 MODE changed back to communicate with WUSN
     //radio.encrypt(ENCRYPTKEY);
       //Serial.println("started waiting for packet");
     RSSIPacketHandle();
}
void RSSIPacketHandle()
{
  int x = (WUSNSIZE*WUSNSIZE-WUSNSIZE)*3;
     RSSIPacket RPs[x];
     int n=0;
     String com;
     unsigned long int timer = millis();
     if(radio.receiveDone()){ 
      Serial.println("Packet reception started");
        if(radio.DATALEN == sizeof(RP) && radio.DATA[0] != radio.DATA[1] && radio.DATA[1]!=0)
          {if(radio.ACKRequested())radio.sendACK();
          Serial.println("Packet received");
          RPs[n] = *(RSSIPacket*)radio.DATA;
          RPs[n].next_RSSI_0 = radio.RSSI>>8;
          RPs[n].next_RSSI_1 = radio.RSSI;
          n++;}
     while(millis()-timer < 110000 && n<x)
     {
      if(radio.receiveDone())
      { 
         if(radio.DATALEN == sizeof(RP) && radio.DATA[0] != radio.DATA[1]&& radio.DATA[1]!=0 && (radio.DATA[0] != RPs[n-1].source_ID || radio.DATA[1] != RPs[n-1].node_id ||radio.DATA[2] != RPs[n-1].seq_num ))
        { 
          if(radio.ACKRequested())radio.sendACK(); Serial.println("Packet received");
          RPs[n] = *(RSSIPacket*)radio.DATA;
          RPs[n].next_RSSI_0 = radio.RSSI>>8;
          RPs[n].next_RSSI_1 = radio.RSSI;
          n++;
      }
     }
     }
     for(int i=0;i<WUSNSIZE;i++)
     {
      CP = CPs[i];
          if(radio.sendWithRetry(CP.sink_ID,(const void*)(&CP),sizeof(CP))) Serial.print("Command sent"); else Serial.print("Command not sent");
     }
     for(int j = 0;j<n;j+=5){String Data = "params=";
          for(int i=0;i<5 && (j+i)<n;i++)
             Data += String(RPs[j+i].source_ID)+ "," + String(RPs[j+i].node_id)+ "," + String(RPs[j+i].seq_num)+ "," + String((RPs[j+i].VWC_0 << 8) | (RPs[j+i].VWC_1))+ "," + String((RPs[j+i].temperature_0 << 8) | (RPs[j+i].temperature_1))+ "," + String((RPs[j+i].RSSI_0 << 8) | (RPs[j+i].RSSI_1))+","+String((RPs[j+i].next_RSSI_0 << 8) | (RPs[j+i].next_RSSI_1))+",";
             Serial.print(Data);
             if(Data=="params=") break;
             com  = postData(Data.c_str(),"134.102.188.200/RSSIlog.php");
            Serial.println(com);
           }
          com = com.substring(com.indexOf('{')+1,com.indexOf('}'));
          String A[WUSNSIZE*4];
          int count =0;
          int firstIndex =-1;
          for (int i = 0 ;i<com.length() ;i++)
               if(com.charAt(i)==':' )
               {
               A[count] = com.substring(firstIndex+1,i);
               count++;
               firstIndex = i;
               }
          for (int i = 0 ;i<WUSNSIZE ;i++)
          {
            CP.sink_ID = byte(A[4*i].toInt());
            CP.sample_rate = byte(A[4*i+1].toInt());
            CP.sleep_time = byte(A[4*i+2].toInt());
            CP.next_hop_id = byte(A[4*i+3].toInt());
            CPs[i] = CP;
          }
//          if(radio.sendWithRetry(CP.sink_ID,(const void*)(&CP),sizeof(CP)),10)
//          Serial.println("Command Sent successfully");
//          else 
//          Serial.println("Command Ack not received");
}//else Serial.println("nodes are sleeping");
}
void DavisReceive()
{
unsigned long lastRxTime = millis();
byte hopCount = 0;
int counter  =0;
  String h = "0",t= "0",ws= "0",wg= "0",r= "0",d= "0",weatherData= "0";
  while(true){
         if (radio.Davis_receiveDone()) {
    packetStats.packetsReceived++;
    unsigned int crc = radio.Davis_crc16_ccitt(radio.Davis_DATA, 6,0);
    if ((crc == (word(radio.Davis_DATA[6], radio.Davis_DATA[7]))) && (crc != 0)) {
      packetStats.receivedStreak++;
      hopCount = 1;
      blink(LED,3);
    } else {
      packetStats.crcErrors++;
      packetStats.receivedStreak = 0;
    }

if (radio.Davis_DATA[DAVIS_PACKET_LEN-1]==255 && radio.Davis_DATA[DAVIS_PACKET_LEN-2]==255)
{
//    Switch = false;
    Serial.print(radio.CHANNEL);
    Serial.print(F(" - Data: "));
    ws = String(radio.Davis_DATA[1]);
    d = String(radio.Davis_DATA[2]*360/255);
    for (byte i = 0; i < DAVIS_PACKET_LEN; i++) {
      //Serial.print(radio.Davis_DATA[i], HEX);
      //Serial.print(F(" "));
  }
  //Serial.print(F("  RSSI: "));
  Serial.println(radio.RSSI);
  //Serial.print("Wind Speed : ");
  //Serial.print(radio.Davis_DATA[1]);
  //Serial.print("mph  ");
  //Serial.print("Wind Direction : ");
  //Serial.print(radio.Davis_DATA[2]*360/255);
  //Serial.print("  ");
  switch (radio.Davis_DATA[0] >> 4) {
    case VP2P_TEMP:
//      //Serial.print("Outside Temprature : ");
      t = String((float)(word(radio.Davis_DATA[3], radio.Davis_DATA[4]) >> 4)/10);
//      //Serial.print(t);
//      //Serial.print("degree C \n");
      break;
    case VP2P_HUMIDITY:
//      //Serial.print("Outside Humidity : ");
      h = String((float)(word((radio.Davis_DATA[4] >> 4), radio.Davis_DATA[3])) / 10.0);
//      //Serial.print(h);
//      //Serial.print("\n");
      break;
    case VP2P_UV:
//      //Serial.print("UV Index : ");
    //  UV_Index = ((float)((word((radio.Davis_DATA[3] << 8), radio.Davis_DATA[4]))>>6) / 50.0);
//      //Serial.print(UV_Index);
//      //Serial.print("\n");
      break;
    case VP2P_RAINSECS:
//      //Serial.print("Seconds between tips : ");
      //Rain_secs = radio.Davis_DATA[3];
//      //Serial.print(Rain_secs);
//      //Serial.print("{225 means no rain} \n");
      break;
    case VP2P_SOLAR:
//      //Serial.print("Solar Radiation : ");
   //   Solar = ((float)((word((radio.Davis_DATA[3] << 8), radio.Davis_DATA[4]))>>6) * 1.757936);
//      //Serial.print(Solar);
//      //Serial.print("\n");
      break;
    case VP2P_WINDGUST:
//      //Serial.print("Last 10 Min wind gust : ");
      wg = String(radio.Davis_DATA[3]);
//      //Serial.print(wg);
//      //Serial.print("\n");
      break;
    case VP2P_RAIN:
//      //Serial.print("Rain Bucket Tips : ");
      r = String(radio.Davis_DATA[3]);
//      //Serial.print(r);
//      //Serial.print("\n");
      break;
  }
  if(++counter ==7)
  {
    weatherData = "params=" + t + "," +  h + "," + ws + "," + wg + "," + r + "," + d+",";
     postData(weatherData.c_str(),"134.102.188.200/deploy2.php");
//    Serial.println(com);
    counter = 0;
    break;
  }
}
  // Whether CRC is right or not, we count that as reception and hop.
  lastRxTime = millis();
  radio.Davis_hop();
  }

//   If a packet was not received at the expected time, hop the radio anyway
//   in an attempt to keep up.  Give up after 25 failed attempts.  Keep track
//   of packet stats as we go.  I consider a consecutive string of missed
//   packets to be a single resync.  Thx to Kobuki for this algorithm.
  if ((hopCount > 0) && ((millis() - lastRxTime) > (hopCount * PACKET_INTERVAL + 200))) {
    packetStats.packetsMissed++;
if (hopCount == 1) packetStats.numResyncs++;
    if (++hopCount > 25) hopCount = 0;
    radio.Davis_hop();
  }
}
}



String postData(const char* data4,const char* url)
{ 
  //static const char* url = "134.102.188.200/deploy2.php"; 
  unsigned long    time6;
  unsigned long    diff3;
unsigned long  time5 = millis();
  for (;;) 
    {
    if (gprs.httpInit()) 
    {
      Serial.print("HTTP Connection");
      Serial.println(gprs.buffer);
      break;
    }
    Serial.println(gprs.buffer);
    gprs.httpUninit();
  time6 = millis();
    Serial.print("time 1 ");
//    Serial.println(time1);
    Serial.print("time 2 ");
//    Serial.println(time2);
    Serial.print("Time for http Connection: ");
unsigned long    diff3 = time6 - time5;
    //Serial.println(diff1);
    if(diff3 >= 60000)
    {
      Serial.print("Time for http Connection: ");
      Serial.println(diff3);
      time5 = millis();
      Serial.println("Connection Lost");
      for (;;) 
      {
        Serial.print("Resetting GSM Shield...");
        while (!gprs.init()) 
        {
          Serial.write('.');
        }
        Serial.println("OK");
    
        Serial.print("Setting up network...");
        byte ret = gprs.setup(APN);
        if (ret == 0)
        {
          break;
        }
        Serial.print("Error code:");
        Serial.println(ret);
        Serial.println(gprs.buffer);
       } 
      delay(3000);
      if (gprs.getOperatorName()) 
      {
        Serial.print("Operator:");
        Serial.println(gprs.buffer);
       }
      int ret = gprs.getSignalQuality();
      if (ret) 
      {
        Serial.print("Signal:");
        Serial.print(ret);
        Serial.println("dB");
      }
    }
    delay(1000);
  }
  delay(3000);
  gprs.httpConnect(url);
time5 = millis();
  for (;;)
  {
    byte check = gprs.httpIsConnected();
    if(check == 0)
    {
      
       unsigned long time8 = millis();
       
         unsigned long diff4 = time8 - time5;
       
       if(diff4 >= 60000)
       {
         Serial.print("Time for link Connection : ");
         Serial.println(diff4);
         time5 = millis();
         check = 2;
         Serial.println("Connection Lost");
       } 
    }
    if(check == 1)
    {
      Serial.println("Cool");
      Serial.println(gprs.buffer);
      break;
    }
    if(check == 2)
    {
      Serial.println("Not cool");
      Serial.println(gprs.buffer);
      for (;;) 
      {
        Serial.print("Resetting GSM Shield...");
        while (!gprs.init()) 
        {
          Serial.write('.');
        }
        Serial.println("OK");
    
        Serial.print("Setting up network...");
        byte ret = gprs.setup(APN);
        if (ret == 0)
        {
          break;
        }
        Serial.print("Error code:");
        Serial.println(ret);
        Serial.println(gprs.buffer);
      } 
      delay(3000);
      if (gprs.getOperatorName()) 
      {
        Serial.print("Operator:");
        Serial.println(gprs.buffer);
      }
      int ret = gprs.getSignalQuality();
      if (ret) 
      {
        Serial.print("Signal:");
        Serial.print(ret);
        Serial.println("dB");
      }
      for (;;) 
      {
        if (gprs.httpInit()) 
        {
          Serial.print("HTTP Connection");
          Serial.println(gprs.buffer);
          break;
        }
        Serial.println(gprs.buffer);
        gprs.httpUninit();
        delay(1000);
      }
      delay(3000);
      gprs.httpConnect(url);
    }
  }
  gprs.sendCommand("AT+HTTPPARA=CONTENT,application/x-www-form-urlencoded");
  delay(100);
  Serial.println(gprs.buffer);
  if(data4!=""){
      gprs.sendCommand("AT+HTTPDATA=192,10000");
      delay(100);
      Serial.println(gprs.buffer);
      gprs.sendCommand(data4);
      delay(10000);
      gprs.sendCommand("AT+HTTPACTION=1");
      delay(5000);
  }else
  {
    gprs.sendCommand("AT+HTTPACTION=0");
    delay(1500);
    Serial.println(gprs.buffer);
  }
  Serial.println(gprs.buffer);
  gprs.sendCommand("AT+HTTPREAD");
  delay(10000);
  String com = gprs.buffer;
  gprs.sendCommand("AT+HTTPTERM");
  delay(500);
  Serial.println(gprs.buffer);
  Serial.println("Done"); 
  return com;
}
