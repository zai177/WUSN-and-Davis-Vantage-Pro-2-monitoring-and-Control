// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
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
#ifndef RFM69_h
#define RFM69_h
#include <Arduino.h>            // assumes Arduino IDE v1.0 or greater
#include <SPI.h>

#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define RF69_SPI_CS             SS // SS is the SPI slave select pin, for instance D10 on ATmega328

// INT0 on AVRs should be connected to RFM69's DIO0 (ex on ATmega328 it's D2, on ATmega644/1284 it's D2)
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
  #define RF69_IRQ_PIN          2
  #define RF69_IRQ_NUM          0
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  #define RF69_IRQ_PIN          2
  #define RF69_IRQ_NUM          2
#elif defined(__AVR_ATmega32U4__)
  #define RF69_IRQ_PIN          3
  #define RF69_IRQ_NUM          0
#elif defined(__arm__)//Use pin 10 or any pin you want
  #define RF69_IRQ_PIN          PA3
  #define RF69_IRQ_NUM          3
#else 
  #define RF69_IRQ_PIN          2
  #define RF69_IRQ_NUM          0  
#endif


// DavisRFM69 constants

//#ifndef DAVISRFM69_h
//#define DAVISRFM69_h

// Uncomment ONE AND ONLY ONE of the four #define's below.  This determines the
// frequency table the code will use.  Note that only the US (actually North
// America) and EU frequencies are defined at this time.  Australia and New
// Zealand are placeholders.  Note however that the frequencies for AU and NZ
// are not known at this time.
//#define DAVIS_FREQS_US
#define DAVIS_FREQS_EU
//#define DAVIS_FREQS_AU
//#define DAVIS_FREQS_NZ

#include <Davisdef.h>
#include <Arduino.h>            //assumes Arduino IDE v1.0 or greater

#define DAVIS_PACKET_LEN    10 // ISS has fixed packet length of 10 bytes, including CRC and retransmit CRC
#define RF69_SPI_CS         SS // SS is the SPI slave select pin, for instance D10 on ATmega328

// INT0 on AVRs should be connected to RFM69's DIO0 (ex on ATmega328 it's D2, on ATmega644/1284 it's D2)
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
  #define RF69_IRQ_PIN 2
  #define RF69_IRQ_NUM 0
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  #define RF69_IRQ_PIN 2
  #define RF69_IRQ_NUM 2
#elif defined(__AVR_ATmega32U4__)
  #define RF69_IRQ_PIN 3
  #define RF69_IRQ_NUM 0
#endif

#define CSMA_LIMIT          -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP     0 // XTAL OFF
#define RF69_MODE_STANDBY   1 // XTAL ON
#define RF69_MODE_SYNTH     2 // PLL ON
#define RF69_MODE_RX        3 // RX MODE
#define RF69_MODE_TX        4 // TX MODE

// available frequency bands
#define RF69_315MHZ         31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ         43
#define RF69_868MHZ         86
#define RF69_915MHZ         91

#define null                0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value

// These values aren't in the upstream version of RF69registers.h
#define REG_TESTAFC         0x71

#define RF_FDEVMSB_4800     0x00 // Used for Davis console reception
#define RF_FDEVLSB_4800     0x4e
#define RF_FDEVMSB_9900     0x00 // Used for Davis ISS transmission
#define RF_FDEVLSB_9900     0xa1

// Davis VP2 standalone station types, defined by kobuki
#define STYPE_ISS           0x0 // ISS
#define STYPE_TEMP_ONLY     0x1 // Temperature Only Station
#define STYPE_HUM_ONLY      0x2 // Humidity Only Station
#define STYPE_TEMP_HUM      0x3 // Temperature/Humidity Station
#define STYPE_WLESS_ANEMO   0x4 // Wireless Anemometer Station
#define STYPE_RAIN          0x5 // Rain Station
#define STYPE_LEAF          0x6 // Leaf Station
#define STYPE_SOIL          0x7 // Soil Station
#define STYPE_SOIL_LEAF     0x8 // Soil/Leaf Station
#define STYPE_SENSORLINK    0x9 // SensorLink Station (not supported for the VP2)
#define STYPE_OFF           0xA // No station - OFF

// Davis packet types, also defined by kobuki
#define VP2P_UV             0x4 // UV index
#define VP2P_RAINSECS       0x5 // seconds between rain bucket tips
#define VP2P_SOLAR          0x6 // solar irradiation
#define VP2P_TEMP           0x8 // outside temperature
#define VP2P_WINDGUST       0x9 // 10-minute wind gust
#define VP2P_HUMIDITY       0xA // outside humidity
#define VP2P_RAIN           0xE // rain bucket tips counter

/*end of davis constants*/


#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40


class RFM69 {
  public:
    static volatile uint8_t DATA[RF69_MAX_DATA_LEN]; // recv/xmit buf, including header & crc bytes
    static volatile uint8_t DATALEN;
    static volatile uint8_t SENDERID;
    static volatile uint8_t TARGETID; // should match _address
    static volatile uint8_t PAYLOADLEN;
    static volatile uint8_t ACK_REQUESTED;
    static volatile uint8_t ACK_RECEIVED; // should be polled immediately after sending a packet with ACK request
    static volatile int16_t RSSI; // most accurate RSSI during reception (closest to the reception)
    static volatile uint8_t _mode; // should be protected?

//Davis public variables

	static volatile uint8_t Davis_DATA[DAVIS_PACKET_LEN];  // recv/xmit buf, including header, CRC
    static volatile uint8_t Davis_mode; //should be protected?
    static volatile bool _packetReceived;
    static volatile uint8_t CHANNEL;
    static volatile int16_t Davis_RSSI;
    static volatile bool Davis_Switch;
    
    RFM69(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false, uint8_t interruptNum=RF69_IRQ_NUM, SPIClass &spi = SPI) {
      _slaveSelectPin = slaveSelectPin;
      _interruptPin = interruptPin;
      _interruptNum = interruptNum;
      _mode = RF69_MODE_STANDBY;
      _promiscuousMode = false;
      _powerLevel = 31;
      _isRFM69HW = isRFM69HW;
      _spi = spi;
      _packetReceived  = false;
      Davis_Switch = false;
    }

    bool initialize(uint8_t freqBand, uint8_t ID,uint8_t networkID=1, bool Switch = false);
    bool initialize(uint8_t nodeID);
    void setAddress(uint8_t addr);
    void setNetwork(uint8_t networkID);
    bool canSend();
    virtual void send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK=false);
    virtual bool sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries=2, uint8_t retryWaitTime=40); // 40ms roundtrip req for 61byte packets
    virtual bool receiveDone();
    bool ACKReceived(uint8_t fromNodeID);
    bool ACKRequested();
    virtual void sendACK(const void* buffer = "", uint8_t bufferSize=0);
    uint32_t getFrequency();
    void setFrequency(uint32_t freqHz);
    void encrypt(const char* key);
    void setCS(uint8_t newSPISlaveSelect);
    int16_t readRSSI(bool forceTrigger=false);
    void promiscuous(bool onOff=true);
    virtual void setHighPower(bool onOFF=true); // has to be called after initialize() for RFM69HW
    virtual void setPowerLevel(uint8_t level); // reduce/increase transmit power level
    void sleep();
    uint8_t readTemperature(uint8_t calFactor=0); // get CMOS temperature (8bit)
    void rcCalibration(); // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]

    // allow hacking registers by making these public
    uint8_t readReg(uint8_t addr);
    void writeReg(uint8_t addr, uint8_t val);
    void readAllRegs();
    void readAllRegsCompact();
// DavisRFM69 public functions 
   void Davis_setFrequency(uint32_t FRF);
   void Reset_Config();
   void Reset_Config_Davis();
   void Davis_hop();
   bool Davis_receiveDone();
   uint16_t Davis_crc16_ccitt(volatile uint8_t *buf, uint8_t len, uint16_t initCrc);
   void Davis_setChannel(uint32_t channel);
   uint8_t reverseBits(uint8_t b);
    // End of DavisRFM69 public functions
  protected:
    static void isr0();
    void virtual interruptHandler();
    virtual void interruptHook(uint8_t CTLbyte) {};
    static volatile bool _inISR;
    virtual void sendFrame(uint8_t toAddress, const void* buffer, uint8_t size, bool requestACK=false, bool sendACK=false);
    static void Davis_isr0();  //davis isr function
    static RFM69* selfPointer;
    void Davis_interruptHandler();   //davis function
    uint8_t _slaveSelectPin;
    uint8_t _interruptPin;
    uint8_t _interruptNum;
    uint8_t _address;
    bool _promiscuousMode;
    uint8_t _powerLevel;
    bool _isRFM69HW;
    SPIClass _spi = SPI;
#if defined (SPCR) && defined (SPSR)
    uint8_t _SPCR;
    uint8_t _SPSR;
#endif

    virtual void receiveBegin();
    void Davis_receiveBegin();
    
    virtual void setMode(uint8_t mode);
    virtual void setHighPowerRegs(bool onOff);
    virtual void select();
	void Davis_select();
	void Davis_unselect();
    virtual void unselect();
    inline void maybeInterrupts();
};


struct __attribute__((packed)) PacketStats
{
  uint16_t packetsReceived;
  uint16_t packetsMissed;
  uint16_t numResyncs;
  uint16_t receivedStreak;
  uint16_t crcErrors;
};

static PacketStats packetStats = {0, 0, 0, 0 ,0};


//#if  defined (DAVIS_FREQS_EU)
//#warning ** USING EUROPEAN FREQUENCY TABLE **
#define DAVIS_FREQ_TABLE_LENGTH 5
static const uint8_t __attribute__ ((progmem)) FRF[DAVIS_FREQ_TABLE_LENGTH][3] =
{
  {0xD9, 0x04, 0x45},
  {0xD9, 0x13, 0x04},
  {0xD9, 0x21, 0xC2},
  {0xD9, 0x0B, 0xA4},
  {0xD9, 0x1A, 0x63}
};
//#else
//#error ** ERROR DAVIS_FREQS MUST BE DEFINED AS ONE OF _US, _EU, _AZ, or NZ **
//#endif 
#endif


