#include <Arduino.h>

#include <SPI.h>

#include <RadioLib.h>                              // Radio Library to control RF chip

#include "ogn.h"                                   // OGN packet format
#include "ldpc.h"                                  // Low Density Parity Code encoder and decoder
#include "manchester.h"                            // Manchester encoding/ecoding table

#include "rfm.h"

#ifdef HAS_SX1276
static SX1276 Radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, RADIOLIB_NC);  // create sx1276 RF module
#endif

#ifdef HAS_SX1262
static SX1262 Radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY);    // create sx1262 RF module
#endif

uint64_t UniqueID = 0; // unique CPU ID to create a unique radio address

// OGNv1 SYNC:       0x0AF3656C encoded in Manchester
static const uint8_t OGN1_SYNC[8] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A };

// PilotAware SYNC
static const uint8_t PAW_SYNC [8] = { 0xB4, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x18, 0x71 };

static void SetupOGN(int Channel=0, float TxPower=10)         // Radio setup for given channel #0 or #1
{ Radio.setFrequency(868.2+0.2*Channel);                      // [MHz]  transmission/reception frequency
  Radio.setBitRate(100.0);                                    // [kpbs] 100kbps bit rate but we transmit Manchester encoded thus effectively 50 kbps
  Radio.setFrequencyDeviation(50.0);                          // [kHz]  +/-50kHz deviation
  Radio.setRxBandwidth(250.0);                                // [kHz]  250kHz bandwidth
  Radio.setOutputPower(TxPower);                              // [dBm]  transmission power
  Radio.setCurrentLimit(100);                                 // [mA]   depends on transmission power
  Radio.setDataShaping(RADIOLIB_SHAPING_0_5);                 // [BT]   FSK modulation shaping 
  Radio.setPreambleLength(8);                                 // [bits] minimal preamble
  Radio.setCRC(0, 0);                                         // disable CRC: we do it ourselves
  Radio.fixedPacketLengthMode(2*OGN_TxPacket<OGN1_Packet>::Bytes); // [bytes] Fixed packet size mode
  
  Radio.disableAddressFiltering();
  Radio.setSyncWord((uint8_t *)OGN1_SYNC, 8); }               // SYNC sequence: 8 bytes which is equivalent to 4 bytes

static OGN_TxPacket<OGN1_Packet> TxPacket;                           // encoded OGN packet, to be transmitted
static uint8_t ManchPacket[2*OGN_TxPacket<OGN1_Packet>::Bytes];      // Manchester encoded OGN Packet, ready to be written to the RF chip
static uint8_t RxBuffer[2*OGN_TxPacket<OGN1_Packet>::Bytes];         // received raw OGN packet (still Manchester encoded)
static RFM_FSK_RxPktData         RxPktData;                          // received Manchester decoded data and error bits
static LDPC_Decoder              Decoder;                            // decoder and error corrector for the OGN Gallager/LDPC code
static OGN_RxPacket<OGN1_Packet> RxPacket;                           // received, decoded and corrected packet

static void EncodePacket(OGN_TxPacket<OGN1_Packet> &TxPacket)  // encode some fixed (demo) position of the aircraft
{
  TxPacket.Packet.HeaderWord = 0;                  // start with clean header
  TxPacket.Packet.Header.Address=UniqueID&0xFFFFFF; // set address: 24-bit number
  TxPacket.Packet.Header.AddrType = 0;             // set address-type: 0 for random address-type
  TxPacket.Packet.calcAddrParity();                // set the address-parity bit

  TxPacket.Packet.Position.AcftType = 0xD;         // set the aircraft-type: drone
  TxPacket.Packet.Position.Time = 0x3F;            // [sec] time corresponding to the position or 0x3F if position older than 60sec
  TxPacket.Packet.Position.FixQuality = 1;         // 0 = none, 1 = GPS, 2 = Differential GPS (can be WAAS)
  TxPacket.Packet.Position.FixMode    = 1;         // 0 = 2-D, 1 = 3-D
  TxPacket.Packet.EncodeDOP(15-10);                // [0.1] setDilution-Of-Precision to 1.5
  TxPacket.Packet.EncodeLatitude(5012345*6);       // set Latitude to 50.12345 deg north
  TxPacket.Packet.EncodeLongitude(2012345*6);      // set Longitude to 20.12345 deg east
  TxPacket.Packet.EncodeAltitude(200);             // [m] set GPS altitude to 1200 m AMSL
  TxPacket.Packet.EncodeStdAltitude(250);          // [m] set standard pressure altitude to 1250m
  // TxPacket.Packet.clrBaro();                    // or signal the pressure altitude is not available
  TxPacket.Packet.EncodeSpeed(355);                // [0.1m/s] set the ground speed 35.5 m/s
  TxPacket.Packet.EncodeHeading(1305);             // [0.1deg] set track-over-ground to 130.5 deg
  TxPacket.Packet.EncodeClimbRate(+45);            // [0.1m/s] set climb rate to +4.5 m/s
  // TxPacket.Packet.clrClimbRate();               // or signal climb rate is not available
  TxPacket.Packet.EncodeTurnRate(+15);             // [0.1deg/s] set the ground turn rate to 1.5deg/sec
  // TxPacket.Packet.clrTurnRate();                // or signal the turn rate is not available
  // TxPacket.Packet.Print();                         // print the encoded information
  //                                                  // Note: we decode before whitenning the data part

  TxPacket.Packet.Whiten();                        // whiten the data portion of the packet
  TxPacket.calcFEC();                              // calculate the redundancy part of the packet
}

static int Manchester(uint8_t *Out, const OGN_TxPacket<OGN1_Packet> &TxPacket) // Encode as Manchester
{ const uint8_t *Inp = TxPacket.Byte();
  int Len=0;
  for(int Idx=0; Idx<TxPacket.Bytes; Idx++)        // loop over bytes and encode usinglookup table
  { uint8_t Byte=Inp[Idx];
    Out[Len++]=ManchesterEncode[Byte>>4];
    Out[Len++]=ManchesterEncode[Byte&0x0F]; }
  return Len; }                                    // returns number of bytes in the encoded packet

void setup()                                       // Arduino convention: called once at startup
{
  UniqueID = ESP.getEfuseMac();                    // get unique ID of the CPU/chipset

  Serial.begin(115200);                            // start serial console
  delay(200);

  Serial.println("RadioLib OGN-Tracker demo");

  Radio.beginFSK(868.2, 100.0, 50.0, 250.0, 0, 8); // start the RF chip in FSK mode (not LoRa)
#ifdef HAS_SX1262
  Radio.setTCXO(2.4);
  Radio.setDio2AsRfSwitch();
#endif
}

static int ReceiveLoop(int ListenTime)             // receive/listen for packets for a given time period
{ int state=Radio.startReceive(2*OGN_TxPacket<OGN1_Packet>::Bytes);  // enter reception mode (if it was idle)
  // printf("Radio.startReceive(%d) => %d\n", 2*OGN_TxPacket<OGN1_Packet>::Bytes, state);
  int Count=0;
  uint32_t Start = millis();                       // [ms] record the start time to meaure the time period
  for( ; ; )
  { uint32_t Time = millis()-Start;                //
    if(Time>=ListenTime) break;                    // end the loop when end-of-time reached
    if(digitalRead(LORA_IRQ))                      // if IRQ is high => new packet received
    // if(Radio.getIrqStatus())                       // this actually reads all 16 IRQ flags
    { float RSSI = Radio.getRSSI(true);                 // read the RSSI, but we are too slow, maybe greater RSSI integration time - but not supported by RadioLib
      Radio.readData(RxBuffer, 2*OGN_TxPacket<OGN1_Packet>::Bytes);   // read the packet data
      uint8_t PktIdx=0;
      for(uint8_t Idx=0; Idx<OGN_TxPacket<OGN1_Packet>::Bytes; Idx++) // loop over packet bytes
      { uint8_t ByteH = RxBuffer[PktIdx++];
        ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F; // decode manchester, detect (some) errors
        uint8_t ByteL = RxBuffer[PktIdx++];
        ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F; // second nibble
        RxPktData.Data[Idx]=(ByteH<<4) | ByteL;
        RxPktData.Err [Idx]=(ErrH <<4) | ErrL ; }
      uint8_t DecErr = RxPktData.Decode(RxPacket, Decoder);                  // run through LDPC FEC decoder
      printf("RX: %3.1fdBm LDPC:%d => %d bit errors\n", RSSI, DecErr, RxPacket.RxErr); // DedErr should be zero, if not the LDPC did not converge
      Count++; }                                   // count received packets
    delay(1); }
  Radio.standby();                                 // end reception/listen period
  return Count; }                                  // return number of packets received

void loop()                                        // 
{
  Serial.println("Time-slot");

  EncodePacket(TxPacket);                          // encode position/speed/other data in the OGN packet
  int Bytes=Manchester(ManchPacket, TxPacket);     // convert to Manchester
  SetupOGN(0, 10);                                 // setup for OGN transmission/reception on channel #0 (868.2MHz) with 10dBm
  Radio.transmit(ManchPacket, Bytes);              // transmit the Manchester encoded packet on channel #0
  ReceiveLoop(500);                                // receive OGN packets for 0.5sec

  EncodePacket(TxPacket);                          // 
  Bytes=Manchester(ManchPacket, TxPacket);         //
  SetupOGN(1, 10);                                 // setup for OGN transmission on channel #1 (868.4MHz)
  Radio.transmit(ManchPacket, Bytes);              //
  ReceiveLoop(500);

}
