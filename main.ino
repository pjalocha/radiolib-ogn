#include <Arduino.h>

#include <SPI.h>

#include <RadioLib.h>

#include "ogn.h"                                   // OGN packet format
#include "ldpc.h"                                  // Low Density Parity Code encoder and decoder
#include "manchester.h"                            // Manchester encoding/ecoding table

#ifdef HAS_SX1276
SX1276 Radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, RADIOLIB_NC);
#endif

#ifdef HAS_SX1262
SX1262 Radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY);
#endif

// OGNv1 SYNC:       0x0AF3656C encoded in Manchester
static const uint8_t OGN1_SYNC[8] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A };

static const uint8_t PAW_SYNC [8] = { 0xB4, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x18, 0x71 };

void SetupOGN(int Channel=0, float TxPower=10)                // Radio setup for given channel #0 or #1
{ Radio.setFrequency(868.2+0.2*Channel);                      // [MHz]  transmission/reception frequency
  Radio.setBitRate(100.0);                                    // [kpbs] 100kbps bit rate but we transmit Manchester encoded thus effectively 50 kbps
  Radio.setFrequencyDeviation(50.0);                          // [kHz]  +/-50kHz deviation
  Radio.setRxBandwidth(250.0);                                // [kHz]  250kHz bandwidth
  Radio.setOutputPower(TxPower);                              // [dBm]  transmission power
  Radio.setCurrentLimit(100);                                 // [mA]   depends on transmission power
  Radio.setDataShaping(RADIOLIB_SHAPING_0_5);                 // [BT]   FSK modulation shaping 
  Radio.setPreambleLength(8);                                 // [bits] minimal preamble
  Radio.fixedPacketLengthMode(2*OGN_TxPacket<OGN1_Packet>::Bytes); // [bytes] Fixed packet size mode
  Radio.setSyncWord((uint8_t *)OGN1_SYNC, 8); }               // SYNC sequence: 8 bytes which is equivalent to 4 bytes

OGN_TxPacket<OGN1_Packet> TxPacket;                           // encoded OGN packet;
uint8_t ManchPacket[2*OGN_TxPacket<OGN1_Packet>::Bytes];      // Manchester encoded OGN Packet;

void EncodePacket(OGN_TxPacket<OGN1_Packet> &TxPacket)
{
  TxPacket.Packet.HeaderWord = 0;                  // start with clean header
  TxPacket.Packet.Header.Address  = 0x123456;      // set address: 24-bit number
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
  TxPacket.Packet.Print();                         // print the encoded information
                                                   // Note: we decode before whitenning the data part

  TxPacket.Packet.Whiten();                        // whiten the data portion of the packet
  TxPacket.calcFEC();                              // calculate the redundancy part of the packet
}

int Manchester(uint8_t *Out, const OGN_TxPacket<OGN1_Packet> &TxPacket) // Encode as Manchester
{ const uint8_t *Inp = TxPacket.Byte();
  int Len=0;
  for(int Idx=0; Idx<TxPacket.Bytes; Idx++)
  { uint8_t Byte=Inp[Idx];
    Out[Len++]=ManchesterEncode[Byte>>4];
    Out[Len++]=ManchesterEncode[Byte&0x0F]; }
  return Len; }                                    // returns number of bytes in the encoded packet

void setup()
{
  Serial.begin(115200);
  delay(200);

  Radio.beginFSK();                                // RF in FSK mode (not LoRa)
}

void loop()
{
  EncodePacket(TxPacket);                          // encode position/speed/other data in the OGN packet
  int Bytes=Manchester(ManchPacket, TxPacket);     // convert to Manchester
  SetupOGN(0, 2);                                  // setup for OGN transmission on channel #0 (868.2MHz)
  Radio.transmit(ManchPacket, Bytes);              // transmit the Manchester encoded packet on channel #0
  delay(400);                                      // wait
  EncodePacket(TxPacket);                          // 
  Bytes=Manchester(ManchPacket, TxPacket);         //
  SetupOGN(1, 2);                                  // setup for OGN transmission on channel #1 (868.4MHz)
  Radio.transmit(ManchPacket, Bytes);              //
  delay(400);
}
