#ifndef __RFM_H__
#define __RFM_H__

#include <stdint.h>

// -----------------------------------------------------------------------------------------------------------------------

// #include "config.h"
#include "ogn.h"
#include "fanet.h"
#include "paw.h"

class RFM_LoRa_Config
{ public:

  union
  { uint32_t Word;
    struct
    { uint8_t Spare   :2;
      uint8_t InvIQ   :1; // 0..1, invert IQ (common for Rx/Tx for the SX1262 chip)
      uint8_t LowRate :1; // 0..1, optimize for low-rate (strips two lowest bits of every symbol)
      uint8_t TxInv   :1; // 0..1, invert on TX
      uint8_t RxInv   :1; // 0..1, invert on RX <- probably inverted logic
      uint8_t IHDR    :1; // 0..1, implicit header (no header on TX)
      uint8_t CRC     :1; // 0..1, produce CRC on TX and check CRC on RX
      uint8_t CR      :4; // 1..4, Coding Rate
      uint8_t SF      :4; // 6..12, Spreading Factor
      uint8_t BW      :4; // 0..9, 7=125kHz, 8=250kHz, 9=500kHz
      uint8_t Preamble:4; // 0..15, preamble symbols
      uint8_t SYNC    :8; // 0..0xFF, SYNC
    } ;
  } ;

  public:
   void Print(void)
   { printf("RFM_LoRa_Config: SYNC:0x%02X/%d, BW%d, SF%d, CR%d, IHDR:%d, Inv:%d/%d, LR:%d\n", SYNC, Preamble, BW, SF, CR, IHDR, RxInv, TxInv, LowRate); }

   uint16_t getAirTime(uint8_t PktLen) // [ms] estimated time on the air for given packet length
   { uint16_t Symbols = Preamble+2+3+8;                                 // [symbols] preamble, SYNC and header
     uint8_t  NibPerBlock = SF; if(LowRate) NibPerBlock-=2;             // [nibbles/block] byte-halfs
     uint8_t  SymbPerBlock = 4+CR;                                      // [symbols]
     uint16_t Nibbles = 2*(PktLen+2);                                   // [nibbles]
     uint8_t  Blocks = (Nibbles+NibPerBlock-1)/NibPerBlock;             // [FEC blocks] for the data part
              Symbols += (uint16_t)Blocks*SymbPerBlock;                 // [symbols]
     uint32_t SymbTime = 1<<(10-BW+SF);                                 // [usec/symbol]
     uint32_t Time = SymbTime*Symbols;                                  // [usec]
     return (Time+500)/1000; }

} ;

const RFM_LoRa_Config RFM_FNTcfg { 0xF1587194 } ; // LoRa seting for FANET
const RFM_LoRa_Config RFM_WANcfg { 0x34877194 } ; // LoRa WAN setting for TX

class RFM_LoRa_RxPacket
{ public:

   union
   { uint8_t Flags;
     struct
     { uint8_t  CR:3;  // Coding rate used (RX) or to be used (TX)
       bool hasCRC:1;  // CRC was there (RX)
       bool badCRC:1;  // CRC was bad (RX)
       bool   Done:1;
     } ;
   } ;
   uint8_t Len;       // [bytes] packet length
   static const int MaxBytes = 40;
   uint8_t Byte[MaxBytes+2];

   uint32_t  sTime;         // [ s] reception time: seconds
   uint16_t msTime;         // [ms] reception time: miliseconds
   int16_t FreqOfs;         // [ 10Hz]
    int8_t SNR;             // [0.25dB]
    int8_t RSSI;            // [dBm]
   uint8_t BitErr;          // number of bit errors
   uint8_t CodeErr;         // number of block errors

  public:
   void Print(void) const
   { char HHMMSS[8];
     Format_HHMMSS(HHMMSS, sTime);  HHMMSS[6]='h'; HHMMSS[7]=0;
     printf("%s CR%c%c%c %3.1fdB/%de %+3.1fkHz ", HHMMSS, '0'+CR, hasCRC?'c':'_', badCRC?'-':'+', 0.25*SNR, BitErr, 1e-2*FreqOfs);
     for(uint8_t Idx=0; Idx<Len; Idx++)
       printf("%02X", Byte[Idx]);
     printf("\n"); }

} ;

class RFM_FSK_RxPktData             // OGN packet received by the RF chip
{ public:
   static const uint8_t Bytes=26;   // [bytes] number of bytes in the packet
   uint32_t Time;                   // [sec] Time slot
   uint16_t msTime;                 // [ms] reception time since the PPS[Time]
   uint8_t Channel;                 // [   ] channel where the packet has been recieved
   uint8_t RSSI;                    // [-0.5dBm] receiver signal strength
   uint8_t Data[Bytes];             // Manchester decoded data bits/bytes
   uint8_t Err [Bytes];             // Manchester decoding errors

  public:

   void Print(void (*CONS_UART_Write)(char), uint8_t WithData=0) const
   { // uint8_t ManchErr = Count1s(RxPktErr, 26);
     Format_String(CONS_UART_Write, "RxPktData: ");
     Format_HHMMSS(CONS_UART_Write, Time);
     CONS_UART_Write('+');
     Format_UnsDec(CONS_UART_Write, msTime, 4, 3);
     CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, Channel);
     CONS_UART_Write('/');
     Format_SignDec(CONS_UART_Write, (int16_t)(-5*(int16_t)RSSI), 3, 1);
     Format_String(CONS_UART_Write, "dBm\n");
     if(WithData==0) return;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, Data[Idx]); }
     CONS_UART_Write('\r'); CONS_UART_Write('\n');
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, Err[Idx]); }
     CONS_UART_Write('\r'); CONS_UART_Write('\n');
   }

   bool NoErr(void) const
   { for(uint8_t Idx=0; Idx<Bytes; Idx++)
       if(Err[Idx]) return 0;
     return 1; }

   uint8_t ErrCount(void) const                         // count detected manchester errors
   { uint8_t Count=0;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
       Count+=Count1s(Err[Idx]);
     return Count; }

   uint8_t ErrCount(const uint8_t *Corr) const          // count errors compared to data corrected by FEC
   { uint8_t Count=0;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
       Count+=Count1s((uint8_t)((Data[Idx]^Corr[Idx])&(~Err[Idx])));
     return Count; }

 template <class OGNx_Packet>
  uint8_t Decode(OGN_RxPacket<OGNx_Packet> &Packet, LDPC_Decoder &Decoder, uint8_t Iter=32) const
  { uint8_t Check=0;
    uint8_t RxErr = ErrCount();                                // conunt Manchester decoding errors
    Decoder.Input(Data, Err);                                  // put data into the FEC decoder
    for( ; Iter; Iter--)                                       // more loops is more chance to recover the packet
    { Check=Decoder.ProcessChecks();                           // do an iteration
      if(Check==0) break; }                                    // if FEC all fine: break
    Decoder.Output(Packet.Packet.Byte());                      // get corrected bytes into the OGN packet
    RxErr += ErrCount(Packet.Packet.Byte());
    if(RxErr>15) RxErr=15;
    Packet.RxErr  = RxErr;
    Packet.RxChan = Channel;
    Packet.RxRSSI = RSSI;
    Packet.Correct= Check==0;
    return Check; }

} ;

#if defined(WITH_RFM69) || defined(WITH_RFM95) || defined(WITH_SX1272) || defined(WITH_SX1262)

// -----------------------------------------------------------------------------------------------------------------------

/*
SX1262 setup calls
.Cmd_Writet(0x80, 0x00);
.Cmd_Writet(0x96, 0x01);                          // set regulator mode: 0=only LDO, 1=DC-DC+LDO
.Cmd_Writet(0x9D, 0x01);                          // DIO2 as RF switch
.Cmd_Writet(0x97, 0x0100012C);                    // DIO3 as TCXO control: voltage, timeout
.Cmd_Writet(0x80, 0x00);                          // set Standby 00=RC 01=Xtal
.Cmd_Writet(0x8A, 0x00);                          // 0=FSK 1=LoRa

.Cmd_Writet(0x86, 0x36433333);                    // set frequency [32.0MHz/2^25]
.Cmd_Writet(0x8B, 0x00 28 00 09 0A 00 CC CD);     // set modulation parameters: bitrate, BT, bandwidth, deviation
.Cmd_Writet(0x8C, 0x00 04 00 40 00 00 34 01 00);  // set packet parameters: preamble len. preamble det. len. sync. len. payload len.
.Regs_Write(0x06C0, 0xAA6655A59699965A);          // SYNC
.Cmd_Writet(0x80, 0x00);                          // set standby
.Cmd_Writet(0x82, 0x000000);                      // set RX mode
.Cmd_Writet(0x80, 0x00);                          // set standby
.Cmd_Writet(0x86, 0x36466667);                    // set frequency 868.4MHz
.Cmd_Writet(0x8B, 0x002800090A00CCCD);            // set modulation parameters
.Cmd_Writet(0x8C, 0x000400400000340100);          // set packet parameters
.Cmd_Writet(0x95, 0x04 07 00 01);                 // PA config
.Cmd_Writet(0x8E, 0x0E04);                        // TX parm: power and ramp time
.Cmd_Writet(0x8F, 0x8000);                        // set TX/RX buffer address: TX=0x80, RX=0x00
.Cmd_Writet(0x83, 0x000000);                      // set TX mode: timeout [15.625us]
.Cmd_Writet(0x80, 0x00);                          // set standby
... transmission ...
.Cmd_Writet(0x82, 0x000000);                      // set RX mode: timeout [15.625us] 000000 = single mode, FFFFFF = continous mode
.Cmd_Writet(0x80, 0x00);                          // set standby
.Cmd_Writet(0x82, 0x000000);                      // set RX mode
.Cmd_Writet(0x80, 0x00);                          // set standby
.Cmd_Writet(0x86, 0x36433333);                    // set frequency
.Cmd_Writet(0x8B, 0x002800090A00CCCD);
.Cmd_Writet(0x8C, 0x000400400000340100);
.Cmd_Writet(0x95, 0x04070001);
.Cmd_Writet(0x8E, 0x0E04);
.Cmd_Writet(0x8F, 0x8000);
.Cmd_Writet(0x83, 0x000000);
.Cmd_Writet(0x80, 0x00);
... transmission ...
.Cmd_Writet(0x82, 0x000000);

*/

// -----------------------------------------------------------------------------------------------------------------------

// OGN frequencies for Europe: 868.2 and 868.4 MHz
// static const uint32_t OGN_BaseFreq  = 868200000; // [Hz] base frequency
// static const uint32_t OGN_ChanSpace =   0200000; // [Hz] channel spacing

// OGN frequencies for Australia: 917.0 base channel, with 0.4MHz channel raster and 24 hopping channels
// static const uint32_t OGN_BaseFreq  = 921400000; // [Hz] base frequency
// static const uint32_t OGN_ChanSpace =    400000; // [Hz] channel spacing

// static const double    XtalFreq = 32e6;        // [MHz] RF chip crystal frequency
//
// static const uint32_t BaseFreq  = floor(OGN_BaseFreq /(XtalFreq/(1<<19))+0.5); // conversion from RF frequency
// static const uint32_t ChanSpace = floor(OGN_ChanSpace/(XtalFreq/(1<<19))+0.5); // to RF chip synthesizer setting

// integer formula to convert from frequency to the RFM69 scheme: IntFreq = ((Freq<<16)+ 20)/ 40; where Freq is in [100kHz]
//                                                            or: IntFreq = ((Freq<<14)+ 50)/100; where Freq is in [ 10kHz]
//                                                            or: IntFreq = ((Freq<<12)+125)/250; where Freq is in [  1kHz]
//                                                            or: IntFreq = ((Freq<<11)+ 62)/125; where Freq is in [  1kHz]
// 32-bit arythmetic is enough in the above formulas


#ifdef WITH_RFM69

#include "sx1231.h"            // register addresses and values for SX1231 = RFM69

#define REG_AFCCTRL    0x0B  // AFC method

#define REG_TESTLNA     0x58  // Sensitivity boost ?
#define REG_TESTPA1     0x5A  // only present on RFM69HW/SX1231H
#define REG_TESTPA2     0x5C  // only present on RFM69HW/SX1231H
#define REG_TESTDAGC    0x6F  // Fading margin improvement ?
#define REG_TESTAFC     0x71

#define RF_IRQ_AutoMode       0x0200

#endif // of WITH_RFM69

#ifdef WITH_RFM95
#include "sx1276.h"
#define RF_IRQ_PreambleDetect 0x0200 //
#endif // of WITH_RFM95

#ifdef WITH_SX1262
#include "sx1262.h"

// #define RF_IRQ_PacketSent     0x0001 // packet transmission was completed
// #define RF_IRQ_PayloadReady   0x0002 //
// #define RF_IRQ_PreambleDetect 0x0004 //
// #define RF_IRQ_SyncAddrMatch  0x0008 //
// #define RF_IRQ_CrcErr         0x0040 //
// #define RF_IRQ_Timeout        0x0200 //

#endif

#if defined(WITH_RFM69) || defined(WITH_RFM95) || defined(WITH_SX1272)
                                     // bits in IrqFlags1 and IrfFlags2
#define RF_IRQ_ModeReady      0x8000 // mode change done (between some modes)
#define RF_IRQ_RxReady        0x4000 // receiver ready
#define RF_IRQ_TxReady        0x2000 // transmitter ready
#define RF_IRQ_PllLock        0x1000 //
#define RF_IRQ_Rssi           0x0800 //
#define RF_IRQ_Timeout        0x0400
#define RF_IRQ_PreambleDetect 0x0200
#define RF_IRQ_SyncAddrMatch  0x0100

#define RF_IRQ_FifoFull       0x0080 //
#define RF_IRQ_FifoNotEmpty   0x0040 // at least one byte in the FIFO
#define RF_IRQ_FifoLevel      0x0020 // more bytes than FifoThreshold
#define RF_IRQ_FifoOverrun    0x0010 // write this bit to clear the FIFO
#define RF_IRQ_PacketSent     0x0008 // packet transmission was completed
#define RF_IRQ_PayloadReady   0x0004
#define RF_IRQ_CrcOk          0x0002
#define RF_IRQ_LowBat         0x0001

#endif

#include "manchester.h"

class RFM_TRX
{ public:                             // hardware access functions

#ifdef USE_BLOCK_SPI                                                    // SPI transfers in blocks, implicit control of the SPI-select
   void (*TransferBlock)(uint8_t *Data, uint8_t Len);
   static const size_t MaxBlockLen = 64;
   uint8_t Block_Buffer[MaxBlockLen];

   uint8_t *Block_Read(uint8_t Len, uint8_t Addr)                       // read given number of bytes from given Addr
   { Block_Buffer[0]=Addr; memset(Block_Buffer+1, 0, Len);
     (*TransferBlock) (Block_Buffer, Len+1);
     return  Block_Buffer+1; }                                          // return the pointer to the data read from the given Addr

   uint8_t *Block_Write(const uint8_t *Data, uint8_t Len, uint8_t Addr) // write given number of bytes to given Addr
   { Block_Buffer[0] = Addr | 0x80; memcpy(Block_Buffer+1, Data, Len);
     // printf("Block_Write( [0x%02X, .. ], %d, 0x%02X) .. [0x%02X, 0x%02X, ...]\n", Data[0], Len, Addr, Block_Buffer[0], Block_Buffer[1]);
     (*TransferBlock) (Block_Buffer, Len+1);
     return  Block_Buffer+1; }

#ifdef WITH_SX1262
   uint16_t WaitWhileBusy(uint16_t Loops=100)             // 50 seems to be still too short on RPI
   { for( ; Loops; Loops--)
     { if(!readBusy()) break; }
     return Loops; }

   uint16_t WaitWhileBusy_ms(uint16_t ms=10)
   { WaitWhileBusy(50);
     for( ; ms; ms--)
     { if(!readBusy()) break;
       Delay_ms(1); }
     return ms; }

   uint8_t *Cmd_Write(uint8_t Cmd, const uint8_t *Data, uint8_t Len)  // command code, Data[Len]
   { WaitWhileBusy_ms(); // if(WaitWhileBusy()==0) return 0;
#ifdef DEBUG_PRINT
     if(readBusy())
     { Format_String(CONS_UART_Write, "Cmd_Write(0x");
       Format_Hex(CONS_UART_Write, Cmd);
       Format_String(CONS_UART_Write, ") => RF busy !\n"); }
     CONS_UART_Write(readBusy()?'!':'.');                  // Busy-line state
     Format_String(CONS_UART_Write, "Cmd_Write(0x");
     Format_Hex(CONS_UART_Write, Cmd);
     Format_String(CONS_UART_Write, ", 0x");
     for(uint8_t Idx=0; Idx<Len; Idx++)
       Format_Hex(CONS_UART_Write, Data[Idx]);
     Format_String(CONS_UART_Write, ");\n");
#endif
     return Block_Write(Data, Len, Cmd); }
   // { uint8_t *Ret = Block_Write(Data, Len, Cmd); printf("Cmd_Write: %02X %02X\n", Ret[0], Ret[1]); return Ret; }

   uint8_t *Cmd_Read(uint8_t Cmd, uint8_t Len)
   { WaitWhileBusy_ms(); // if(WaitWhileBusy()==0) return 0;
     Block_Buffer[0] = Cmd; memset(Block_Buffer+1, 0, Len+1);
     (*TransferBlock) (Block_Buffer, Len+2);
#ifdef DEBUG_PRINT
     CONS_UART_Write(readBusy()?'!':'.');                  // Busy-line state
     Format_String(CONS_UART_Write, "Cmd_Read(0x");
     Format_Hex(CONS_UART_Write, Cmd);
     Format_String(CONS_UART_Write, ") => 0x");
     for(uint8_t Idx=0; Idx<Len+2; Idx++)
       Format_Hex(CONS_UART_Write, Block_Buffer[Idx]);
     Format_String(CONS_UART_Write, ";\n");
#endif
     return  Block_Buffer+2;  }

   uint8_t *Regs_Write(uint16_t Addr, const uint8_t *Data, uint8_t Len)  // register-write code, 2-byte Address, Data[Len]
   { WaitWhileBusy_ms(); // if(WaitWhileBusy()==0) return 0;
#ifdef DEBUG_PRINT
     CONS_UART_Write(readBusy()?'!':'.');                  // Busy-line state
     Format_String(CONS_UART_Write, "Regs_Write(0x");
     Format_Hex(CONS_UART_Write, Addr);
     Format_String(CONS_UART_Write, ", 0x");
     for(uint8_t Idx=0; Idx<Len; Idx++)
       Format_Hex(CONS_UART_Write, Data[Idx]);
     Format_String(CONS_UART_Write, ");\n");
#endif
     Block_Buffer[0] = CMD_WRITEREGISTER; Block_Buffer[1] = Addr>>8; Block_Buffer[2] = Addr; memcpy(Block_Buffer+3, Data, Len);
     (*TransferBlock) (Block_Buffer, Len+3);
     return  Block_Buffer+3; }

   uint8_t *Regs_Read(uint16_t Addr, uint8_t Len)  // register-read code, 2-byte Address, zero, Data[Len]
   { WaitWhileBusy_ms(); // if(WaitWhileBusy()==0) return 0;
     Block_Buffer[0] = CMD_READREGISTER; Block_Buffer[1] = Addr>>8; Block_Buffer[2] = Addr;  memset(Block_Buffer+3, 0, Len+1);
     (*TransferBlock) (Block_Buffer, Len+4);
     return  Block_Buffer+4; }

   uint8_t *Buff_Write(uint8_t Ofs, const uint8_t *Data, uint8_t Len)   // buffer-write code, 1-byte offset, Data[Len]
   { WaitWhileBusy_ms(); // if(WaitWhileBusy()==0) return 0;
#ifdef DEBUG_PRINT
     if(readBusy())
     { Format_String(CONS_UART_Write, "Buff_Write(0x");
       Format_Hex(CONS_UART_Write, Ofs);
       Format_String(CONS_UART_Write, ") => RF busy !\n"); }
     CONS_UART_Write(readBusy()?'!':'.');                  // Busy-line state
     Format_String(CONS_UART_Write, "Buff_Write(0x");
     Format_Hex(CONS_UART_Write, Ofs);
     Format_String(CONS_UART_Write, ", 0x");
     for(uint8_t Idx=0; Idx<Len; Idx++)
       Format_Hex(CONS_UART_Write, Data[Idx]);
     Format_String(CONS_UART_Write, ");\n");
#endif
     Block_Buffer[0] = CMD_WRITEBUFFER; Block_Buffer[1] = Ofs; memcpy(Block_Buffer+2, Data, Len);
     (*TransferBlock) (Block_Buffer, Len+2);
     return  Block_Buffer+2; }

   uint8_t *Buff_Read(uint8_t Ofs, uint8_t Len)                         // buffer-read code, 1-byte offset, zero, Data[Len]
   { WaitWhileBusy_ms(); // if(WaitWhileBusy()==0) return 0;
     Block_Buffer[0] = CMD_READBUFFER; Block_Buffer[1] = Ofs; memset(Block_Buffer+2, 0, Len+1);
     (*TransferBlock) (Block_Buffer, Len+3);
     return  Block_Buffer+3; }
#endif

#else // USE_BLOCK_SPI                                                  // SPI transfers as single bytes, explicit control of the SPI-select
   void (*Select)(void);                                                // activate SPI select
   void (*Deselect)(void);                                              // desactivate SPI select
   uint8_t (*TransferByte)(uint8_t);                                    // exchange one byte through SPI
#endif

   void (*Delay_ms)(int ms);
   bool (*DIO0_isOn)(void);                                              // read DIO0 = packet is ready
#ifdef WITH_SX1262
   bool (*Busy_isOn)(void);                                              // sx1262 has an additional BUSY line
#endif
   // bool (*DIO4_isOn)(void);
   void (*RESET)(uint8_t On);                                            // activate or desactivate the RF chip reset

   bool readIRQ(void) { return (*DIO0_isOn)(); }
#ifdef WITH_SX1262
   bool readBusy(void) { return (*Busy_isOn)(); }                        // read the BUSY line of the sx1262
#endif
                                      // the following are in units of the synthesizer with 8 extra bits of precision
   uint32_t BaseFrequency;            // [32MHz/2^19/2^8] base frequency = channel #0
   uint32_t ChannelSpacing;           // [32MHz/2^19/2^8] spacing between channels
    int16_t FreqCorr;                 // [0.1ppm]
    int16_t Channel;                  // [       integer] channel being used

  uint8_t chipVer;                    // [] version ID read from the RF chip
   int8_t chipTemp;                   // [degC] temperature read from the RF chip
  uint8_t averRSSI;                   // [-0.5dB]
  uint8_t dummy;

/*
#ifdef WITH_RFM95
   void WriteDefaultReg(void)
   { const uint8_t Default[64] = {  0x00, 0x01, 0x1A, 0x0B, 0x00, 0x52, 0xE4, 0xC0, 0x00, 0x0F, 0x19, 0x2B, 0x20, 0x08, 0x02, 0x0A,
                                    0xFF, 0x00, 0x15, 0x0B, 0x28, 0x0C, 0x12, 0x47, 0x32, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
                                    0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x03, 0x93, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                                    0x90, 0x40, 0x40, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0xF5, 0x20, 0x82, 0x00, 0x02, 0x80, 0x40 };
     WriteBytes(Default+0x01, 0x0F, 0x01);
     WriteBytes(Default+0x10, 0x10, 0x10);
     WriteBytes(Default+0x20, 0x10, 0x20);
     WriteBytes(Default+0x30, 0x10, 0x30);
     WriteWord(0x0000, 0x40);
     WriteByte(0x2D, 0x44);
     WriteByte(0x09, 0x4B);
     WriteByte(0x84, 0x4D);
     WriteByte(0x00, 0x5D);
     WriteByte(0x13, 0x61);
     WriteByte(0x0E, 0x62);
     WriteByte(0x5B, 0x63);
     WriteByte(0xDB, 0x64);
   }
#endif
*/

   static uint32_t calcSynthFrequency(uint32_t Frequency) { return (((uint64_t)Frequency<<16)+7812)/15625; }

  public:
   void setBaseFrequency(uint32_t Frequency=868200000) { BaseFrequency=calcSynthFrequency(Frequency); } // [Hz] => [synth]
   void setChannelSpacing(uint32_t  Spacing=   200000) { ChannelSpacing=calcSynthFrequency(Spacing); }  // [Hz] => [synth]
   void setFrequencyCorrection(int16_t ppmFreqCorr=0)  { FreqCorr = ppmFreqCorr; }                      // [0.1ppm]

   void setFrequency(uint32_t Freq)                                 // [Hz] set for given frequency
   { Freq = calcSynthFrequency(Freq);                               // [32MHz/2^27]
     int32_t Corr = ((int64_t)Freq*FreqCorr+5000000)/10000000;      // [32MHz/2^27]
     Freq+=Corr; WriteFreq(Freq); }                                 // [32MHz/2^27] write into the RF chip

   void setChannel(int16_t newChannel)                              // set for given channel
   { Channel=newChannel;
     uint32_t Freq = BaseFrequency+ChannelSpacing*Channel;          // [32MHz/2^27]
      int32_t Corr = ((int64_t)Freq*FreqCorr+5000000)/10000000;     // [32MHz/2^27]
              Freq += Corr;                                         // [32MHz/2^27]
     WriteFreq(Freq); }                                             // [32MHz/2^27] write into the RF chip

   uint8_t getChannel(void) const { return Channel; }

#ifdef USE_BLOCK_SPI

   static uint16_t SwapBytes(uint16_t Word) { return (Word>>8) | (Word<<8); }

   uint8_t WriteByte(uint8_t Byte, uint8_t Addr=0) // write Byte
   { // printf("WriteByte(0x%02X => [0x%02X])\n", Byte, Addr);
     uint8_t *Ret = Block_Write(&Byte, 1, Addr); return *Ret; }

   void WriteWord(uint16_t Word, uint8_t Addr=0) // write Word => two bytes
   { // printf("WriteWord(0x%04X => [0x%02X])\n", Word, Addr);
     uint16_t Swapped = SwapBytes(Word); Block_Write((uint8_t *)&Swapped, 2, Addr); }

   uint8_t ReadByte (uint8_t Addr=0)
   { uint8_t *Ret = Block_Read(1, Addr);
     // printf("ReadByte(0x%02X) => 0x%02X\n", Addr, *Ret );
     return *Ret; }

   uint16_t ReadWord (uint8_t Addr=0)
   { uint16_t *Ret = (uint16_t *)Block_Read(2, Addr);
     // printf("ReadWord(0x%02X) => 0x%04X\n", Addr, SwapBytes(*Ret) );
     return SwapBytes(*Ret); }

   void WriteBytes(const uint8_t *Data, uint8_t Len, uint8_t Addr=0)
   { Block_Write(Data, Len, Addr); }

#if defined(WITH_RFM95) || defined(WITH_SX1272) || defined(WITH_RFM69)
   void WriteFreq(uint32_t Freq)                       // [32MHz/2^27] Set center frequency in units of RFM69 synth.
   { const uint8_t Addr = REG_FRFMSB;
     Freq = (Freq+128)>>8;                             // [32MHz/2^19]
     // printf("RFM::WriteFreq(%06X)\n", Freq);
     uint8_t Buff[4];
     Buff[0] = Freq>>16;
     Buff[1] = Freq>> 8;
     Buff[2] = Freq    ;
     Buff[3] =        0;
     Block_Write(Buff, 3, Addr); }

   uint32_t ReadFreq(uint8_t Addr=REG_FRFMSB)
   { uint8_t *Data = Block_Read(3, Addr);
     uint32_t Freq=Data[0]; Freq<<=8; Freq|=Data[1]; Freq<<=8; Freq|=Data[2];
     return Freq; }                             // [32MHz/2^19]

   void WriteFIFO(const uint8_t *Data, uint8_t Len)
   { Block_Write(Data, Len, REG_FIFO); }

   uint8_t *ReadFIFO(uint8_t Len)
   { return Block_Read(Len, REG_FIFO); }
#endif

#ifdef WITH_SX1262
   void WriteFreq(uint32_t Freq)               // [32MHz/2^27] set transmission frequency
   { uint8_t Buff[4];
     Freq = (Freq+2)>>2;                       // [32MHz/2^25]
     Buff[0] = Freq>>24;
     Buff[1] = Freq>>16;
     Buff[2] = Freq>> 8;
     Buff[3] = Freq    ;
     Cmd_Write(CMD_SETRFFREQUENCY, Buff, 4); }

   void WriteFIFO(const uint8_t *Data, uint8_t Len)
   { const uint8_t BaseOfs[2] = { 0x80, 0x00 };  // TX, RX offsets in the 256-byte buffer
     Cmd_Write(CMD_SETBUFFERBASEADDRESS, BaseOfs, 2);
     Buff_Write(0x80, Data, Len); }

   uint8_t *ReadFIFO(uint8_t Len)
   { uint8_t *BuffStat = Cmd_Read(CMD_GETRXBUFFERSTATUS, 2);        // length, offset
     return Buff_Read(BuffStat[1], BuffStat[0]); }
#endif

   void FNT_WritePacket(const uint8_t *Data, uint8_t Len)
   { WriteFIFO(Data, Len); }

   void OGN_WritePacket(const uint8_t *Data, uint8_t Len=26)         // write the packet data (26 bytes)
   { uint8_t Packet[2*Len];
     uint8_t PktIdx=0;
     for(uint8_t Idx=0; Idx<Len; Idx++)
     { uint8_t Byte=Data[Idx];
       Packet[PktIdx++]=ManchesterEncode[Byte>>4];                   // software manchester encode every byte
       Packet[PktIdx++]=ManchesterEncode[Byte&0x0F];
     }
     WriteFIFO(Packet, 2*Len); }

   void PAW_WritePacket(const uint8_t *Data, uint8_t Len=24)
   { uint8_t Packet[Len+1];
     for(uint8_t Idx=0; Idx<Len; Idx++)
     { Packet[Idx] = Data[Idx]; }
     PAW_Packet::Whiten(Packet, Len);
     Packet[Len] = PAW_Packet::CRC8(Packet, Len);
     WriteFIFO(Packet, Len+1); }

   void OGN_ReadPacket(uint8_t *Data, uint8_t *Err, uint8_t Len=26)         // read packet data from FIFO
   { uint8_t *Packet = ReadFIFO(2*Len);                                     // read 2x26 bytes from the RF chip RxFIFO
     uint8_t PktIdx=0;
     for(uint8_t Idx=0; Idx<Len; Idx++)                                     // loop over packet bytes
     { uint8_t ByteH = Packet[PktIdx++];
       ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F; // decode manchester, detect (some) errors
       uint8_t ByteL = Packet[PktIdx++];
       ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F;
       Data[Idx]=(ByteH<<4) | ByteL;
       Err [Idx]=(ErrH <<4) | ErrL ;
     }
   }

#else // single Byte transfer SPI

  private:
   uint8_t WriteByte(uint8_t Byte, uint8_t Addr=0) const  // write Byte
   { Select();
     TransferByte(Addr | 0x80);
     uint8_t Old=TransferByte(Byte);
     Deselect();
     return Old; }

   uint16_t WriteWord(uint16_t Word, uint8_t Addr=0) const // write Word => two bytes
   { Select();
     TransferByte(Addr | 0x80);
     uint16_t Old=TransferByte(Word>>8);             // upper byte first
     Old = (Old<<8) | TransferByte(Word&0xFF);       // lower byte second
     Deselect();
     return Old; }

   void WriteBytes(const uint8_t *Data, uint8_t Len, uint8_t Addr=0) const
   { Select();
     TransferByte(Addr | 0x80);
     for(uint8_t Idx=0; Idx<Len; Idx++)
     { TransferByte(Data[Idx]); }
     Deselect(); }

   uint8_t ReadByte (uint8_t Addr=0) const
   { Select();
     TransferByte(Addr);
     uint8_t Byte=TransferByte(0);
     Deselect();
     return Byte; }

   uint16_t ReadWord (uint8_t Addr=0) const
   { Select();
     TransferByte(Addr);
     uint16_t Word=TransferByte(0);
     Word = (Word<<8) | TransferByte(0);
     Deselect();
     return Word; }

  public:
   uint32_t WriteFreq(uint32_t Freq) const                       // [32MHz/2^19] Set center frequency in units of RFM69 synth.
   { const uint8_t Addr = REG_FRFMSB;
     Freq = (Freq+128)>>8;                                       // [32MHz/2^19]
     Select();
     TransferByte(Addr | 0x80);
     uint32_t Old  =  TransferByte(Freq>>16);
     Old = (Old<<8) | TransferByte(Freq>>8);
     Old = (Old<<8) | TransferByte(Freq);                        // actual change in the frequency happens only when the LSB is written
     Deselect();
     return Old<<8; }                                            // return the previously set frequency

   void WriteFIFO(const uint8_t *Data, uint8_t Len)
   { const uint8_t Addr=REG_FIFO;                                // write to FIFO
     Select();
     TransferByte(Addr | 0x80);
     for(uint8_t Idx=0; Idx<Len; Idx++)
       TransferByte(Data[Idx]);
     Deselect(); }

   void OGN_WritePacket(const uint8_t *Data, uint8_t Len=26) const   // write the packet data (26 bytes)
   { const uint8_t Addr=REG_FIFO;                                // write to FIFO
     Select();
     TransferByte(Addr | 0x80);
     for(uint8_t Idx=0; Idx<Len; Idx++)
     { uint8_t Byte=Data[Idx];
       TransferByte(ManchesterEncode[Byte>>4]);                  // software manchester encode every byte
       TransferByte(ManchesterEncode[Byte&0x0F]);
     }
     Deselect(); }

   void PAW_WritePacket(const uint8_t *Data, uint8_t Len=24)
   { uint8_t Packet[Len+1];
     for(uint8_t Idx=0; Idx<Len; Idx++)
     { Packet[Idx] = Data[Idx]; }
     PAW_Packet::Whiten(Packet, Len);
     Packet[Len] = PAW_Packet::CRC8(Packet, Len);
     const uint8_t Addr=REG_FIFO;                                // write to FIFO
     Select();
     TransferByte(Addr | 0x80);
     for(uint8_t Idx=0; Idx<=Len; Idx++)
     { TransferByte(Packet[Idx]); }
     Deselect(); }

   void ReadPacketOGN(uint8_t *Data, uint8_t *Err, uint8_t Len=26) const       // read packet data from FIFO
   { const uint8_t Addr=REG_FIFO;
     Select();                                                              // select the RF chip: start SPI transfer
     TransferByte(Addr);                                                    // trasnfer the address/read: FIFO
     for(uint8_t Idx=0; Idx<Len; Idx++)                                     // loop over packet byte
     { uint8_t ByteH = 0;
       ByteH = TransferByte(ByteH);
       ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F; // decode manchester, detect (some) errors
       uint8_t ByteL = 0;
       ByteL = TransferByte(ByteL);
       ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F;
       Data[Idx]=(ByteH<<4) | ByteL;
       Err [Idx]=(ErrH <<4) | ErrL ;
     }
     Deselect(); }                                                          // de-select RF chip: end of SPI transfer

#endif // USE_BLOCK_SPI

#ifdef WITH_RFM69
   void FSK_WriteSYNC(uint8_t WriteSize, uint8_t SyncTol, const uint8_t *SyncData)
   { if(SyncTol>7) SyncTol=7;                                                // no more than 7 bit errors can be tolerated on SYNC
     if(WriteSize>8) WriteSize=8;                                            // up to 8 bytes of SYNC can be programmed
     WriteBytes(SyncData+(8-WriteSize), WriteSize, REG_SYNCVALUE1);          // write the SYNC, skip some initial bytes
     WriteByte(  0x80 | ((WriteSize-1)<<3) | SyncTol, REG_SYNCCONFIG);       // write SYNC length [bytes] and tolerance to errors [bits]
     WriteWord( /* 9-WriteSize, */ 1, REG_PREAMBLEMSB); }                    // write preamble length [bytes] (page 71)
//              ^ 8 or 9 ?
#endif

#if defined(WITH_RFM95) || defined(WITH_SX1272)
   void FSK_WriteSYNC(uint8_t WriteSize, uint8_t SyncTol, const uint8_t *SyncData)
   { if(SyncTol>7) SyncTol=7;
     if(WriteSize>8) WriteSize=8;
     WriteBytes(SyncData+(8-WriteSize), WriteSize, REG_SYNCVALUE1);        // write the SYNC, skip some initial bytes
     WriteByte(  0x90 | (WriteSize-1), REG_SYNCCONFIG);                    // write SYNC length [bytes] AAPS_sss
     WriteWord( /* 9-WriteSize, */ 1, REG_PREAMBLEMSB); }                  // write preamble length [bytes] (page 71)
//              ^ 8 or 9 ?
#endif

#ifdef WITH_SX1262
   void setTCXOctrlDIO3(uint8_t Volt=1, uint32_t Delay=320)                        // [0..7 => 1.6-3.3V] [1/64ms] p.82
   { uint8_t Data[4] = { Volt, (uint8_t)(Delay>>16), (uint8_t)(Delay>>8), (uint8_t)Delay }; Cmd_Write(CMD_SETDIO3ASTCXOCTRL, Data, 4); }
   void setRFswitchDIO2(uint8_t Mode=0) { Cmd_Write(CMD_SETDIO2ASRFSWITCHCTRL, &Mode, 1); } // enable DIO2 as RF switch
   void setRegulator(uint8_t Mode=0) { Cmd_Write(CMD_SETREGULATORMODE, &Mode, 1); } // 0=LDO, 1=DCDC+LDO

   void setModulation(uint8_t Mode=0) { Cmd_Write(CMD_SETPACKETTYPE, &Mode, 1); }   // 0=FSK, 1=LoRa
   void setLoRa(void) { setModulation(0x01); }                                      // switch to LoRa
   void setFSK(void)  { setModulation(0x00); }                                      // switch to FSK
   uint8_t getModulation(void) { return Cmd_Read(CMD_GETPACKETTYPE, 1)[0]; }

   uint8_t getStatus(void) { return Cmd_Read(CMD_GETSTATUS, 0)[-1]; }               // RMMM SSSR MMM: 2=STBY_RC, 3=STBY_XOSC, 4:FS, 5:RX, 6:TX p.95

   void WriteTxPower(int8_t TxPower)
   { if(TxPower>22) TxPower=22;                                                     // for high power PA
     else if(TxPower<(-9)) TxPower=(-9);
     uint8_t PAparm[4] = { 0x04, 0x07, 0x00, 0x01 } ;                               // for high power PA: paDutyCycle, hpMax, deviceSel, paLut
     Cmd_Write(CMD_SETPACONFIG, PAparm, 4);                                         // Power Amplifier Configuration
     uint8_t TxParm[2] = { (uint8_t)TxPower, 0x04 } ;                               // RampTime = 200us
     Cmd_Write(CMD_SETTXPARAMS, TxParm, 2); }                                       // 0x8E, Power, RampTime
   void WriteTxPowerMin(void) { WriteTxPower(-8); }

   void Calibrate(void)                                                             // Calibrate receiver image rejection
   { uint8_t CalParm[2] = { 0xD7, 0xDB }; // for 868MHz                             // { 0xE1, 0xE9 } for 915MHz
     Cmd_Write(CMD_SETTXPARAMS, CalParm, 2); }

   void FSK_WriteSYNC(uint8_t WriteSize, uint8_t SyncTol, const uint8_t *SyncData)
   { if(SyncTol>7) SyncTol=7;
     if(WriteSize>8) WriteSize=8;
     uint8_t Param[12];
     Param[0] = 0;                                 //
     Param[1] = 4;                                 // [bits] preamble length
     Param[2] = 0x04;                              // preamble detect: 0x00:OFF, 0x04:8bits, 0x05:16bits, 0x06=24bits, 0x07=32bits
     Param[3] = WriteSize*8;                       // [bits] SYNC word length, write word at 0x06C0
     Param[4] = 0x00;                              // address filtering: OFF
     Param[5] = 0x00;                              // fixed packet size
     Param[6] = 2*26;                              // 26 bytes, software Manchester
     Param[7] = 0x01;                              // no CRC
     Param[8] = 0x00;                              // no whitening
     Cmd_Write(CMD_SETPACKETPARAMS, Param, 9);     // 0x8C, PacketParam
     Regs_Write(REG_SYNCWORD0, SyncData+(8-WriteSize), WriteSize); } // Write the SYNC word

   static void Pack3bytes(uint8_t *Byte, uint32_t Value) { Byte[0]=Value>>16; Byte[1]=Value>>8; Byte[2]=Value; }

   void FNT_Configure(uint8_t CR=1)                   // configure for FANET/LoRa
   { WriteTxPower(0);
     RFM_LoRa_Config CFG = RFM_FNTcfg; CFG.CR=CR;
     LoRa_Configure(CFG, FANET_Packet::MaxBytes); }

   void LoRa_Configure(RFM_LoRa_Config CFG, uint8_t MaxSize=64)
   { setChannel(0);
     uint8_t Param[8];
     Param[0] = CFG.SF;                            // Spreading Factor
     Param[1] = CFG.BW-3;                          // work only for 62.5/125/256/512kHz
     Param[2] = CFG.CR;                            // Coding Rate
     Param[3] = CFG.LowRate;                       // Low-Rate optimize
     Cmd_Write(CMD_SETMODULATIONPARAMS, Param, 4); // 0x8B, ModParam
     Param[0] = 0x00;
     Param[1] = CFG.Preamble;                      // preamble size
     Param[2] = CFG.IHDR;                          // implicit header
     Param[3] = MaxSize;                           // [bytes]
     Param[4] = CFG.CRC;                           // check or not CRC
     Param[5] = CFG.InvIQ;                         // common flag for Tx/Rx
     Cmd_Write(CMD_SETPACKETPARAMS, Param, 6);     // 0x8C, PacketParam
     setDioMode( /* IRQ_TXDONE | */ IRQ_RXDONE);
     Param[0] = (CFG.SYNC&0xF0) | 0x04;
     Param[1] = (CFG.SYNC<<4)   | 0x04;
     Regs_Write(REG_LORASYNCWORD, Param, 2); }

   void OGN_Configure(int16_t Channel, const uint8_t *SyncData)
   { setChannel(Channel);
     uint8_t Param[12];
     Pack3bytes(Param, 10240);                     // data bitrate = 32*Xtal/100e3 for OGN 100kbps
     Param[3] = 0x09;                              // 0x00:no filter, 0x08:BT=0.3, 0x09:BT=0.5, 0x0A:BT=0.7, 0x0B:BT=1.0
     Param[4] = 0x0A;                              // DSB RX bandwidth: 0x0A=232.3kHz, 0x19=312.2kHz, 0x1B=78.2kHz, 0x13=117.3kHz
     Pack3bytes(Param+5, 52429);                   // FSK deviation: 50e3*2^25/Xtal for OGN +/-50kHz
     Cmd_Write(CMD_SETMODULATIONPARAMS, Param, 8); // 0x8B, ModParam
     Param[0] = 0;                                 //
     Param[1] = 4;                                 // [bits] preamble length
     Param[2] = 0x00;                              // preamble detect: 0x00:OFF, 0x04:8bits, 0x05:16bits, 0x06=24bits, 0x07=32bits
     Param[3] = 8*8;                               // [bits] SYNC word length, write word at 0x06C0
     Param[4] = 0x00;                              // address filtering: OFF
     Param[5] = 0x00;                              // fixed packet size
     Param[6] = 2*26;                              // 26 bytes, software Manchester
     Param[7] = 0x01;                              // no CRC
     Param[8] = 0x00;                              // no whitening
     Cmd_Write(CMD_SETPACKETPARAMS, Param, 9);     // 0x8C, PacketParam
     setDioMode(/* IRQ_TXDONE | */ IRQ_RXDONE);
     Regs_Write(REG_SYNCWORD0, SyncData, 8); }     // Write the SYNC word

   void PAW_Configure(const uint8_t *Sync)
   { setFrequency(869525000);
     uint8_t Param[12];
     Pack3bytes(Param, 26667);                     // data bitrate = 32*Xtal/38.4e3 for PAW 38.4kbps
     Param[3] = 0x09;                              // 0x00:no filter, 0x08:BT=0.3, 0x09:BT=0.5, 0x0A:BT=0.7, 0x0B:BT=1.0
     Param[4] = 0x18;                              // DSB RX bandwidth: 0x0A=232.3kHz, 0x19=312.2kHz, 0x1B=78.2kHz, 0x13=117.3kHz
     Pack3bytes(Param+5, 10066);                   // FSK deviation: 9.6e3*2^25/Xtal for PAW +/-9.6kHz
     Cmd_Write(CMD_SETMODULATIONPARAMS, Param, 8); // 0x8B, ModParam
     Param[0] = 0;                                 //
     Param[1] = 10*8;                              // [bits] preamble length
     Param[2] = 0x06;                              // preamble detect: 0x00:OFF, 0x04:8bits, 0x05:16bits, 0x06=24bits, 0x07=32bits
     Param[3] = 8*8;                               // [bits] SYNC word length, write word at 0x06C0
     Param[4] = 0x00;                              // address filtering: OFF
     Param[5] = 0x00;                              // fixed packet size
     Param[6] = 25;                                // 25 bytes
     Param[7] = 0x01;                              // no CRC
     Param[8] = 0x00;                              // no whitening
     Cmd_Write(CMD_SETPACKETPARAMS, Param, 9);     // 0x8C, PacketParam
     setDioMode( /* IRQ_TXDONE | */ IRQ_RXDONE);
     Regs_Write(REG_SYNCWORD0, Sync, 8); }         // Write the SYNC word

   void ClearIrqFlags(uint16_t Mask=IRQ_ALL) { uint8_t Data[2]; Data[0]=Mask>>8; Data[1]=Mask; Cmd_Write(CMD_CLEARIRQSTATUS, Data, 2); }
   uint16_t ReadIrqFlags(void) { uint8_t *Stat=Cmd_Read(CMD_GETIRQSTATUS, 2); return (((uint16_t)(Stat[0]))<<8) | Stat[1]; }

   void setModeSleep(void) { uint8_t Config[3] = { 0, 0, 0 }; Cmd_Write(CMD_SETSLEEP, Config, 3); }
   void setModeStandby(uint8_t Mode=0) { uint8_t Config[1] = { Mode }; Cmd_Write(CMD_SETSTANDBY, Config, 1); }
   void setModeTX(uint32_t Timeout=0) { uint8_t T[3]; Pack3bytes(T, Timeout); Cmd_Write(CMD_SETTX, T, 3); }
   void setModeRX(uint32_t Timeout=0) { uint8_t T[3]; Pack3bytes(T, Timeout); Cmd_Write(CMD_SETRX, T, 3); }
   void setModeSynth(void) { Cmd_Write(CMD_SETFS, 0, 0); }

   void setDioMode(uint16_t Mask = IRQ_RXDONE)
   { uint8_t Param[8];
     Param[0] = Mask>>8;   // IRQ mask
     Param[1] = Mask;
     Param[2] = Mask>>8;   // DIO1 mask
     Param[3] = Mask;
     Param[4] = 0;         // DIO2 mask
     Param[5] = 0;
     Param[6] = 0;         // DIO3 mask
     Param[7] = 0;
     Cmd_Write(CMD_SETDIOIRQPARAMS, Param, 8); }
#endif

#if defined(WITH_RFM95) || defined(WITH_SX1272) || defined(WITH_RFM69)
   void    WriteMode(uint8_t Mode=RF_OPMODE_STANDBY) { WriteByte(Mode, REG_OPMODE); } // SLEEP/STDBY/FSYNTH/TX/RX
   uint8_t ReadMode (void) { return ReadByte(REG_OPMODE); }
   uint8_t ModeReady(void) { return ReadByte(REG_IRQFLAGS1)&0x80; }

   uint16_t ReadIrqFlags(void) { return ReadWord(REG_IRQFLAGS1); }

   void ClearIrqFlags(void)    { WriteWord(RF_IRQ_FifoOverrun | RF_IRQ_Rssi | RF_IRQ_PreambleDetect | RF_IRQ_SyncAddrMatch, REG_IRQFLAGS1); }

   void setModeSleep(void)   { WriteMode(RF_OPMODE_SLEEP); }                // FSK sleep
   void setModeStandby(void) { WriteMode(RF_OPMODE_STANDBY); }              // FSK standby
   void setModeTX(void)      { WriteMode(RF_OPMODE_TRANSMITTER); }          // FSK transmit
   bool  isModeTX(void)      { return ReadMode()==RF_OPMODE_TRANSMITTER; }  // in transmitter mode ?
   void setModeRX(void)      { WriteMode(RF_OPMODE_RECEIVER); }             // FSK receive
   bool  isModeRX(void)      { return ReadMode()==RF_OPMODE_RECEIVER; }     // in receiver mode ? ?
#if defined(WITH_RFM95) || defined(WITH_SX1272)
   void setModeLoRaStandby(void)  { WriteMode(RF_OPMODE_LORA_STANDBY); }    // LoRa standby
   void setModeLoRaRXcont(void)   { WriteMode(RF_OPMODE_LORA_RX_CONT); }    // Lora continues recieve
   void setModeLoRaRXsingle(void) { WriteMode(RF_OPMODE_LORA_RX_SINGLE); }  // LoRa single receive
   bool  isModeLoRaTX(void)       { return ReadMode()==RF_OPMODE_LORA_TX; } // LoRa still transmitting ?
#endif
#endif

#ifdef WITH_RFM69
   void WriteTxPower_W(int8_t TxPower=10)       // [dBm] for RFM69W: -18..+13dBm
   { if(TxPower<(-18)) TxPower=(-18);           // check limits
     if(TxPower>  13 ) TxPower=  13 ;
     WriteByte(  0x80+(18+TxPower), REG_PALEVEL);
     WriteByte(  0x1A             , REG_OCP);
     WriteByte(  0x55             , REG_TESTPA1);
     WriteByte(  0x70             , REG_TESTPA2);
   }

   void WriteTxPower_HW(int8_t TxPower=10)       // [dBm] // for RFM69HW: -14..+20dBm
   { if(TxPower<(-14)) TxPower=(-14);            // check limits
     if(TxPower>  20 ) TxPower=  20 ;
     if(TxPower<=17)
     { WriteByte(  0x60+(14+TxPower), REG_PALEVEL);
       WriteByte(  0x1A             , REG_OCP);
       WriteByte(  0x55             , REG_TESTPA1);
       WriteByte(  0x70             , REG_TESTPA2);
     } else
     { WriteByte(  0x60+(11+TxPower), REG_PALEVEL);
       WriteByte(  0x0F             , REG_OCP);
       WriteByte(  0x5D             , REG_TESTPA1);
       WriteByte(  0x7C             , REG_TESTPA2);
     }
   }

   void WriteTxPower(int8_t TxPower, bool isHW)
   { WriteByte(  0x09, REG_PARAMP); // Tx ramp up/down time 0x06=100us, 0x09=40us, 0x0C=20us, 0x0F=10us (page 66)
     if(isHW) WriteTxPower_HW(TxPower);
         else WriteTxPower_W (TxPower);  }

   void WriteTxPowerMin(void) { WriteTxPower_W(-18); } // set minimal Tx power and setup for reception

   void OGN_Configure(int16_t Channel, const uint8_t *Sync)
   { WriteMode(RF_OPMODE_STANDBY);          // mode = STDBY
     ClearIrqFlags();
     WriteByte(  0x02, REG_DATAMODUL);      // [0x00] Packet mode, FSK, 0x02: BT=0.5, 0x01: BT=1.0, 0x03: BT=0.3
     WriteWord(0x0140, REG_BITRATEMSB);     // bit rate = 100kbps
     WriteWord(0x0333, REG_FDEVMSB);        // FSK deviation = +/-50kHz
     setChannel(Channel);                   // operating channel
     FSK_WriteSYNC(8, 7, Sync);             // SYNC pattern (setup for transmission)
     WriteByte(  0x00, REG_PACKETCONFIG1);  // [0x10] Fixed size packet, no DC-free encoding, no CRC, no address filtering
     WriteByte(0x80+51, REG_FIFOTHRESH);    // [ ] TxStartCondition=FifoNotEmpty, FIFO threshold = 51 bytes
     WriteByte(  2*26, REG_PAYLOADLENGTH);  // [0x40] Packet size = 26 bytes Manchester encoded into 52 bytes
     WriteByte(  0x02, REG_PACKETCONFIG2);  // [0x02] disable encryption (it is permanent between resets !), AutoRxRestartOn=1
     WriteByte(  0x00, REG_AUTOMODES);      // [0x00] all "none"
     WriteTxPowerMin();                     // TxPower (setup for reception)
     WriteByte(  0x08, REG_LNA);            // [0x08/88] bit #7 = LNA input impedance: 0=50ohm or 1=200ohm ?
     WriteByte( 2*112, REG_RSSITHRESH);     // [0xE4] RSSI threshold = -112dBm
     WriteByte(  0x42, REG_RXBW);           // [0x86/55] +/-125kHz Rx bandwidth => p.27+67 (A=100kHz, 2=125kHz, 9=200kHz, 1=250kHz)
     WriteByte(  0x82, REG_AFCBW);          // [0x8A/8B] +/-125kHz Rx bandwidth while AFC
     WriteWord(0x4047, REG_DIOMAPPING1);    // DIO signals: DIO0=01, DIO4=01, ClkOut=OFF
                                            // RX: DIO0 = PayloadReady, DIO4 = Rssi
                                            // TX: DIO0 = TxReady,      DIO4 = TxReady
     WriteByte(  0x1B, REG_TESTLNA);        // [0x1B] 0x2D = LNA sensitivity up by 3dB, 0x1B = default
     WriteByte(  0x30, REG_TESTDAGC);       // [0x30] 0x20 when AfcLowBetaOn, 0x30 otherwise-> page 25
     WriteByte(  0x00, REG_AFCFEI);         // [0x00] AfcAutoOn=0, AfcAutoclearOn=0
     WriteByte(  0x00, REG_AFCCTRL);        // [0x00] 0x20 = AfcLowBetaOn=1 -> page 64 -> page 33
     WriteByte(   +10, REG_TESTAFC); }      // [0x00] [488Hz] if AfcLowBetaOn
#endif

// #ifdef WITH_RFM95
#if defined(WITH_RFM95) || defined(WITH_SX1272)

   void WriteTxPower(int8_t TxPower=0)
   { if(TxPower>17)
     { if(TxPower>20) TxPower=20;
       WriteByte(0x87, REG_PADAC);
       WriteByte(0xF0 | (TxPower-5), REG_PACONFIG); }
     else // if(TxPower>14)
     { if(TxPower<2) TxPower=2;
       WriteByte(0x84, REG_PADAC);
       WriteByte(0xF0 | (TxPower-2), REG_PACONFIG); }
     // else
     // { if(TxPower<0) TxPower=0;
     //   WriteByte(0x84, REG_PADAC);
     //   WriteByte(0x70 | (TxPower+1), REG_PACONFIG); }

     // if(TxPower<2) TxPower=2;
     // else if(TxPower>17) TxPower=17;
     // if(TxPower<=14)
     // { WriteByte(0x70 | TxPower    , REG_PACONFIG);
     // }
     // else
     // { WriteByte(0xF0 | (TxPower-2), REG_PACONFIG); }
   }

   void WriteTxPowerMin(void) { WriteTxPower(0); }

   void setLoRa(void)                            // switch to LoRa: has to go througth the SLEEP mode
   { WriteMode(RF_OPMODE_LORA_SLEEP);
     WriteMode(RF_OPMODE_LORA_SLEEP); }

   void setFSK(void)                             // switch to FSK: has to go through the SLEEP mode
   { WriteMode(RF_OPMODE_SLEEP);
     WriteMode(RF_OPMODE_SLEEP); }

   void LoRa_Configure(RFM_LoRa_Config CFG, uint8_t MaxSize=64)
   { WriteByte(0x00,   REG_LORA_HOPPING_PERIOD);                                // disable fast-hopping
     WriteByte(CFG.SYNC,   REG_LORA_SYNC);                                      // SYNC
     WriteWord(CFG.Preamble, REG_LORA_PREAMBLE_MSB);                            // [symbols] minimal preamble
     WriteByte((CFG.BW<<4) | (CFG.CR<<1) | CFG.IHDR, REG_LORA_MODEM_CONFIG1);   // 0x88 = 250kHz, 4+4, explicit header
     WriteByte((CFG.SF<<4) | (CFG.CRC<<2), REG_LORA_MODEM_CONFIG2);             // 0x74 = SF7, CRC on
     // WriteByte(CFG.InvIQ?0x67:0x26, REG_LORA_INVERT_IQ);
     // WriteByte(CFG.InvIQ?0x19:0x1D, REG_LORA_INVERT_IQ2);
     WriteByte((CFG.RxInv<<6) | 0x26 | CFG.TxInv, REG_LORA_INVERT_IQ);
     // Format_String(CONS_UART_Write, "REG_LORA_INVERT_IQ:");
     // Format_Hex(CONS_UART_Write, CFG.Word);
     // Format_String(CONS_UART_Write, ":");
     // Format_Hex(CONS_UART_Write, ReadByte(REG_LORA_INVERT_IQ));
     // Format_String(CONS_UART_Write, "\n");
     WriteByte(0xC3,   REG_LORA_DETECT_OPTIMIZE);
     WriteByte(0x0A,   REG_LORA_DETECT_THRESHOLD);
     WriteByte(0x04 | (CFG.LowRate<<3),   REG_LORA_MODEM_CONFIG3);  // LNA auto-gain and low-rate-optimize
     WriteByte(0xFF,   REG_LORA_SYMBOL_TIMEOUT);    //
     WriteByte(MaxSize,  REG_LORA_PACKET_MAXLEN);   // [bytes]
     WriteByte(0x00,   REG_LORA_RX_ADDR);
     setChannel(0);                                 // operating channel
     WriteWord(0x0000, REG_DIOMAPPING1);            // 001122334455___D signals: 00=DIO0 11=DIO1 22=DIO2 33=DIO3 44=DIO4 55=DIO5 D=MapPreambleDetect
                                                    // DIO0: 00=RxDone, 01=TxDone, 10=CadDone
   }

   void FNT_Configure(uint8_t CR=1)                   // configure for FANET/LoRa
   { WriteTxPower(0);
     RFM_LoRa_Config CFG = RFM_FNTcfg; CFG.CR=CR;
     LoRa_Configure(CFG, FANET_Packet::MaxBytes); }

   void WAN_Configure(uint8_t CR=1)                   // configure for LoRaWAN
   { WriteTxPower(0);
     RFM_LoRa_Config CFG = RFM_WANcfg; CFG.CR=CR;
     LoRa_Configure(CFG, 40); }

   void LoRa_setIRQ(uint8_t Mode=0)                  // 0:on RX, 1:on TX, 2: on CAD
   { WriteByte(Mode<<6, REG_DIOMAPPING1); }

   void LoRa_setCRC(bool ON=1)                       // LoRaWAN: uplink with CRC, downlink without CRC
   { uint8_t Reg=ReadByte(REG_LORA_MODEM_CONFIG2);
     if(ON) Reg|=0x04;
       else Reg&=0xFB;
     WriteByte(Reg, REG_LORA_MODEM_CONFIG2); }

   void LoRa_InvertIQ(bool ON=0)                     // LoRaWAN: uplink without inversion, downlink with inversion, but beacon without
   { WriteByte(ON?0x66:0x27, REG_LORA_INVERT_IQ);
     WriteByte(ON?0x19:0x1D, REG_LORA_INVERT_IQ2); }

   int LoRa_SendPacket(const uint8_t *Data, uint8_t Len, int Wait=0)
   { // WriteMode(RF_OPMODE_LORA_STANDBY);
     // check if FIFO empty, packets could be received ?
     WriteByte(0x00, REG_LORA_FIFO_ADDR);   // tell write to FIFO at address 0x00
     WriteFIFO(Data, Len);                  // write the packet data
     WriteByte(0x00, REG_LORA_TX_ADDR);     // tell packet address in the FIFO
     WriteByte(Len, REG_LORA_PACKET_LEN);   // tell packet length
     WriteMode(RF_OPMODE_LORA_TX);          // enter transmission mode
     if(Wait==0) return 0;
     Delay_ms(10); int Check=10;
     for(Check=0; Check<Wait; Check++)
     { Delay_ms(1);
       uint8_t Mode=ReadMode();
       if(Mode!=RF_OPMODE_LORA_TX) break; }
     return Check+1; }                      // afterwards just wait for TX mode to stop

   int FNT_SendPacket(const uint8_t *Data, uint8_t Len, int Wait=0)
   { return LoRa_SendPacket(Data, Len, Wait); }

  template<class RxPacket>
   int LoRa_ReceivePacket(RxPacket &Packet)
   { uint8_t Flags = ReadByte(REG_LORA_IRQ_FLAGS);
     if((Flags&LORA_FLAG_RX_DONE)==0) return 0;
     uint8_t Stat = ReadByte(REG_LORA_MODEM_STATUS);     // coding rate in three top bits
     uint8_t HopChan = ReadByte(REG_LORA_HOP_CHANNEL);
     Packet.CR    = Stat>>5;                             // coding rate used for this packet
     Packet.hasCRC = HopChan&0x40;                       // did this packet have CRC ? (flags should be checked for CRC error)
     Packet.badCRC = Flags&LORA_FLAG_BAD_CRC;
     Packet.SNR   = ReadByte(REG_LORA_PACKET_SNR);       // [0.25dB] read SNR
     Packet.RSSI = -157+ReadByte(REG_LORA_PACKET_RSSI);  // [dBm] read RSSI
     int32_t FreqOfs = ReadFreq(REG_LORA_FREQ_ERR_MSB);  //
     if(FreqOfs&0x00080000) FreqOfs|=0xFFF00000;         // extend the sign bit
                       else FreqOfs&=0x000FFFFF;
     Packet.FreqOfs = (FreqOfs*1718+0x8000)>>16;         //  [10Hz]
     Packet.BitErr  = 0;
     Packet.CodeErr = 0;
     int Len=LoRa_ReceivePacket(Packet.Byte, Packet.MaxBytes); // read packet data
     // printf("ReceivePacketFNT() => %d %02X %3.1fdB %+ddBm 0x%08X=%+6.3fkHz, %02X%02X%02X%02X\n",
     //       Packet.Len, Stat, 0.25*Packet.SNR, Packet.RSSI, FreqOfs, 0.5*0x1000000/32e9*FreqOfs,
     //       Packet.Byte[0], Packet.Byte[1], Packet.Byte[2], Packet.Byte[3]);
     Packet.Len=Len;
     WriteByte(LORA_FLAG_RX_DONE | LORA_FLAG_BAD_CRC | LORA_FLAG_RX_HEADER, REG_LORA_IRQ_FLAGS);
     return Len; }

   int LoRa_ReceivePacket(uint8_t *Data, uint8_t MaxLen)
   { uint8_t Len=ReadByte(REG_LORA_PACKET_BYTES);    // packet length
     uint8_t Ptr=ReadByte(REG_LORA_PACKET_ADDR);     // packet address in FIFO
     WriteByte(Ptr, REG_LORA_FIFO_ADDR);             // ask to read FIFO from this address
     // uint8_t Stat = ReadByte(REG_LORA_MODEM_STATUS); //
     //  int8_t SNR  = ReadByte(REG_LORA_PACKET_SNR);   // [0.25dB] read SNR
     //  int8_t RSSI = ReadByte(REG_LORA_PACKET_RSSI);  // [dBm] read RSSI
     // int32_t FreqOfs = ReadFreq(REG_LORA_FREQ_ERR_MSB); // (FreqOfs*1718+0x8000)>>16 [10Hz]
     // if(FreqOfs&0x00080000) FreqOfs|=0xFFF00000;     // extend the sign bit
     //                   else FreqOfs&=0x000FFFFF;
     uint8_t *ReadData = ReadFIFO(Len);              // read data from FIFO
     memcpy(Data, ReadData, Len);
     // printf("ReceivePacketFNT( , %d) => %d [%02X] %02X %3.1fdB %+ddBm 0x%08X=%+6.3fkHz, %02X%02X%02X%02X\n",
     //       MaxLen, Len, Ptr, Stat, 0.25*SNR, -157+RSSI, FreqOfs, 0.5*0x1000000/32e9*FreqOfs,
     //       ReadData[0], ReadData[1], ReadData[2], ReadData[3]);
     return Len; }

   int PAW_Configure(const uint8_t *Sync)
   { // WriteMode(RF_OPMODE_STANDBY);
     WriteTxPower(0);
     ClearIrqFlags();
     WriteWord(0x0341, REG_BITRATEMSB);         // bit rate = 0x0341 = 38.415kbps
     WriteByte(  0x05, REG_BITRATEFRAC);        // one should set exactly 38.400kbps for PW
     WriteWord(   157, REG_FDEVMSB);            // FSK deviation = 0x013B x Fstep = 19.226kHz, Fstep=32MHz/(1<<19);
     setFrequency(869525000);                   // 869.525MHz
     FSK_WriteSYNC(8, 7, Sync);                 // SYNC pattern
     WriteWord(    10, REG_PREAMBLEMSB);        // 10 preamble bytes
     WriteByte(  0x85, REG_PREAMBLEDETECT);     // preamble detect: 1 byte, page 92 (or 0x85 ?)
     WriteByte(  0x00, REG_PACKETCONFIG1);      // Fixed size packet, no DC-free encoding, no CRC, no address filtering
     WriteByte(  0x40, REG_PACKETCONFIG2);      // Packet mode
     WriteByte(    25, REG_PAYLOADLENGTH);      // Packet size = 25 bytes
     WriteByte(    24, REG_FIFOTHRESH);         // TxStartCondition=FifoNotEmpty, FIFO threshold = 24 bytes
     WriteWord(0x3030, REG_DIOMAPPING1);        // DIO signals: DIO0=00, DIO1=11, DIO2=00, DIO3=00, DIO4=00, DIO5=11, => p.64, 99
     // WriteByte(  0x02, REG_RXBW);               // Man=0=16 Exp=2 +/-125kHz Rx (single-side) bandwidth => p.27,67,83,90
     WriteByte(  0x23, REG_RXBW);               // Man=1=20 Exp=3 +/-50kHz Rx (single-side) bandwidth => p.27,67,83,90
     WriteByte(  0x23, REG_AFCBW);              // +/-125kHz AFC bandwidth
     WriteByte(  0x29, REG_PARAMP);             // 2:BT=1.0 (seems better ?), 4:BT=0.5 shaping, 40us ramp up/down
     WriteByte(  0x0E, REG_RXCONFIG);           // => p.90 (or 0x8E ?)
     WriteByte(  0x07, REG_RSSICONFIG);         // 256 samples for RSSI, no offset, => p.90,82
     WriteByte(  0x20, REG_LNA);                // max. LNA gain, => p.89

     return 0; }

   void OGN_Configure(int16_t Channel, const uint8_t *Sync)
   { // WriteMode(RF_OPMODE_STANDBY);              // mode: STDBY, modulation: FSK, no LoRa
     // usleep(1000);
     WriteTxPower(0);
     ClearIrqFlags();
     WriteWord(0x0140, REG_BITRATEMSB);         // bit rate = 100kbps (32MHz/100000)
     WriteByte(0x00, REG_BITRATEFRAC);          //
     // ReadWord(REG_BITRATEMSB);
     WriteWord(0x0333, REG_FDEVMSB);            // FSK deviation = +/-50kHz [32MHz/(1<<19)]
     // ReadWord(REG_FDEVMSB);
     setChannel(Channel);                       // operating channel
     FSK_WriteSYNC(8, 7, Sync);                 // SYNC pattern (setup for reception)
     WriteByte(  0x85, REG_PREAMBLEDETECT);     // preamble detect: 1 byte, page 92 (or 0x85 ?)
     WriteByte(  0x00, REG_PACKETCONFIG1);      // Fixed size packet, no DC-free encoding, no CRC, no address filtering
     WriteByte(  0x40, REG_PACKETCONFIG2);      // Packet mode
     WriteByte(  2*26, REG_PAYLOADLENGTH);      // Packet size = 26 bytes Manchester encoded into 52 bytes
     WriteByte(    51, REG_FIFOTHRESH);         // TxStartCondition=FifoNotEmpty, FIFO threshold = 51 bytes
     WriteWord(0x3030, REG_DIOMAPPING1);        // DIO signals: DIO0=00, DIO1=11, DIO2=00, DIO3=00, DIO4=00, DIO5=11, => p.64, 99
     WriteByte(  0x02, REG_RXBW);               // +/-125kHz Rx (single-side) bandwidth => p.27,67,83,90
     WriteByte(  0x02, REG_AFCBW);              // +/-125kHz AFC bandwidth
     WriteByte(  0x49, REG_PARAMP);             // BT=0.5 shaping, 40us ramp up/down
     WriteByte(  0x0E, REG_RXCONFIG);           // => p.90 (or 0x8E ?)
     WriteByte(  0x07, REG_RSSICONFIG);         // 256 samples for RSSI, no offset, => p.90,82
     WriteByte(  0x20, REG_LNA); }              // max. LNA gain, => p.89

   uint8_t ReadLowBat(void)  { return ReadByte(REG_LOWBAT ); }

  void PrintReg(void (*CONS_UART_Write)(char))
  { Format_String(CONS_UART_Write, "RFM95 Mode:");
    uint8_t RxMode=ReadMode();
    Format_Hex(CONS_UART_Write, RxMode);
    CONS_UART_Write(' '); CONS_UART_Write('0'+DIO0_isOn());
    Format_String(CONS_UART_Write, " IRQ:");
    Format_Hex(CONS_UART_Write, ReadWord(REG_IRQFLAGS1));
    Format_String(CONS_UART_Write, " Pre:");
    Format_Hex(CONS_UART_Write, ReadWord(REG_PREAMBLEMSB));
    Format_String(CONS_UART_Write, " SYNC:");
    Format_Hex(CONS_UART_Write, ReadByte(REG_SYNCCONFIG));
    CONS_UART_Write('/');
    for(uint8_t Idx=0; Idx<8; Idx++)
      Format_Hex(CONS_UART_Write, ReadByte(REG_SYNCVALUE1+Idx));
    Format_String(CONS_UART_Write, " FREQ:");
    Format_Hex(CONS_UART_Write, ReadByte(REG_FRFMSB));
    Format_Hex(CONS_UART_Write, ReadByte(REG_FRFMID));
    Format_Hex(CONS_UART_Write, ReadByte(REG_FRFLSB));
    Format_String(CONS_UART_Write, " RATE:");
    Format_Hex(CONS_UART_Write, ReadWord(REG_BITRATEMSB));
    Format_String(CONS_UART_Write, " FDEV:");
    Format_Hex(CONS_UART_Write, ReadWord(REG_FDEVMSB));
    Format_String(CONS_UART_Write, " DIO:");
    Format_Hex(CONS_UART_Write, ReadWord(REG_DIOMAPPING1));
    Format_String(CONS_UART_Write, " CFG:");
    Format_Hex(CONS_UART_Write, ReadByte(REG_PREAMBLEDETECT));
    Format_Hex(CONS_UART_Write, ReadByte(REG_PACKETCONFIG1));
    Format_Hex(CONS_UART_Write, ReadByte(REG_PACKETCONFIG2));
    Format_Hex(CONS_UART_Write, ReadByte(REG_FIFOTHRESH));
    Format_Hex(CONS_UART_Write, ReadByte(REG_PAYLOADLENGTH));
    Format_Hex(CONS_UART_Write, ReadByte(REG_RXBW));
    Format_Hex(CONS_UART_Write, ReadByte(REG_RSSICONFIG));
    Format_String(CONS_UART_Write, " PA:");
    Format_Hex(CONS_UART_Write, ReadByte(REG_PARAMP));
    Format_Hex(CONS_UART_Write, ReadByte(REG_PACONFIG));
    Format_String(CONS_UART_Write, "\n"); }

#endif // WITH_RFM95

#if defined(WITH_RFM95) || defined(WITH_SX1272) || defined(WITH_RFM69)
     uint8_t ReadVersion(void) { chipVer=ReadByte(REG_VERSION); return chipVer; } // 0x24 for RFM69 or 0x12 for RFM95
     uint8_t ReadRSSI(void)    { return ReadByte(REG_RSSIVALUE); }         // read value: RSS = -Value/2
#endif
#ifdef WITH_SX1262
     uint8_t ReadVersion(void) { return 0x12; }
     uint8_t ReadRSSI(void) { uint8_t *RSSI=Cmd_Read(CMD_GETRSSIINST, 1); return RSSI[0]; }
#endif

#ifdef WITH_RFM69
     void    TriggerRSSI(void) { WriteByte(0x01, REG_RSSICONFIG); }        // trigger measurement
     uint8_t ReadyRSSI(void)   { return ReadByte(REG_RSSICONFIG) & 0x02; } // ready ?
     void    TriggerTemp(void) { WriteByte(0x08, REG_TEMP1); }             // trigger measurement
     uint8_t RunningTemp(void) { return ReadByte(REG_TEMP1) & 0x04; }      // still running ?
     int8_t ReadTemp(void)     { chipTemp=165-ReadByte(REG_TEMP2); return chipTemp; } // [deg]
#endif

#if defined(WITH_RFM95) || defined(WITH_SX1272)
     int8_t ReadTemp(void)     { chipTemp = 15-ReadByte(REG_TEMP); return chipTemp; } // [degC]
#endif
} ;

#endif // defined(WITH_RFM69) || defined(WITH_RFM95) || defined(WITH_SX1272) || defined(WITH_SX1262)

#endif // __RFM_H__

