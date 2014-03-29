#include "Packet_Serial_Test.h"

void PACKETS_OneArg(byte bCmd, byte bArg) {
    byte bCRC = (PACKETS_ADDRESS + bCmd + bArg) & 0x7f;    // calculate the checksum 
    PacketSerial.write(PACKETS_ADDRESS);
    PacketSerial.write(bCmd);
    PacketSerial.write(bArg);
    PacketSerial.write(bCRC);
}

inline void PACKETS_DriveForwardM1(byte bVal) {
    PACKETS_OneArg(0, bVal);
}

inline void PACKETS_DriveBackwardsM1(byte bVal) {
    PACKETS_OneArg(1, bVal);
}

inline void PACKETS_DriveForwardM2(byte bVal) {
    PACKETS_OneArg(4, bVal);
}

inline void PACKETS_DriveBackwardsM2(byte bVal) {
    PACKETS_OneArg(5, bVal);
}

void setup (void) {
	PacketSerial.begin(38400);
}

void loop (void) {
  PACKETS_DriveForwardM1(15);	
  PACKETS_DriveForwardM1(15);

  PACKETS_DriveBackwardsM1(15);	
  PACKETS_DriveBackwardsM1(15);

  PACKETS_DriveForwardM1(0);	
  PACKETS_DriveForwardM1(0);
}
