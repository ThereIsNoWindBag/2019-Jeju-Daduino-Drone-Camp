/*
  HM10BLE.h - Library for flashing HM10BLE code.
  Created by Minu Suh, December 3, 2016.  
*/
#ifndef HM10BLE_h
#define HM10BLE_h

#include "Arduino.h"

enum {  
  HEAD1,  HEAD2,  HEAD3,  DATASIZE, CMD, 
  ROLL,   PITCH,  YAW,    THROTTLE,
  AUX,    CRC,    PACKETSIZE,
};

class HM10BLE
{
  public:
    HM10BLE();
	void begin(unsigned long baud);
	void receive_user_rpyt(int &roll, int &pitch, int &yaw, int &throttle);
	
  private:
	void checkMspPacket(int &roll, int &pitch, int &yaw, int &throttle);
    
  private:
    int mspPacket[PACKETSIZE];
};

#endif