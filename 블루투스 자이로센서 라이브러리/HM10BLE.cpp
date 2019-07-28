/*
  HM10BLE.cpp - Library for flashing HM10BLE code.
  Created by Minu Suh, December 3, 2016.
*/

#include "Arduino.h"
#include "HM10BLE.h"

HM10BLE::HM10BLE()
{  
} 

#ifdef ARDUINO_AVR_LEONARDO
#define Serial4Ble Serial1
#elif defined(ARDUINO_AVR_UNO)
#define Serial4Ble Serial
#endif

void HM10BLE::begin(unsigned long baud)
{
	Serial4Ble.begin(baud);	
} 
	
void HM10BLE::receive_user_rpyt(int &roll, int &pitch, int &yaw, int &throttle)
{
	checkMspPacket(roll, pitch, yaw, throttle);  
}

void HM10BLE::checkMspPacket(int &roll, int &pitch, int &yaw, int &throttle) {  
  static uint32_t cnt;
  static int _roll=125, _pitch=125, _yaw=125, _throttle=0;

  if(Serial4Ble.available() > 0) {
    while(Serial4Ble.available() > 0) {
      int mspData = Serial4Ble.read(); 
      if(mspData == '$') cnt = HEAD1;
      else cnt++;
 
      mspPacket[cnt] = mspData;
 
      if(cnt == CRC) {
        if(mspPacket[CMD] == 150) {
          _throttle = mspPacket[THROTTLE];
		  _roll = mspPacket[ROLL];
		  _pitch = mspPacket[PITCH];
		  _yaw = mspPacket[YAW];          
        }
      }
    }
  }
  roll = _roll;
  pitch = _pitch;
  yaw = _yaw;
  throttle = _throttle;
}