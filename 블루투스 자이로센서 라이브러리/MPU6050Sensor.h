/*
  MPU6050Sensor.h - Library for flashing MPU6050Sensor code.
  Created by Minu Suh, December 3, 2016.
*/
#ifndef MPU6050Sensor_h
#define MPU6050Sensor_h

#include "Arduino.h"

class MPU6050Sensor
{
  public:
    MPU6050Sensor();
	void begin(int MPU_addr);
    void read_rpy_angle(float &roll, float &pitch, float &yaw);	
	void read_rpy_angular_velocity(float &roll, float &pitch, float &yaw);
	
  private:
	void initMPU6050();
	void readAccelGyro();
	void calibAccelGyro();
	void initDT();
	void calcDT();
	void calcAccelYPR();
	void calcGyroYPR();
	void calcFilteredYPR();
	
  private:
	int _MPU_addr;  
	int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
    float baseAcX, baseAcY, baseAcZ;
	float baseGyX, baseGyY, baseGyZ ;
	unsigned long t_now, t_prev;
	float _dt;
	float accel_angle_x, accel_angle_y, accel_angle_z;
	float gyro_x, gyro_y, gyro_z;	
	float filtered_angle_x, filtered_angle_y, filtered_angle_z;
	bool bAngleRead, bGyroRead;
};

#endif