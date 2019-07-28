/*
  MPU6050Sensor.cpp - Library for flashing MPU6050Sensor code.
  Created by Minu Suh, December 3, 2016.
*/

#include "Arduino.h"
#include "MPU6050Sensor.h"
#include <Wire.h>

MPU6050Sensor::MPU6050Sensor()
{	
}

void MPU6050Sensor::begin(int MPU_addr)
{
	_MPU_addr = MPU_addr;
	bAngleRead = false;
	bGyroRead = false;
	
	initMPU6050();
	calibAccelGyro();
	initDT();
}

void MPU6050Sensor::read_rpy_angle(float &roll, float &pitch, float &yaw)
{
	if(!bGyroRead) {
		readAccelGyro();
		calcDT();
		calcAccelYPR();  
		calcGyroYPR();    
		calcFilteredYPR();
		
		bAngleRead = true;
	}
  
	roll = filtered_angle_y;
	pitch = filtered_angle_x;
	yaw = filtered_angle_z;
}

void MPU6050Sensor::read_rpy_angular_velocity(float &roll, float &pitch, float &yaw)
{
	if(!bAngleRead) {
		readAccelGyro();
		calcDT();
		calcAccelYPR();  
		calcGyroYPR();    
		calcFilteredYPR();
		
		bGyroRead = true;
	}
	
	roll = gyro_y;
	pitch = gyro_x;
	yaw = gyro_z;  
}

void MPU6050Sensor::initMPU6050() {
  Wire.begin();
  Wire.beginTransmission(_MPU_addr);
  Wire.write(0x6B);  
  Wire.write(0);     
  Wire.endTransmission(true);
}

void MPU6050Sensor::readAccelGyro() {
  Wire.beginTransmission(_MPU_addr);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(_MPU_addr,14,true);  
  AcX=Wire.read()<<8|Wire.read();  
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  Tmp=Wire.read()<<8|Wire.read();  
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();    
}

void MPU6050Sensor::calibAccelGyro() {
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0; 

  readAccelGyro();
  
  for(int i=0;i<10;i++) {
    readAccelGyro();
    sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
    sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
    delay(100);
  }

  baseAcX = sumAcX / 10; 
  baseAcY = sumAcY / 10; 
  baseAcZ = sumAcZ / 10; 
  
  baseGyX = sumGyX / 10; 
  baseGyY = sumGyY / 10; 
  baseGyZ = sumGyZ / 10;   
}

void MPU6050Sensor::initDT() {
  t_prev = micros();
}

void MPU6050Sensor::calcDT() {
  t_now = micros();
  _dt = (t_now - t_prev)/1000000.0;
  t_prev = t_now;  
}

void MPU6050Sensor::calcAccelYPR() {
  float accel_x, accel_y, accel_z;
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180/3.14159;
  
  accel_x = AcX - baseAcX;
  accel_y = AcY - baseAcY;
  accel_z = AcZ + (16384 - baseAcZ);

  accel_yz = sqrt(pow(accel_y,2) + pow(accel_z,2));
  accel_angle_y = atan(-accel_x/accel_yz)*RADIANS_TO_DEGREES;

  accel_xz = sqrt(pow(accel_x,2) + pow(accel_z,2));
  accel_angle_x = atan( accel_y/accel_xz)*RADIANS_TO_DEGREES;
    
  accel_angle_z = 0;
}

void MPU6050Sensor::calcGyroYPR() {  
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131;
  
  gyro_x = (GyX - baseGyX)/GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_y = (GyY - baseGyY)/GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_z = (GyZ - baseGyZ)/GYROXYZ_TO_DEGREES_PER_SEC;
}

void MPU6050Sensor::calcFilteredYPR() {
  const float ALPHA = 0.96;
  float tmp_angle_x, tmp_angle_y, tmp_angle_z;

  tmp_angle_x = filtered_angle_x + gyro_x * _dt;
  tmp_angle_y = filtered_angle_y + gyro_y * _dt;
  tmp_angle_z = filtered_angle_z + gyro_z * _dt;
  
  filtered_angle_x = 
    ALPHA * tmp_angle_x + (1.0-ALPHA) * accel_angle_x;    
  filtered_angle_y = 
    ALPHA * tmp_angle_y + (1.0-ALPHA) * accel_angle_y;    
  filtered_angle_z = tmp_angle_z;
}