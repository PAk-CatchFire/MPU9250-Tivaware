/*
 * MPU9250.h
 *
 *  Created on: August 24, 2016
 *      Author: PAk
 *      https://e2e.ti.com/members/521000
 */
#ifndef _MPU9250_H
#define _MPU9250_H

//Otra LIB
#define RAD_TO_DEG 		57.29577
#define KALMAN_PERIOD	0.01		//seconds
#define COMPLIMENT_COEFFICIENT 0.98684

#define ROLL_SET 2
#define PITCH_SET -1
#define YAW_SET 0
//////////////////////////////

#define SerialDebug true   // set to true to get Serial output for debugging

int accelCalibData[3];
float magnetAdjData[3];
float SelfTest[6];
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values

void ConfigMPU9250();
void CalibrateMPU9250(float * dest1, float * dest2);
void MPU9250SelfTest(float * destination) ;
void ConfigAK8963(float * destination);
int16_t ReadTempData();
void ReadMagData(int16_t * destination);
void ReadGyroData(int16_t * destination);
void ReadAccelData(int16_t * destination);
void ResetMPU9250();
void Setup();
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void UpdateData();
void loop();

void GetAres();
void GetGres();
void GetMres();

void delayMS(int ms) ;

void delay_uS(int us) ;

extern float pfData[16];
extern float *pfAccel, *pfGyro, *pfMag, *pfEulers, *pfQuaternion;

#define PRINT_SKIP_COUNT        2

#endif
