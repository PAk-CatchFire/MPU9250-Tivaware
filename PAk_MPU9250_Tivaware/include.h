#ifndef INCLUDE_H_
#define INCLUDE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/fpu.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/systick.h"
#include "driverlib/debug.h"
#include "utils/uartstdio.h"
#include "inc/hw_i2c.h"
#include <inc/hw_ints.h>
#include <inc/hw_nvic.h>
#include <inc/hw_types.h>
#include <math.h>


#include "I2C/I2C_pak.h"
//#include "MPU9250/MPU9250.h"
#include "MPU9250/MPU9250_reg.h"



//#include "I2C/I2C_pak.h"
//#include "MPU9150/MPU9150.h"
#include "MPU9150/MPU9150_reg.h"


#define DWT_O_CYCCNT 				0x00000004

extern float pfData[16];
extern float *pfAccel, *pfGyro, *pfMag, *pfEulers, *pfQuaternion;

extern int_fast32_t i32IPart[16], i32FPart[16];

//*****************************************************************************
//
// Global counter to control and slow down the rate of data to the terminal.
//
//*****************************************************************************
//#define PRINT_SKIP_COUNT        4
extern uint32_t g_ui32PrintSkipCounter;

extern uint32_t cnt_IMU_calc;
extern uint32_t cnt_IMU_calc_reales;
extern uint32_t cnt_msg_IMU;

//#define sampleFreq	512.0f		// sample frequency in Hz

extern void delayMS(int ms);

extern void delay_uS(int us);
// Set initial input parameters
//extern enum Ascale;
//extern enum Gscale;
//extern enum Mscale;

enum Ascale {
	AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G
};

enum Gscale {
	GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
};

enum Mscale {
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
};


extern void EnableBenchmarkingCode(void);

// Specify sensor full scale
extern uint8_t Gscale;
extern uint8_t Ascale;
extern uint8_t Mscale; // Choose either 14-bit or 16-bit magnetometer resolution
extern uint8_t Mmode; // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
extern float aRes, gRes, mRes; // scale resolutions per LSB for the sensors

extern int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
extern int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
extern int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
extern float magCalibration[3], magBias[3]; // Factory mag calibration and mag bias
extern float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
extern int16_t tempCount;      // temperature raw count output
extern float temperature; // Stores the real internal chip temperature in degrees Celsius
extern float SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define PI 3.141516
extern float GyroMeasError; // gyroscope measurement error in rads/s (start at 40 deg/s)
//float GyroMeasDrift = PI * (0.0  / 180.0);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
//float GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
extern float beta_p;   //// sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
extern float GyroMeasDrift; //PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
extern float zeta_p; //sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

extern float deltat, sum;   // integration interval for both filter schemes

// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.

extern uint32_t count, sumCount; // used to control display output rate

extern float q[4];    // vector to hold quaternion
extern float eInt[3]; // vector to hold integral error for Mahony method

extern float q0;
extern float q1;
extern float q2;
extern float q3;

extern float pitch, yaw, roll;


extern bool samplingFlag;
extern uint32_t cycles1;
extern uint32_t cycles1_ant;
extern float cycles1f;
extern float CLOCK_f;


extern float invSqrtf(float x);

#endif
