/*
 * include.c
 *
 *  CODED by PAk on 24/08/2016
 *  https://e2e.ti.com/members/521000
*/

#include "include.h"

float pfData[16];
float *pfAccel, *pfGyro, *pfMag, *pfEulers, *pfQuaternion;

int_fast32_t i32IPart[16], i32FPart[16];

//*****************************************************************************
//
// Global counter to control and slow down the rate of data to the terminal.
//
//*****************************************************************************
//#define PRINT_SKIP_COUNT        4
uint32_t g_ui32PrintSkipCounter;

uint32_t cnt_IMU_calc = 0;
uint32_t cnt_IMU_calc_reales = 0;
uint32_t cnt_msg_IMU = 0;

//#define sampleFreq	512.0f		// sample frequency in Hz

void delayMS(int ms) {

	ROM_SysCtlDelay((g_ui32SysClock / (1 * 1000)) * ms); // more accurate
// ROM_SysCtlDelay ((ROM_SysCtlClockGet () / (1 * 1000)) * ms); // more accurate
//SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;  // less accurate
}

void delay_uS(int us) {

	ROM_SysCtlDelay((g_ui32SysClock / (1 * 1000000)) * us); // more accurate
// ROM_SysCtlDelay ((ROM_SysCtlClockGet () / (1 * 1000)) * ms); // more accurate
//SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;  // less accurate
}

// Set initial input parameters
/*
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
/**/

void EnableBenchmarkingCode(void) {
	//Needed for the Now() function
	static int enabled = 0;
	if (!enabled) {
		HWREG(NVIC_DBG_INT) |= 0x01000000; //enable TRCENA bit in NVIC_DBG_INT//
		HWREG(DWT_BASE + DWT_O_CYCCNT) = 0; // reset the counter //
		HWREG(DWT_BASE) |= 0x01; // enable the counter //
		enabled = 1;
	}
}

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x02; // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes = 0, gRes = 0, mRes = 0; // scale resolutions per LSB for the sensors

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = { 0, 0, 0 }, magBias[3] = { 0, 0, 0 }; // Factory mag calibration and mag bias
float gyroBias[3] = { 0, 0, 0 }, accelBias[3] = { 0, 0, 0 }; // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output
float temperature; // Stores the real internal chip temperature in degrees Celsius
float SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define PI 3.141516
float GyroMeasError = PI * (40.0 / 180.0); // gyroscope measurement error in rads/s (start at 40 deg/s)
//float GyroMeasDrift = PI * (0.0  / 180.0);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
//float GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta_p = 0.604;   //// sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = 0.0174; //PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta_p = 0.015; //sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float deltat = 0.0, sum = 0.1;   // integration interval for both filter schemes

// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.

uint32_t count = 0, sumCount = 0; // used to control display output rate

float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };    // vector to hold quaternion
float eInt[3] = { 0.0f, 0.0f, 0.0f }; // vector to hold integral error for Mahony method

float q0 = 0;
float q1 = 0;
float q2 = 0;
float q3 = 0;

float pitch, yaw, roll;


bool samplingFlag = false;
uint32_t cycles1 = 0;
uint32_t cycles1_ant = 0;
float cycles1f = 0;
float CLOCK_f = 0;


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrtf(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

