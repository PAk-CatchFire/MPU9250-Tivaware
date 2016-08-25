#include "../include.h"
#include "MPU9250.h"

/*
#define DWT_O_CYCCNT 				0x00000004

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

void EnableBenchmarkingCode(void) {
	//Needed for the Now() function
	static int enabled = 0;
	if (!enabled) {
		HWREG(NVIC_DBG_INT) |= 0x01000000; //enable TRCENA bit in NVIC_DBG_INT//
		HWREG(DWT_BASE + DWT_O_CYCCNT) = 0; // reset the counter //
		HWREG(DWT_BASE) |= 0x01; /* enable the counter //
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

/**/
void ConfigMPU9250() {
	// wake up device
	I2CWriteByte(MPU9250_ADDR, MPUREG_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	//SysCtlDelay(8000);//	delay(100); // Wait for all registers to reset
	delayMS(300);

	// get stable time source
	I2CWriteByte(MPU9250_ADDR, MPUREG_PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
	//SysCtlDelay(8000);//delay(200);
	delayMS(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	I2CWriteByte(MPU9250_ADDR, MPUREG_CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + MPUREG_SMPLRT_DIV)
	I2CWriteByte(MPU9250_ADDR, MPUREG_SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate
	// determined inset in MPUREG_CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = I2CReadByte(MPU9250_ADDR, MPUREG_GYRO_CONFIG);
	//  writeRegister(MPUREG_MPUREG_GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	delayMS(100);
	I2CWriteByte(MPU9250_ADDR, MPUREG_GYRO_CONFIG, c & ~0xE0); // Clear Fchoice bits [7:5]
	//I2CWriteByte(MPU9250_ADDR, MPUREG_GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0]
	I2CWriteByte(MPU9250_ADDR, MPUREG_GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	I2CWriteByte(MPU9250_ADDR, MPUREG_GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

	//c=c & ~0xE0;
	//c=c & ~0x18;
	//c=c | Gscale << 3;
	//I2CWriteByte(MPU9250_ADDR, MPUREG_GYRO_CONFIG, c);

	// writeRegister(MPUREG_GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of MPUREG_GYRO_CONFIG
	delayMS(100);

	// Set accelerometer full-scale range configuration
	c = I2CReadByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG);
	delayMS(100);
	//  writeRegister(MPUREG_ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	I2CWriteByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG, c & ~0xE0); // Clear AFS bits [7:5]
	I2CWriteByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	I2CWriteByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

	//c = c & ~0x18;  // Clear AFS bits [4:3]
	//c = c | Ascale << 3; // Set full scale range for the accelerometer
	//I2CWriteByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG, c ); // Set full scale range for the accelerometer

	delayMS(100);

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = I2CReadByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG_2);
	delayMS(100);

	I2CWriteByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG_2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	I2CWriteByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG_2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	//c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	//  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	//I2CWriteByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG_2, c);
	delayMS(100);

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the MPUREG_SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of MPUREG_INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	//uint8_t valor_dato=MPU9150_INT_PIN_CFG_INT_LEVEL |
	//		MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
	//	           MPU9150_INT_PIN_CFG_LATCH_INT_EN;

	I2CWriteByte(MPU9250_ADDR, MPUREG_INT_PIN_CFG, 0x22);
	I2CWriteByte(MPU9250_ADDR, MPUREG_INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt

	//SOME TESTS
	//I2CWriteByte(MPU9250_ADDR, MPUREG_INT_PIN_CFG, valor_dato);

	//I2CWriteByte(MPU9250_ADDR, MPUREG_INT_PIN_CFG, 0x36);
	//I2CWriteByte(MPU9250_ADDR, MPUREG_INT_PIN_CFG, 0x16);
	//I2CWriteByte(MPU9250_ADDR, MPUREG_INT_ENABLE, 0x09);  // Enable data ready (bit 0) interrupt

	delayMS(100);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void CalibrateMPU9250(float * dest1, float * dest2) {
	int8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint32_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device
	I2CWriteByte(MPU9250_ADDR, MPUREG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	//SysCtlDelay(8000);//delay(100);
	delayMS(500);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	I2CWriteByte(MPU9250_ADDR, MPUREG_PWR_MGMT_1, 0x01);
	I2CWriteByte(MPU9250_ADDR, MPUREG_PWR_MGMT_2, 0x00);
	//SysCtlDelay(8000);//delay(200);
	delayMS(300);

	// Configure device for bias calculation
	I2CWriteByte(MPU9250_ADDR, MPUREG_INT_ENABLE, 0x00); // Disable all interrupts
	I2CWriteByte(MPU9250_ADDR, MPUREG_FIFO_EN, 0x00);      // Disable FIFO
	I2CWriteByte(MPU9250_ADDR, MPUREG_PWR_MGMT_1, 0x00); // Turn on internal clock source
	I2CWriteByte(MPU9250_ADDR, MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
	I2CWriteByte(MPU9250_ADDR, MPUREG_USER_CTRL, 0x00); // Disable FIFO and I2C master modes
	I2CWriteByte(MPU9250_ADDR, MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
	//SysCtlDelay(800);//delay(15);
	delayMS(100);

	// Configure MPU9250 gyro and accelerometer for bias calculation
	I2CWriteByte(MPU9250_ADDR, MPUREG_CONFIG, 0x01); // Set low-pass filter to 188 Hz
	I2CWriteByte(MPU9250_ADDR, MPUREG_SMPLRT_DIV, 0x00); // Set sample rate to 1 kHz
	I2CWriteByte(MPU9250_ADDR, MPUREG_GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	I2CWriteByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	delayMS(100);

	uint32_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint32_t accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	I2CWriteByte(MPU9250_ADDR, MPUREG_USER_CTRL, 0x40);   // Enable FIFO
	I2CWriteByte(MPU9250_ADDR, MPUREG_FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	//SysCtlDelay(800);//delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes
	delayMS(100);

	// At end of sample accumulation, turn off FIFO sensor read
	I2CWriteByte(MPU9250_ADDR, MPUREG_FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
	I2CRead(MPU9250_ADDR, MPUREG_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t) data[0] << 8) | data[1];
	packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging
	delayMS(100);

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		I2CRead(MPU9250_ADDR, MPUREG_FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0] += (int32_t) gyro_temp[0];
		gyro_bias[1] += (int32_t) gyro_temp[1];
		gyro_bias[2] += (int32_t) gyro_temp[2];

	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0] /= (int32_t) packet_count;
	gyro_bias[1] /= (int32_t) packet_count;
	gyro_bias[2] /= (int32_t) packet_count;

	if (accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

	// Push gyro biases to hardware registers
	I2CWriteByte(MPU9250_ADDR, MPUREG_XG_OFFS_USRH, data[0]);
	I2CWriteByte(MPU9250_ADDR, MPUREG_XG_OFFS_USRL, data[1]);
	I2CWriteByte(MPU9250_ADDR, MPUREG_YG_OFFS_USRH, data[2]);
	I2CWriteByte(MPU9250_ADDR, MPUREG_YG_OFFS_USRL, data[3]);
	I2CWriteByte(MPU9250_ADDR, MPUREG_ZG_OFFS_USRH, data[4]);
	I2CWriteByte(MPU9250_ADDR, MPUREG_ZG_OFFS_USRL, data[5]);

	// Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	I2CRead(MPU9250_ADDR, MPUREG_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t) data[0] << 8) | data[1]);
	I2CRead(MPU9250_ADDR, MPUREG_XA_OFFSET_L, 2, &data[0]);
	accel_bias_reg[1] = (int32_t) (((int16_t) data[0] << 8) | data[1]);
	I2CRead(MPU9250_ADDR, MPUREG_ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t) (((int16_t) data[0] << 8) | data[1]);

	delayMS(100);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++) {
		if ((accel_bias_reg[ii] & mask))
			mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	I2CWriteByte(MPU9250_ADDR, MPUREG_XA_OFFSET_H, data[0]);
	I2CWriteByte(MPU9250_ADDR, MPUREG_XA_OFFSET_L, data[1]);
	I2CWriteByte(MPU9250_ADDR, MPUREG_YA_OFFSET_H, data[2]);
	I2CWriteByte(MPU9250_ADDR, MPUREG_YA_OFFSET_L, data[3]);
	I2CWriteByte(MPU9250_ADDR, MPUREG_ZA_OFFSET_H, data[4]);
	I2CWriteByte(MPU9250_ADDR, MPUREG_ZA_OFFSET_L, data[5]);

	delayMS(100);

	// Output scaled accelerometer biases for display in the main program
	dest2[0] = (float) accel_bias[0] / (float) accelsensitivity;
	dest2[1] = (float) accel_bias[1] / (float) accelsensitivity;
	dest2[2] = (float) accel_bias[2] / (float) accelsensitivity;
}
// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	int8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t selfTest[6];
	int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
	float factoryTrim[6];
	uint8_t FS = 0;

	I2CWriteByte(MPU9250_ADDR, MPUREG_SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
	I2CWriteByte(MPU9250_ADDR, MPUREG_CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	I2CWriteByte(MPU9250_ADDR, MPUREG_GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
	I2CWriteByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG_2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	I2CWriteByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

	char ii;
	for (ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

		I2CRead(MPU9250_ADDR, MPUREG_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

		I2CRead(MPU9250_ADDR, MPUREG_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
	}

	for (ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	I2CWriteByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	I2CWriteByte(MPU9250_ADDR, MPUREG_GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	SysCtlDelay(800);  // Delay a while to let the device stabilize

	for (ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

		I2CRead(MPU9250_ADDR, MPUREG_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

		I2CRead(MPU9250_ADDR, MPUREG_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
	}
	for (ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	I2CWriteByte(MPU9250_ADDR, MPUREG_ACCEL_CONFIG, 0x00);
	I2CWriteByte(MPU9250_ADDR, MPUREG_GYRO_CONFIG, 0x00);
	SysCtlDelay(800);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = I2CReadByte(MPU9250_ADDR, MPUREG_SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = I2CReadByte(MPU9250_ADDR, MPUREG_SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = I2CReadByte(MPU9250_ADDR, MPUREG_SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = I2CReadByte(MPU9250_ADDR, MPUREG_SELF_TEST_X_GYRO); // X-axis gyro self-test results
	selfTest[4] = I2CReadByte(MPU9250_ADDR, MPUREG_SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
	selfTest[5] = I2CReadByte(MPU9250_ADDR, MPUREG_SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[2] - 1.0))); // FT[Za] factory trim calculation
	factoryTrim[3] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	char i;
	for (i = 0; i < 3; i++) {
		destination[i] = 100.0 * ((float) (aSTAvg[i] - aAvg[i]))
				/ factoryTrim[i];   // Report percent differences
		destination[i + 3] = 100.0 * ((float) (gSTAvg[i] - gAvg[i]))
				/ factoryTrim[i + 3]; // Report percent differences
	}
}

void ConfigAK8963(float * destination) {
	// First extract the factory calibration for each magnetometer axis
	int8_t rawData[3];  // x/y/z gyro calibration data stored here
	I2CWriteByte(AK8963_I2C_ADDR, AK8963_CNTL, 0x00); // Power down magnetometer
	SysCtlDelay(800);
	I2CWriteByte(AK8963_I2C_ADDR, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	SysCtlDelay(800);
	I2CRead(AK8963_I2C_ADDR, AK8963_ASAX, 3, &rawData[0]); // Read the x-, y-, and z-axis calibration values
	destination[0] = (float) (rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
	destination[1] = (float) (rawData[1] - 128) / 256. + 1.;
	destination[2] = (float) (rawData[2] - 128) / 256. + 1.;
	I2CWriteByte(AK8963_I2C_ADDR, AK8963_CNTL, 0x00); // Power down magnetometer
	SysCtlDelay(800);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	I2CWriteByte(AK8963_I2C_ADDR, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	SysCtlDelay(800);
}
int16_t ReadTempData() {
	int8_t rawData[2];  // x/y/z gyro register data stored here
	I2CRead(MPU9250_ADDR, MPUREG_TEMP_OUT_H, 2, &rawData[0]); // Read the two raw data registers sequentially into data array
	return ((int16_t) rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a 16-bit value
}
void ReadMagData(int16_t * destination) {
	int8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	if (I2CReadByte(AK8963_I2C_ADDR, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
		I2CRead(AK8963_I2C_ADDR, AK8963_XOUT_L, 7, &rawData[0]); // Read the six raw data and ST2 registers sequentially into data array
		int8_t c = rawData[6]; // End data read by reading ST2 register
		if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			destination[0] =
					(int16_t) (((int16_t) rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
			destination[1] =
					(int16_t) (((int16_t) rawData[3] << 8) | rawData[2]); // Data stored as little Endian
			destination[2] =
					(int16_t) (((int16_t) rawData[5] << 8) | rawData[4]);
		}
	}
}
void ReadGyroData(int16_t * destination) {
	int8_t rawData[6];  // x/y/z gyro register data stored here
	I2CRead(MPU9250_ADDR, MPUREG_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
	destination[0] = (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
	destination[2] = (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
}
void ReadAccelData(int16_t * destination) {
	int8_t rawData[6];  // x/y/z accel register data stored here
	I2CRead(MPU9250_ADDR, MPUREG_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
	destination[0] = (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
	destination[2] = (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
}
void GetMres() {
	switch (Mscale) {
	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		mRes = 10. * 4912. / 8190.; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mRes = 10. * 4912. / 32760.0; // Proper scale to return milliGauss
		break;
	}
}

void GetGres() {
	switch (Gscale) {
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case GFS_250DPS:
		gRes = 250.0 / 32768.0;
		break;
	case GFS_500DPS:
		gRes = 500.0 / 32768.0;
		break;
	case GFS_1000DPS:
		gRes = 1000.0 / 32768.0;
		break;
	case GFS_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}

void GetAres() {
	switch (Ascale) {
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case AFS_2G:
		aRes = 2.0 / 32768.0;
		break;
	case AFS_4G:
		aRes = 4.0 / 32768.0;
		break;
	case AFS_8G:
		aRes = 8.0 / 32768.0;
		break;
	case AFS_16G:
		aRes = 16.0 / 32768.0;
		break;
	}
}



float Now() {
	//	UARTprintf("delay deltat=%d beta:%d - dif_cycles1:%d\n ",(uint32_t)(deltat*1000),(uint32_t)(beta*1000),(cycles1-cycles1_ant));

	float Now = 0;

	CLOCK_f = (float) (g_ui32SysClock);  //system clock
	cycles1_ant = cycles1;
	cycles1 = HWREG(DWT_BASE + DWT_O_CYCCNT);

	cycles1f = (float) (cycles1 - cycles1_ant);

	Now = (cycles1f * 1) / (CLOCK_f);	//e

	return Now;
}
void ResetMPU9250() {
	// reset device
	I2CWriteByte(MPU9250_ADDR, MPUREG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	//SysCtlDelay(80000);
	delayMS(500);
}

void Setup() {

	pfAccel = pfData;
	pfGyro = pfData + 3;
	pfMag = pfData + 6;
	pfEulers = pfData + 9;
	pfQuaternion = pfData + 12;

	EnableBenchmarkingCode();
	// Read the WHO_AM_I register, this is a good test of communication
	char c = 0;
	uint8_t whoami = 0;

	while (c != 0x71)
	//	while(c!=0x68)
	{
		c = I2CReadByte(MPU9250_ADDR, MPUREG_WIA); // Read WHO_AM_I register for MPU-9250
//		UARTprintf("MPU9250 "); UARTprintf("I AM ");
//		UARTprintf("%d,",c);
//		UARTprintf(" I should be ");
//		UARTprintf("%d \n",0x71);
		if (c != 0x71) {

			//UARTprintf("SETUP  addr: 0x%x\n",c);
		}
		UARTprintf("FOUND addr: 0x%x\n", c);
		delayMS(200);
	}

	if (c == 0x71) // WHO_AM_I should always be 0x68
			// if (c == 0x68) // WHO_AM_I should always be 0x68
			{
		UARTprintf("MPU9250 is online...\n");

		MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
		UARTprintf("x-axis self test: acceleration trim within : ");
		UARTprintf("%d ", (int32_t) (SelfTest[0] * 1000));
		UARTprintf(" of factory value\n");
		UARTprintf("y-axis self test: acceleration trim within : ");
		UARTprintf("%d ", (int32_t) (SelfTest[1] * 1000));
		UARTprintf(" of factory value\n");
		UARTprintf("z-axis self test: acceleration trim within : ");
		UARTprintf("%d ", (int32_t) (SelfTest[2] * 1000));
		UARTprintf(" of factory value\n");
		UARTprintf("x-axis self test: gyration trim within : ");
		UARTprintf("%d ", (int32_t) (SelfTest[3] * 1000));
		UARTprintf(" of factory value\n");
		UARTprintf("y-axis self test: gyration trim within : ");
		UARTprintf("%d ", (int32_t) (SelfTest[4] * 1000));
		UARTprintf(" of factory value\n");
		UARTprintf("z-axis self test: gyration trim within : ");
		UARTprintf("%d ", (int32_t) (SelfTest[5] * 1000));
		UARTprintf(" of factory value\n");

		delayMS(200);

		/*
		 //Better without CALIB??!!

		 CalibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

		 UARTprintf("x gyro bias = %d\n\r", (int32_t) (gyroBias[0]) * 1000);
		 UARTprintf("y gyro bias = %d\n\r", (int32_t) (gyroBias[1] * 1000));
		 UARTprintf("z gyro bias = %d\n\r", (int32_t) (gyroBias[2] * 1000));
		 UARTprintf("x accel bias = %d\n\r", (int32_t) (accelBias[0] * 1000));
		 UARTprintf("y accel bias = %d\n\r", (int32_t) (accelBias[1] * 1000));
		 UARTprintf("z accel bias = %d\n\r", (int32_t) (accelBias[2] * 1000));

		 delayMS(200);
		 /**/

		ConfigMPU9250();
		UARTprintf("MPU9250 initialized for active data mode....\n"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

		delayMS(200);

		char d = 0;

		while (d != 0x48) {

			// Read the WHO_AM_I register of the magnetometer, this is a good test of communication

			d = I2CReadByte(AK8963_I2C_ADDR, AK8963_WIA); // Read WHO_AM_I register for AK8963
			UARTprintf("AK8963 ");
			UARTprintf("I AM ");
			UARTprintf("0x%x", d);
			UARTprintf(" I should be ");
			UARTprintf("0x%x\n", 0x48);
		}
		// Get magnetometer calibration from AK8963 ROM
		ConfigAK8963(magCalibration);
		UARTprintf("AK8963 initialized for active data mode....\n"); // Initialize device for active mode read of magnetometer

		if (SerialDebug) {
			UARTprintf("\nCalibration values: ");
			UARTprintf("x-axis sensitivity adjustment value ");
			UARTprintf("%d \n", (int32_t) (magCalibration[0] * 1000));
			UARTprintf("y-axis sensitivity adjustment value ");
			UARTprintf("%d \n", (int32_t) (magCalibration[1] * 1000));
			UARTprintf("z-axis sensitivity adjustment value ");
			UARTprintf("%d \n", (int32_t) (magCalibration[2] * 1000));

		}

	}
}


//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
		float gz, float mx, float my, float mz) {

	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	/*		UARTprintf("MadgwickQ Input: \nax:%d ay:%d az:%d  \ngx:%d gy:%d gz:%d \nmx:%d my:%d mz:%d \nq1:%d q2:%d q3:%d q4:%d\n\n",(int32_t)(ax*1000),(int32_t)(ay*1000),(int32_t)(az*1000),
	 (int32_t)(gx*1000),(int32_t)(gy*1000),(int32_t)(gz*1000),
	 (int32_t)(mx*1000),(int32_t)(my*1000),(int32_t)(mz*1000),
	 (int32_t)(q1*1000),(int32_t)(q2*1000),(int32_t)(q3*1000),(int32_t)(q4*1000)
	 );/**/

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

//		UARTprintf("MadgwickQ ax:%d ay:%d az:%d norm:%d \n",(int32_t)(ax*1000),(int32_t)(ay*1000),(int32_t)(az*1000),(int32_t)(norm*1000));

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	//	UARTprintf("MadgwickQ mx:%d my:%d mz:%d norm:%d \n",(int32_t)(mx*1000),(int32_t)(my*1000),(int32_t)(mz*1000),(int32_t)(norm*1000));

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3
			+ _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2
			+ my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2
			+ _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

//		UARTprintf("MadgwickQ _2bx:%d _2bz:%d _4bx:%d _4bz:%d norm:%d \n",(int32_t)(_2bx*1000000),(int32_t)(_2bz*1000000),(int32_t)(_4bx*1000000),(int32_t)(_4bz*1000000),(int32_t)(norm*1000));

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax)
			+ _2q2 * (2.0f * q1q2 + _2q3q4 - ay)
			- _2bz * q3
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (-_2bx * q4 + _2bz * q2)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ _2bx * q3
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay)
			- 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az)
			+ _2bz * q4
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (_2bx * q3 + _2bz * q1)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ (_2bx * q4 - _4bz * q2)
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax)
			+ _2q4 * (2.0f * q1q2 + _2q3q4 - ay)
			- 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az)
			+ (-_4bx * q3 - _2bz * q1)
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (_2bx * q2 + _2bz * q4)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ (_2bx * q1 - _4bz * q3)
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay)
			+ (-_4bx * q4 + _2bz * q2)
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (-_2bx * q1 + _2bz * q3)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ _2bx * q2
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
	norm = 1.0f / norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

//		UARTprintf("MadgwickQ s1:%d s2:%d s3:%d s4:%d norm:%d \n",(int32_t)(s1*1000000),(int32_t)(s2*1000000),(int32_t)(s3*1000000),(int32_t)(s4*1000000),(int32_t)(norm*1000));

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta_p * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta_p * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta_p * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta_p * s4;

	//	UARTprintf("MadgwickQ qDot1:%d qDot2:%d qDot3:%d qDot4:%d norm:%d \n",(int32_t)(qDot1*1000000),(int32_t)(qDot2*1000000),(int32_t)(qDot3*1000000),(int32_t)(qDot4*1000000),(int32_t)(norm*1000));

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

	//	UARTprintf("MadgwickQ q1:%d q2:%d q3:%d q4:%d norm:%d \n",(int32_t)(q1*1000000),(int32_t)(q2*1000000),(int32_t)(q3*1000000),(int32_t)(q4*1000000),(int32_t)(norm*1000));

//		UARTprintf("MadgwickQ q[0]:%d q[1]:%d q[2]:%d q[3]:%d norm:%d \n",(int32_t)(q[0]*1000000),(int32_t)(q[1]*1000000),(int32_t)(q[2]*1000000),(int32_t)(q[3]*1000000),(int32_t)(norm*1000));

}

// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
		float gz, float mx, float my, float mz) {
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4)
			+ 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4)
			+ 2.0f * mz * (q3q4 - q1q2);
	bx = sqrtf((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2)
			+ 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f) {
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	} else {
		eInt[0] = 0.0f;     // prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

	// Normalise quaternion
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

void UpdateData() {

	bool new_data = false;
	// SerialDebug=0;
	// If intPin goes high, all data registers have new data
	if (I2CReadByte(MPU9250_ADDR, MPUREG_INT_STATUS) & 0x01) {

		// On interrupt, check if data ready interrupt
		/***READ ACCELEROMETER***/
		ReadAccelData(accelCount);  // Read the x/y/z adc values
//		GetAres();

		// Now we'll calculate the accleration value into actual g's
		ax = (float) (accelCount[0]) * aRes - accelBias[0]; // get actual g value, this depends on scale being set
		ay = (float) (accelCount[1]) * aRes - accelBias[1];
		az = (float) (accelCount[2]) * aRes - accelBias[2];

		/***READ GYROSCOPE***/
		ReadGyroData(gyroCount);  // Read the x/y/z adc values
		//GetGres();

		// Calculate the gyro value into actual degrees per second
		gx = (float) (gyroCount[0]) * gRes - gyroBias[0]; // get actual gyro value, this depends on scale being set
		gy = (float) (gyroCount[1]) * gRes - gyroBias[1];
		gz = (float) (gyroCount[2]) * gRes - gyroBias[2];

		/***READ MAGNETOMETER***/
		ReadMagData(magCount);  // Read the x/y/z adc values
		//GetMres();
//		magBias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
//		magBias[1] = +120.;  // User environmental y-axis correction in milliGauss
//		magBias[2] = +125.;  // User environmental z-axis correction in milliGauss

		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		mx = (float) (magCount[0]) * mRes * magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
		my = (float) (magCount[1]) * mRes * magCalibration[1] - magBias[1];
		mz = (float) (magCount[2]) * mRes * magCalibration[2] - magBias[2];

		pfAccel[0] = ax;
		pfAccel[1] = ay;
		pfAccel[2] = az;

		pfGyro[0] = gx;
		pfGyro[1] = gy;
		pfGyro[2] = gz;

		pfMag[0] = mx;
		pfMag[1] = my;
		pfMag[2] = mz;

		new_data = true;
	}

	deltat = Now();

	//UARTprintf("update_out!!\n ");
	// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
	// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
	// We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
	// For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
	// in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
	// This is ok by aircraft orientation standards!
	// Pass gyro rate as rad/s
	MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f,gz * PI / 180.0f, my, mx, mz);
	//MadgwickQuaternionUpdate(ax, ay, -az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
	//MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

	//NOT NICE
	//	 roll  = atan2f(ay, az) * RAD_TO_DEG;
	//	 pitch = atanf(-ax / sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

	//		// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	//		// In this coordinate system, the positive z-axis is down toward Earth.
	//		// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	//		// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	//		// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	//		// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	//		// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	//		// applied in the correct order which for this configuration is yaw, pitch, and then roll.
	//		// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
	yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),	q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

	pitch *= 180.0f / PI;
	yaw *= 180.0f / PI;
	//yaw   -= 0.8; // Declination at WHEREVER
	roll *= 180.0f / PI;
	/*
	 UARTprintf("Yaw, Pitch, Roll: ");
	 UARTprintf("%d",(int)yaw);
	 UARTprintf(", ");
	 UARTprintf("%d",(int)pitch);
	 UARTprintf(", ");
	 UARTprintf("%d \n",(int)roll);/**/

	pfData[0] = ax;
	pfData[1] = ay;
	pfData[2] = az;

	pfData[3] = gx;
	pfData[4] = gy;
	pfData[5] = gz;

	pfData[6] = mx;
	pfData[7] = my;
	pfData[8] = mz;

	pfData[9] = roll;
	pfData[10] = pitch;
	pfData[11] = yaw;
	if (pfData[11] < 0) {
		pfData[11] += 360.0f;
	}

	while (pfData[9] >= 360) {
		pfData[9] -= 360.0f;
	}

	while (pfData[10] >= 360) {
		pfData[10] -= 360.0f;
	}

	while (pfData[11] >= 360) {
		pfData[11] -= 360.0f;
	}

	pfData[12] = q[0];
	pfData[13] = q[1];
	pfData[14] = q[2];
	pfData[15] = q[3];

	// Now drop back to using the data as a single array for the
	// purpose of decomposing the float into a integer part and a
	// fraction (decimal) part.
	//
	uint32_t ui32Idx = 0;
	for (ui32Idx = 0; ui32Idx < 16; ui32Idx++) {
		//
		// Conver float value to a integer truncating the decimal part.
		//
		i32IPart[ui32Idx] = (int32_t) pfData[ui32Idx];

		//
		// Multiply by 1000 to preserve first three decimal values.
		// Truncates at the 3rd decimal place.
		//
		i32FPart[ui32Idx] = (int32_t) (pfData[ui32Idx] * 1000.0f);

		//
		// Subtract off the integer part from this newly formed decimal
		// part.
		//
		i32FPart[ui32Idx] = i32FPart[ui32Idx] - (i32IPart[ui32Idx] * 1000);

		//
		// make the decimal part a positive number for display.
		//
		if (i32FPart[ui32Idx] < 0) {
			i32FPart[ui32Idx] *= -1;
		}
	}

	if (new_data) {
		/*
		 UARTprintf("ax = %d", (int32_t)(1000*ax*100));
		 UARTprintf(" ay = %d", (int32_t)(1000*ay*100));
		 UARTprintf(" az = %d  mg\n\r", (int32_t)(1000*az*100));

		 UARTprintf("gx = %d", (int32_t)(gx*100));
		 UARTprintf(" gy = %d", (int32_t)(gy*100));
		 UARTprintf(" gz = %d  deg/s\n\r", (int32_t)(gz*100));

		 UARTprintf("mx = %d", (int32_t)(mx*100));
		 UARTprintf(" my = %d", (int32_t)(my*100));
		 UARTprintf(" mz = %d  mG\n\r", (int32_t)(mz*100));

		 // tempCount = MPU9150.readTempData();  // Read the adc values
		 //   temperature = ((float) tempCount) / 340.0f + 36.53f; // Temperature in degrees Centigrade
		 //   pc.printf(" temperature = %f  C\n\r", temperature);

		 UARTprintf("q0 = %d\n\r", (int32_t)(q[0]*100));
		 UARTprintf("q1 = %d\n\r", (int32_t)(q[1]*100));
		 UARTprintf("q2 = %d\n\r", (int32_t)(q[2]*100));
		 UARTprintf("q3 = %d\n\r", (int32_t)(q[3]*100));
		 /**/
		/*			                //
		 // Print the acceleration numbers in the table.
		 //
		 UARTprintf("\033[5;17H%3d.%03d", i32IPart[0], i32FPart[0]);
		 UARTprintf("\033[5;40H%3d.%03d", i32IPart[1], i32FPart[1]);
		 UARTprintf("\033[5;63H%3d.%03d", i32IPart[2], i32FPart[2]);

		 //
		 // Print the angular velocities in the table.
		 //
		 UARTprintf("\033[7;17H%3d.%03d", i32IPart[3], i32FPart[3]);
		 UARTprintf("\033[7;40H%3d.%03d", i32IPart[4], i32FPart[4]);
		 UARTprintf("\033[7;63H%3d.%03d", i32IPart[5], i32FPart[5]);

		 //
		 // Print the magnetic data in the table.
		 //
		 UARTprintf("\033[9;17H%3d.%03d", i32IPart[6], i32FPart[6]);
		 UARTprintf("\033[9;40H%3d.%03d", i32IPart[7], i32FPart[7]);
		 UARTprintf("\033[9;63H%3d.%03d", i32IPart[8], i32FPart[8]);

		 //
		 // Print the Eulers in a table.
		 //
		 UARTprintf("\033[14;17H%3d.%03d", i32IPart[9], i32FPart[9]);
		 UARTprintf("\033[14;40H%3d.%03d", i32IPart[10], i32FPart[10]);
		 UARTprintf("\033[14;63H%3d.%03d", i32IPart[11], i32FPart[11]);

		 //
		 // Print the quaternions in a table format.
		 //
		 UARTprintf("\033[19;14H%3d.%03d", i32IPart[12], i32FPart[12]);
		 UARTprintf("\033[19;32H%3d.%03d", i32IPart[13], i32FPart[13]);
		 UARTprintf("\033[19;50H%3d.%03d", i32IPart[14], i32FPart[14]);
		 UARTprintf("\033[19;68H%3d.%03d", i32IPart[15], i32FPart[15]);
		 /**/


		cnt_IMU_calc++;
		g_ui32PrintSkipCounter++;
	}

	//if(samplingFlag)
	//{
	// Increment the skip counter.  Skip counter is used so we do not
	// overflow the UART with data.
	//

	if (g_ui32PrintSkipCounter >= PRINT_SKIP_COUNT) {
		cnt_IMU_calc_reales++;

		//
		// Reset skip counter.
		//
		g_ui32PrintSkipCounter = 0;

		samplingFlag = false;
		//To processing SKETCH
			UARTprintf(
					"%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%03d.%03d,%u,%u,%u\n",
					i32IPart[12], i32FPart[12], i32IPart[13], i32FPart[13],
					i32IPart[14], i32FPart[14], i32IPart[15],
					i32FPart[15], //quaternions
					i32IPart[0], i32FPart[0], i32IPart[1], i32FPart[1], i32IPart[2],
					i32FPart[2], //acc
					i32IPart[3], i32FPart[3], i32IPart[4], i32FPart[4], i32IPart[5],
					i32FPart[5], //gyro
					i32IPart[6], i32FPart[6], i32IPart[7], i32FPart[7], i32IPart[8],
					i32FPart[8], //mag
					i32IPart[9], i32FPart[9], i32IPart[10], i32FPart[10],
					i32IPart[11], i32FPart[11], //Euler
					deltat, cnt_IMU_calc_reales, cnt_msg_IMU++);/**/

	}

}

