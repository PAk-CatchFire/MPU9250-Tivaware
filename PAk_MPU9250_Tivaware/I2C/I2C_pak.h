/*
 * I2C_pak.h
 *
 *  Created on: August 24, 2016
 *      Author: PAk
 *      https://e2e.ti.com/members/521000
 */

#ifndef I2C_pak_H_
#define I2C_pak_H_

void ConfigI2C();
void I2CWriteByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t TXData);
void I2CRead(uint8_t slaveAddr, uint8_t regAddr, uint32_t count,
		int8_t* RXData);
uint8_t I2CReadByte(uint8_t addr, uint8_t regAddr);

uint32_t I2CBusScan(uint32_t ulI2CBase);

extern uint32_t g_ui32SysClock;

#endif /* I2C_pak_H_ */
