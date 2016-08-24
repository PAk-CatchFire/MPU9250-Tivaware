/*
 * I2C_pak.c
 *
 *  Created on: August 24, 2016
 *      Author: PAk
 *      https://e2e.ti.com/members/521000
 */
#include "../include.h"
void ConfigI2C()
{
	//ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C2);
	//ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

	ROM_SysCtlDelay(2);// Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);// Enable GPIOA peripheral
	ROM_SysCtlDelay(2);// Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

	//
		// Configure the pin muxing for I2C2 functions on port E4 and E5.
		// This step is not necessary if your part does not support pin muxing.
		//
		GPIOPinConfigure(GPIO_PN5_I2C2SCL);
		GPIOPinConfigure(GPIO_PN4_I2C2SDA);

		//
		// Select the I2C function for these pins.  This function will also
		// configure the GPIO pins for I2C operation, setting them to
		// open-drain operation with weak pull-ups.  Consult the data sheet
		// to see which functions are allocated per pin.
		//
		GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);	//	special I2CSCL treatment for M4F devices
		ROM_GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

		//
		// Wait for the Peripheral to be ready for programming
		//
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2));

		ROM_I2CMasterInitExpClk(I2C2_BASE, g_ui32SysClock, true); // Enable and set frequency to 400 kHz

		ROM_SysCtlDelay(4); // Insert a few cycles after enabling the I2C to allow the clock to be fully activated

		//
		// Enable the Glitch Filter. Writting a value 0 will
		// disable the glitch filter
		// I2C_MASTER_GLITCH_FILTER_DISABLED
		// I2C_MASTER_GLITCH_FILTER_1
		// I2C_MASTER_GLITCH_FILTER_2 : Ideal Value when in HS Mode
		// for 120MHz clock
		// I2C_MASTER_GLITCH_FILTER_4
		// I2C_MASTER_GLITCH_FILTER_8 : Ideal Value when in Std,
		// Fast, Fast+ for 120MHz clock
		// I2C_MASTER_GLITCH_FILTER_16
		// I2C_MASTER_GLITCH_FILTER_32
		//
		I2CMasterGlitchFilterConfigSet(I2C2_BASE, I2C_MASTER_GLITCH_FILTER_2);



}
void I2CWriteByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t TXData)
{

	//
	// Wait until master module is done transferring.
	//
	while(ROM_I2CMasterBusy(I2C2_BASE)){};


	ROM_I2CMasterSlaveAddrSet(I2C2_BASE, slaveAddr, false);//Send slave address with receive flag is clear.
	ROM_I2CMasterDataPut(I2C2_BASE,regAddr);//Place register address to write into I2C Master Data Register.
	ROM_I2CMasterControl(I2C2_BASE,I2C_MASTER_CMD_BURST_SEND_START);//Initiate send of burst command to slave.
	 while(!I2CMasterBusy(I2C2_BASE));//pak bug
	while(ROM_I2CMasterBusy(I2C2_BASE));//Wait until transfer is done.

	ROM_I2CMasterDataPut(I2C2_BASE,TXData);//Place data to write into I2C Master Data Register.
	ROM_I2CMasterControl(I2C2_BASE,I2C_MASTER_CMD_BURST_SEND_FINISH);//Finish sending of burst command to slave.
	 while(!I2CMasterBusy(I2C2_BASE));//pak bug
	while(ROM_I2CMasterBusy(I2C2_BASE));//Wait until transfer is done.
}
void I2CRead(uint8_t slaveAddr, uint8_t regAddr, uint32_t count, int8_t* RXData)
{
	uint32_t i;
	ROM_I2CMasterSlaveAddrSet(I2C2_BASE, slaveAddr, false);//Send slave address with receive flag is clear.
	ROM_I2CMasterDataPut(I2C2_BASE,regAddr);//Place register address to read into I2C Master Data Register.
	ROM_I2CMasterControl(I2C2_BASE,I2C_MASTER_CMD_BURST_SEND_START);//Initiate send of burst command to slave.
	 while(!I2CMasterBusy(I2C2_BASE));//pak bug
	while(ROM_I2CMasterBusy(I2C2_BASE));//Wait until transfer is done.

	ROM_I2CMasterSlaveAddrSet(I2C2_BASE, slaveAddr, true);//Send slave address again with receive flag is set.
	ROM_I2CMasterControl(I2C2_BASE,I2C_MASTER_CMD_BURST_RECEIVE_START);//Initiate receive of burst command from slave.
	for (i=0;i<count-1;i++)
	{
		 while(!I2CMasterBusy(I2C2_BASE));//pak bug
		while(ROM_I2CMasterBusy(I2C2_BASE));//Wait for master to complete receiving previous data.
		RXData[i] = ROM_I2CMasterDataGet(I2C2_BASE);//Read a byte of data from the I2C Master Data Register and save to
												//RXData buffer
		if (i == count-2)
			ROM_I2CMasterControl(I2C2_BASE,I2C_MASTER_CMD_BURST_RECEIVE_FINISH);//Finish reading of burst command to slave.
		else
			ROM_I2CMasterControl(I2C2_BASE,I2C_MASTER_CMD_BURST_RECEIVE_CONT);//Continute reading of burst command to slave.
	}
	 while(!I2CMasterBusy(I2C2_BASE));//pak bug
	while(ROM_I2CMasterBusy(I2C2_BASE));//Wait for master to complete receiving previous data.
	RXData[count-1] = ROM_I2CMasterDataGet(I2C2_BASE);//
}

uint8_t I2CReadByte(uint8_t addr, uint8_t regAddr)
{
	ROM_I2CMasterSlaveAddrSet(I2C2_BASE, addr, false); //Send slave address with receive flag is clear.
	ROM_I2CMasterDataPut(I2C2_BASE, regAddr); //Place register address to read into I2C Master Data Register.
	ROM_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND); //Initiate send of single command to slave.
	 while(!I2CMasterBusy(I2C2_BASE));//pak bug
    while (ROM_I2CMasterBusy(I2C2_BASE)); // Wait until transfer is done.



    ROM_I2CMasterSlaveAddrSet(I2C2_BASE, addr, true); //Send slave address again with receive flag is set.
    ROM_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); //Initiate receive of burst command from slave.
    while(!I2CMasterBusy(I2C2_BASE));//pak bug
    while (ROM_I2CMasterBusy(I2C2_BASE)); //Wait for master to complete receiving previous data.



    return ROM_I2CMasterDataGet(I2C2_BASE); //Return data
}

//*****************************************************************************
//
//! A beautiful function provided by:  aBUGSworstnightmare
//! modded by PAk for TM4C129X devices
//! Probes the selected I2C bus for available slave devices
//!
//! \param ulI2CBase is the base for the I2C module.
//!
//! This function scans the selected I2C bus for available I2C slave device.
//! The ulI2CBase parameter is the I2C modules master base address.
//! \e ulI2CBase parameter can be one of the following values:
//!
//! - \b I2C0_MASTER_BASE
//! - \b I2C1_MASTER_BASE
//! - \b I2C2_MASTER_BASE
//! - \b I2C3_MASTER_BASE
//!
//! \return 0 if there was an error or 1 if there was not.
//
//*****************************************************************************

uint32_t g_ucerrorstate=0;
uint32_t g_ulI2CBase=0;

uint32_t
I2CBusScan(uint32_t ulI2CBase)
{
	uint8_t ucProbeAdress;
	uint32_t ucerrorstate;

	//
	// Check the arguments.
	//
	ASSERT(_I2CBaseValid(ulI2CBase));

	 ASSERT((ulI2CBase == I2C0_BASE) || (ulI2CBase == I2C1_BASE) ||
	           (ulI2CBase == I2C2_BASE) || (ulI2CBase == I2C3_BASE) ||
	           (ulI2CBase == I2C4_BASE) || (ulI2CBase == I2C5_BASE) ||
	           (ulI2CBase == I2C6_BASE) || (ulI2CBase == I2C7_BASE) ||
	           (ulI2CBase == I2C8_BASE) || (ulI2CBase == I2C9_BASE));


	//
	// Wait until master module is done transferring.
	//
	while(I2CMasterBusy(ulI2CBase))
	{
	};
	g_ulI2CBase=ulI2CBase;

	//
	// I2C Addresses are 7-bit values
	// probe the address range of 0 to 127 to find I2C slave devices on the bus
	//
	for (ucProbeAdress = 0; ucProbeAdress < 127; ucProbeAdress++)
	{
		//
		// Tell the master module what address it will place on the bus when
		// writing to the slave.
		//
		ROM_I2CMasterSlaveAddrSet(ulI2CBase, ucProbeAdress, false);
		ROM_SysCtlDelay(50000);

		//
		// Place the command to be sent in the data register.
		//
		ROM_I2CMasterDataPut(ulI2CBase, 0x00);

		//
		// Initiate send of data from the master.
		//
		ROM_I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_BURST_SEND_START);

		//I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_SINGLE_SEND);

		//ok ROM_I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_QUICK_COMMAND);

		//
		// Make some delay
		//
		ROM_SysCtlDelay(500000);

		   while(ROM_I2CMasterBusy(ulI2CBase));

		//
		// Read the I2C Master Control/Status (I2CMCS) Register to a local
		// variable
		//
		//original: ucerrorstate = ROM_I2CMasterErr(ulI2CBase);
		//original: ucerrorstate = I2CMasterIntStatus(ulI2CBase,false);
		   ucerrorstate =  HWREG(ulI2CBase + I2C_O_MRIS) ;//ok


		//g_ucerrorstate = I2CMasterErr(ulI2CBase);
		//ucerrorstate=g_ucerrorstate;
		//
		// Examining the content I2C Master Control/Status (I2CMCS) Register
		// to see if the ADRACK-Bit (Acknowledge Address) is TRUE (1)
		// ( 1: The transmitted address was not acknowledged by the slave)
		//
		//orig if(ucerrorstate & I2C_MASTER_ERR_ADDR_ACK)
		if((ucerrorstate & I2C_MASTER_INT_NACK) == I2C_MASTER_INT_NACK)
		{
			//
			// device at selected address did not acknowledge --> there's no device
			// with this address present on the I2C bus
			//
			//
			// Print a message to Stdio
			//
			//	UARTprintf("Address not found: 0x%2x - %3d- error:0x%x\n",ucProbeAdress,ucProbeAdress,ucerrorstate);
			//
			// Make some delay
			//
			//SysCtlDelay(1500000);
		}

		//
		// ( 0: The transmitted address was acknowledged by the slave)
		//
		else
		{
			//
			// device at selected address acknowledged --> there is a device
			// with this address present on the I2C bus
			//
			//
			// Print a message to Stdio
			//
			UARTprintf("\n xxxxx  ----- Address found: 0x%2x - %3d - error:0x%x\n",ucProbeAdress,ucProbeAdress,ucerrorstate);

			//
			// Make some delay
			//
			SysCtlDelay(150000);
		}
		I2CMasterIntClearEx(ulI2CBase,ucerrorstate);
	}

	//
	// End transfer of data from the master.
	//
	I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

	//
	// Print a message to Stdio
	//
	UARTprintf("I2C Bus-Scan done...\n");

	//
	// Return 1 if there is no error.
	//
	return 1;
}
