//*****************************************************************************
//
// PAk_MPU9250_Tivaware.c - Sample WebServer Application using lwIP.
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/flash.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/locator.h"
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "drivers/pinout.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "sensorlib/i2cm_drv.h"
#include "drivers/buttons.h"

#include "driverlib/fpu.h"
/////////////////////

#include "I2C/I2C_pak.h"
//#include "MPU9250/MPU9250.h"

#include "include.h"



//extern float deltat;

//*****************************************************************************
//
// Defines for setting up the system clock.
//
//*****************************************************************************
#define SYSTICKHZ               100
#define SYSTICKMS               (1000 / SYSTICKHZ)

//*****************************************************************************
//
// Interrupt priority definitions.  The top 3 bits of these values are
// significant with lower values indicating higher priority interrupts.
//
//*****************************************************************************
#define SYSTICK_INT_PRIORITY    0x80

//*****************************************************************************
//
// The system clock frequency.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The interrupt handler for the SysTick interrupt.
//
//*****************************************************************************
void SysTickIntHandler(void) {
	//
	// Whatever you want to do
	//

}


//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port M interrupt event. For this
// application GPIO port M pin 3 is the interrupt line for the MPU9150
//
// For BoosterPack 2 Interface use Port M pin 7.
//
//*****************************************************************************
void
GPIOPortMIntHandler(void)
{
    unsigned long ulStatus;

	 UARTprintf("\r GPIOPortMIntHandler 1\n");
    //
    // Get the status flags to see which pin(s) caused the interrupt.
    //
    ulStatus = GPIOIntStatus(GPIO_PORTM_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTM_BASE, ulStatus);

    //
    // Check if this is an interrupt on the MPU9150 interrupt line.
    //
    // For BoosterPack 2 use Pin 7 instead.
    //
    //if(ulStatus & GPIO_PIN_3)
    if(ulStatus & GPIO_PIN_7)
    {
        //
        // Turn on the LED to show that transaction is starting.
        //
        LEDWrite(CLP_D3 | CLP_D4, CLP_D3);

        //
        // MPU9250 Data is ready for retrieval and processing.
        //
        //MPU9250DataRead(...);
    }
}

//*****************************************************************************
//
// This example demonstrates the use of the MPU9250
//
//*****************************************************************************

int main(void) {

	uint8_t pui8MACArray[8];

	FPULazyStackingEnable();
	FPUEnable();

	//
	// Make sure the main oscillator is enabled because this is required by
	// the PHY.  The system must have a 25MHz crystal attached to the OSC
	// pins. The SYSCTL_MOSC_HIGHFREQ parameter is used when the crystal
	// frequency is 10MHz or higher.
	//
	SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

	//
	// Run from the PLL at 120 MHz.
	//
	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
	SYSCTL_OSC_MAIN |
	SYSCTL_USE_PLL |
	SYSCTL_CFG_VCO_480), 120000000);

	//
	// Configure the device pins.
	//
	PinoutSet(false, false);

	//
	// Configure UART.
	//
	UARTStdioConfig(0, 115200, g_ui32SysClock);

	//
	// Clear the terminal and print banner.
	//

	UARTprintf("PAk's MPU9250 Tivaware example\n\n");

	//
	// START I2C TX in I2C2 port
	//
	ConfigI2C();

	//
	// Make an I2C Bus Scan to find any live device
	//
	UARTprintf("\r I2CBusScan I2C2_BASE\n");
	I2CBusScan(I2C2_BASE);

	bool MPU9150_or_MPU9250=false;//false for MPU9150 - true for MPU9250

	if (MPU9150_or_MPU9250)
	{
		//
		// Reset MPU9250
		//
		UARTprintf("\r ResetMPU9250 ResetMPU9250\n");
		ResetMPU9250();

		//
		// Setup MPU9250 (Calibration, Init, etc)
		//
		UARTprintf("\r Setup MPU9250\n");
		Setup();
	}
	else
	{

		//
		// Reset MPU9150
		//
		UARTprintf("\r ResetMPU9150 ResetMPU9150\n");
		ResetMPU9150();

		//
		// Setup MPU9250 (Calibration, Init, etc)
		//
		UARTprintf("\r Setup MPU9150\n");
		Setup_MPU9150();
	}


	//
	// Configure Port N1 for as an output for the animation LED.
	//
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

	//
	// Initialize LED to OFF (0)
	//
	MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, ~GPIO_PIN_1);

	//
	// Configure SysTick for a periodic interrupt.
	//
	MAP_SysTickPeriodSet(g_ui32SysClock / SYSTICKHZ);
	MAP_SysTickEnable();
	MAP_SysTickIntEnable();

	MAP_IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);

	GetAres();
	GetGres();
	GetMres();

	//
	// Loop forever.  All the work is done in interrupt handlers.
	//
	UARTprintf("LOOPING!!\n");
	while (1) {

		if (MPU9150_or_MPU9250)
			{
				UpdateData();
				//better via interrupt
				delayMS(6);    //MPU9250 is configured at 200Hz, so enough
			}
		else
		{
			UpdateData_MPU9150();
					delayMS(6);
		}



	}
}
