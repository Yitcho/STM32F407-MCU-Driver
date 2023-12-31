/*
 * spi_tx_testing.c
 *
 *  Created on: Apr 10, 2023
 *      Author: Mahdi Kaffel
 */
#include <string.h>
#include "stm32f407xx.h"

//command codes
#define COMMAND_LED_CTRL			0x50
#define COMMAND_SENSOR_READ			0x51
#define COMMAND_LED_READ			0x52
#define COMMAND_PRINT				0x53
#define COMMAND_ID_READ				0x54

#define LED_ON		1
#define LED_OFF		0

// Arduino analog Pins
#define ANALOG_PIN0			0
#define ANALOG_PIN1			1
#define ANALOG_PIN2			2
#define ANALOG_PIN3			3
#define ANALOG_PIN4			4

// arduino Led
#define LED_PIN				9

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i++);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t	SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handle);
}

void GPIOA_Inits(void)
{
	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioButton);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}
	return 0;
}

int main(void)
{
	uint8_t dummy_write = 0xff ;
	uint8_t dummy_read ;
	// Initialize GPIO Pins to behave as SPI2 Pins
	SPI2_GPIOInits();
	// Initialize GPIO Pins to behave as button
	GPIOA_Inits();
	// Initialize SPI2 peripheral parameters
	SPI2_Inits();
	/*
	 * SSOE 1 Make NSS Output enable
	 * The NSS pin is automatically set low
	 * when SPE = 1 and high when SPE = 0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);
	while (1) {
		// wait till button is pressed
		while (! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		// to avoid button de-bouncing
		delay();
		// Enable SPI
		SPI_PeripheralControl(SPI2, ENABLE);
		// 1. CMD_LED_CTRL	<pin no(1)>       <value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];
		// send command
		SPI_SendData(SPI2, &commandcode, 1);
		// dummy read to clear RXNE bit
		SPI_ReceiveData(SPI2,&dummy_read,1);
		// send some dummy bits (1 byte) to fetch response from slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// read the ackbyte
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if(SPI_VerifyResponse(ackbyte))
		{
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2,args,2);
		}
		// end if COMMAND_LED_CTRL

		//2. CMD_SENSOR_READ <analog pin number (1)>
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		// to avoid button de-bouncing
		delay();
		// send command
		commandcode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &commandcode, 1);
		// dummy read to clear RXNE bit
		SPI_ReceiveData(SPI2,&dummy_read,1);
		// send some dummy bits (1 byte) to fetch response from slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// read the ackbyte
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)) {
			//send arguments
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);

			// dummy read to clear RXNE bit
			SPI_ReceiveData(SPI2, &dummy_read, 1);
			// insert delay so that slave can be ready with the data
			delay();
			// send some dummy bits (1 byte) to fetch response from slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}
		// end if COMMAND_SENSOR_READ

		// confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
		// Disable SPI
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}


