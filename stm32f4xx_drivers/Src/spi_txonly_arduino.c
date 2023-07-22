/*
 * spi_tx_testing.c
 *
 *  Created on: Apr 10, 2023
 *      Author: Mahdi Kaffel
 */
#include <string.h>
#include "stm32f407xx.h"

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
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);
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


int main(void)
{
	char user_data[] = "Hello World , Testing My SPI Driver";
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
		// Send data length to arduino
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);
		// Send Data
		SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));
		// confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
		// Disable SPI
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}


