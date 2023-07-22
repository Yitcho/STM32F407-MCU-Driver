/*
 * spi_txonly_arduino.c
 *
 *  Created on: Apr 15, 2023
 *      Author: Mahdi Kaffel
 */

/*
 * spi_tx_testing.c
 *
 *  Created on: Apr 10, 2023
 *      Author: Mahdi Kaffel
 */
#include <string.h>
#include "stm32f407xx.h"

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


int main(void)
{
	char user_data[] = "Hello World";
	// Initialize GPIO Pins to behave as SPI2 Pins
	SPI2_GPIOInits();
	// Initialize SPI2 peripheral parameters
	SPI2_Inits();
	// Enable Internal slave select
	SPI_SSIConfig(SPI2, ENABLE);
	// Enable SPI
	SPI_PeripheralControl(SPI2, ENABLE);
	// Send Data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
	// confirm SPI is not busy
	while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
	// Disable SPI
	SPI_PeripheralControl(SPI2, DISABLE);
	while(1);

	return 0;
}


