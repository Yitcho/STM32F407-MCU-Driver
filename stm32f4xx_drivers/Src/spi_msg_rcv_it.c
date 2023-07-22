/*
 * spi_msg_rcv_it.c
 *
 *  Created on: Jul 22, 2023
 *      Author: Mahdi Kaffel
 */

/*
 * This application receives and prints the user message received from the Arduino peripheral in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV itm data console
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"


void SPI2_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

SPI_Handle_t SPI2handle;

#define MAX_LEN	500

char RcvBuff[MAX_LEN];

char RcvBuff[MAX_LEN];


volatile uint8_t rcvStop = 0;

volatile char ReadByte;

volatile uint8_t dataAvailable = 0 ;

void delay()
{
	for(int32_t i=0 ; i < 500000/2 ; i++);
}

void SPI_GPIO()
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
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

void SPI_inits()
{
	SPI2handle.pSPIx = SPI2;

	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI; //Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}

/*This function configures the gpio pin over which SPI peripheral issues data available interrupt */
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin,0,sizeof(spiIntPin));

	//this is led gpio configuration
	spiIntPin.pGPIOx = GPIOD;
	spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&spiIntPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI_15);
	GPIO_IRQConfig(IRQ_NO_EXTI9_5,ENABLE);

}


int main ()
{
	uint8_t dummy = 0xff;

	Slave_GPIO_InterruptPinInit();
	SPI_inits();
	SPI_GPIO();

	SPI_SSOEConfig(SPI2, ENABLE);
	SPI_IRQConfig(IRQ_NO_SPI2, ENABLE);

	while(1)
	{
		rcvStop = 0;

		while(!dataAvailable);

		GPIO_IRQConfig(IRQ_NO_EXTI9_5, DISABLE);

		SPI_PeripheralControl(SPI2, ENABLE);

		while (!rcvStop) {
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while (SPI_SendDataIT(&SPI2handle, &dummy, 1) == SPI_BUSY_IN_TX);
			while (SPI_ReceiveDataIT(&SPI2handle,(uint8_t*) &ReadByte, 1) == SPI_BUSY_IN_RX);
		}

		while (SPI_GetFlagStatus( SPI2, SPI_BSY_FLAG))
			;

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		dataAvailable = 0;

		GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);

	}

	return 0;
}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI2_IRQHandler(void)
{

	SPI_IRQHandling(&SPI2handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv) {
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if (AppEv == SPI_EVENT_RX_CMPLT) {
		RcvBuff[i++] = ReadByte;
		if (ReadByte == '\0' || (i == MAX_LEN)) {
			rcvStop = 1;
			RcvBuff[i - 1] = '\0';
			i = 0;
		}
	}

}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}
