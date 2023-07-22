/*
 * usart_tx_string.c
 *
 *  Created on: Jun 3, 2023
 *      Author: Mahdi Kaffel
 */


#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

/******************GLOBAL VARIABLES**********************/
USART_Handle_t USART1_handle;
char *msg[1] = {"Testing UART Driver"};

char receive_buff[1024];

uint8_t rxCmplt = RESET;

/********************************************************/

void delay(void)
{
	for(int i=0; i < 500000; i++)
	{
		i++;
	}
}

void USART_Inits(void)
{
	USART1_handle.pUSARTx = USART1;
	USART1_handle.USART_config.USART_Baud = USART_STD_BAUD_115200;
	USART1_handle.USART_config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART1_handle.USART_config.USART_Mode = USART_MODE_TXRX;
	USART1_handle.USART_config.USART_ParityControl = USART_PARITY_DISABLE;
	USART1_handle.USART_config.USART_WordLength = USART_WORDLEN_8BITS;
	USART1_handle.USART_config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART_Init(&USART1_handle);
}

void USART1_GPIOInits(void)
{
	GPIO_Handle_t USARTpins;
	USARTpins.pGPIOx = GPIOB ;
	USARTpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTpins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USARTpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	USARTpins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	// TX Pin
	USARTpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&USARTpins);
	// RX pin
	USARTpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&USARTpins);
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
	uint32_t cnt = 0;

	GPIOA_Inits();

	USART1_GPIOInits();

	USART_Inits();

	USART_IRQConfig(IRQ_NO_USART1, ENABLE );

	USART_PeripheralControl(USART1, ENABLE);

	while (1) {
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		cnt = cnt % 3 ;

		while(USART_ReceiveDataIT(&USART1_handle, (uint8_t*)receive_buff, strlen(msg[cnt])) != USART_READY);

		USART_SendData(&USART1_handle, (uint8_t*)msg[cnt], strlen(msg[cnt]));

		//move on to next message indexed in msg[]
		cnt++;
	}

	return 0;
}

void USART1_IRQHandler(void)
{
	USART_IRQHandling(&USART1_handle);
}

void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
   if(ApEv == USART_EVENT_RX_CMPLT)
   {
			rxCmplt = SET;

   }else if (ApEv == USART_EVENT_TX_CMPLT)
   {
	   ;
   }
}
