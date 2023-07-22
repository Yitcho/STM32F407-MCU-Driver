/*
 * 003led_button_it.c
 *
 *  Created on: Apr 2, 2023
 *      Author: Mahdi Kaffel
 */

#include <string.h>
#include "stm32f407xx.h"


void delay(void)
{
	for(uint32_t i = 0 ; i < 200000 ; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioButton;
	memset(&GpioButton,0,sizeof(GpioButton));
	memset(&GpioLed,0,sizeof(GpioLed));

	/* LED Configuration */
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);
	/* Button Configuration */
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	/* CLK enable and GPIO initialization */
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioButton);
	/* IRQ Configurations*/
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI_15);
	GPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);
	while(1);
	return 0;
}

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
