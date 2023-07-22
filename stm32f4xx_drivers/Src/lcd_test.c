/*
 * lcd_test.c
 *
 *  Created on: Jun 10, 2023
 *      Author: Mahdi Kaffel
 */
#include "i2c-lcd.h"
#include "stdio.h"

char buffer[10];
float num = 1234;

void gpio_init(void)
{
	GPIO_Handle_t button;
	button.pGPIOx = GPIOA;
	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&button);
}

int main(void)
{
	lcd_init();
	lcd_clear();

	sprintf(buffer, "val=%d", num);
	lcd_put_cur(0, 0);
	lcd_send_string(buffer);
	return 0;
}
