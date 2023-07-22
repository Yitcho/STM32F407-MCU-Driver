/*
 * i2c_master_tx_testing.c
 *
 *  Created on: May 13, 2023
 *      Author: Mahdi Kaffel
 */

#include <string.h>
#include <stdio.h>

#include "stm32f407xx.h"

#define MY_ADDR		0x61
#define SLAVE_ADDR	0x68

// FLag


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i++);
}

I2C_Handle_t I2C1handle;

uint8_t receive_buff[32];

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t	I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);
	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{

	I2C1handle.pI2Cx = I2C1;
	I2C1handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1handle.I2C_Config.I2C_SclkSpeed = I2C_SCL_SPEED_SM;
	I2C_Init(&I2C1handle);
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
	uint8_t commandcode;

	uint8_t len;

	GPIOA_Inits();

	I2C1_GPIOInits();

	I2C1_Inits();
	// I2C IRQ configurations
	I2C_IRQConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_ER, ENABLE);
	// Enable I2C
	I2C_PeripheralControl(I2C1, ENABLE);
	// Ack bit is made 1 after PE = 1 ;
	I2C_ManagAcking(I2C1, I2C_ACK_ENABLE);

	while(1)
	{
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		// to avoid button de-bouncing
		delay();

		commandcode = 0x51;

		while(I2C_MasterSendDataIT(&I2C1handle,&commandcode,1, SLAVE_ADDR , I2C_ENABLE_SR) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1handle, &len, 1, SLAVE_ADDR , I2C_ENABLE_SR)!= I2C_READY);

		commandcode = 0x52;

		while(I2C_MasterSendDataIT(&I2C1handle,&commandcode,1, SLAVE_ADDR , I2C_ENABLE_SR) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1handle, receive_buff, len , SLAVE_ADDR , I2C_DISABLE_SR) != I2C_READY);


	}
}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1handle);

}
// In case of semihosting usage the event call back is helpful for tracing and debugging
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	if(AppEv == I2C_EV_TX_CMPLT)
	{
		printf("Tx is completed\n");
	}else if (AppEv == I2C_EV_RX_CMPLT)
	{
		printf("Rx is completed\n");
	}else if (AppEv == I2C_ERROR_AF)
	{
		printf("Error : Ack failure\n");
		// In Master Ack failure when slave fails to send ack for the byte
		// sent from the master.

		I2C_CloseSendData(&I2C1handle);
		// Generate stop condition
		I2C_GenerateStopCondition(I2C1);
		// hang in infinite loop
		while(1);
	}
}
