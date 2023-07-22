/*
 * stm32f407xx_i2c.h
 *
 *  Created on: Apr 24, 2023
 *      Author: Mahdi Kaffel
 */

#ifndef INC_STM32F407XX_I2C_H_
#define INC_STM32F407XX_I2C_H_

#include "stm32f407xx.h"
/*
 *  Configuration structure for I2Cx Peripheral
 */

typedef struct
{
	uint8_t I2C_ACKControl;
	uint8_t I2C_DeviceAddress;
	uint32_t I2C_SclkSpeed;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t	 *pI2Cx;
	I2C_Config_t	 I2C_Config;
	uint8_t 		 *pTxBuffer; /* To store the app. Tx buffer address */
	uint8_t 		 *pRxBuffer; /* To store the app. Rx buffer address */
	uint32_t 		 TxLen; /* To store Tx len */
	uint32_t 		 RxLen; /* To store Rx len */
	uint8_t			 TxRxState; /* To store Tx state */
	uint8_t			 DevAddr;
	uint32_t 		 RxSize;
	uint8_t			 Sr;
}I2C_Handle_t;

/*
 *  @I2C_SCLKSpeed
 */
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM4K		400000
#define I2C_SCL_SPEED_FM2K		200000

/*
 *  @I2C_ACKControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 *  @I2C_ACKControl
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/*
 * I2C Related status flags definitions
 */
#define I2C_FLAG_TXE					(1 << I2C_SR1_TxE)
#define I2C_FLAG_RXNE					(1 << I2C_SR1_RxNE)
#define I2C_FLAG_SB						(1 << I2C_SR1_SB)
#define I2C_FLAG_BTF					(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR					(1 << I2C_SR1_ADDR)
#define I2C_FLAG_ADDR10					(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF					(1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR					(1 << I2C_SR1_BERR)
#define I2C_FLAG_AF						(1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO					(1 << I2C_SR1_ARLO)
#define I2C_FLAG_TIMEOUT				(1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR		RESET
#define I2C_ENABLE_SR		SET

/*
 * I2C Application states
 */
#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2

/*
 *  I2C Application events
 */
#define I2C_EV_TX_CMPLT				0
#define I2C_EV_RX_CMPLT				1
#define I2C_EV_STOP					2
#define I2C_EV_DATA_REQ				8
#define I2C_EV_DATA_RCV				9
/*
 *  I2C Application error
 */
#define I2C_ERROR_BERR  			3
#define I2C_ERROR_ARLO 				4
#define I2C_ERROR_AF    			5
#define I2C_ERROR_OVR   			6
#define I2C_ERROR_TIMEOUT 			7


/**********************************************************************************
 * 							APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 **********************************************************************************/

/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi );

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 *  Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint8_t Len, uint8_t SlaveAddr , uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint8_t Len, uint8_t SlaveAddr , uint8_t Sr);

/*
 *  Data Send and Receive Interrupt Mode
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint8_t Len, uint8_t SlaveAddr , uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint8_t Len, uint8_t SlaveAddr , uint8_t Sr);

/*
 *  Data Send and Receive in Slave Mode
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2C , uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);
/*
 *  IRQ Configuration and ISR handling
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi );
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority );
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 *  Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx , uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
void I2C_ManagAcking(I2C_RegDef_t *pI2Cx , uint8_t EnOrDi);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2C, uint8_t EnOrDi);





/*
 * Application Callback
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);


#endif /* INC_STM32F407XX_I2C_H_ */



