/*
 * stm32f407xx_spi.h
 *
 *  Created on: Apr 8, 2023
 *      Author: Mahdi Kaffel
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

#include "stm32f407xx.h"

/*
 *  Configuration structure for SPIx Peripheral
 */

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct
{

	SPI_RegDef_t 	*pSPIx; 				/* This holds the base address of the SPIx(x:0,1,2) */
	SPI_Config_t 	SPI_Config; 			/* This holds SPI configuration settings */
	uint8_t			*pTxBuffer;				/* To store the app. Tx buffer address */
	uint8_t			*pRxBuffer;				/* To store the app. Rx buffer address */
	uint32_t		TxLen;					/* To store Tx len */
	uint32_t		RxLen;					/* To store Rx len */
	uint8_t			TxState;				/* To store Tx state */
	uint8_t			RxState;				/* To store Rx state */
}SPI_Handle_t;


/*
 * @SPI_DEVICE_MODE
 * SPI possible modes
 */
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

/*
 * @SPI_BUS_CONFIG
 * SPI possible Bus Configs
 */
#define SPI_BUS_CONFIG_FD				1		/* Full Duplex */
#define SPI_BUS_CONFIG_HD				2		/* Half Duplex */
#define SPI_BUS_CONFIG_S_RXONLY			3		/* Simplex Receive only */

/*
 * @SPI_SCLK_SPEED
 * SPI Possible Clock Speed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_DFF
 * SPI Data Frame Format
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * @SPI_CPOL
 * Clock Polarity
 */
#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW					0

/*
 * @SPI_CPHA
 * Clock Phase
 */
#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW					0

/*
 * @SPI_SSM
 *  Software slave management
 */
#define SPI_SSM_EN						1
#define SPI_SSM_DI						0

/*
 * SPI Related status flags definitions
 */
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG					(1 << SPI_SR_BSY)

/*
 *  SPI Application states
 */
#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

/*
 *  SPI Application events
 */
#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3
#define SPI_EVENT_CRC_ERR				4

/**********************************************************************************
 * 							APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 **********************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi );

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 *  Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
/*
 *  Data Send and Receive Interrupt Mode
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 *  IRQ Configuration and ISR handling
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi );
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority );
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 *  Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx , uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);
void SPI_SSIConfig(SPI_RegDef_t *pSPIX , uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIX , uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);


#endif /* INC_STM32F407XX_SPI_H_ */
