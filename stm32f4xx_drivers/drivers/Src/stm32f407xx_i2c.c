/*
 * stm32f407xx_i2c.c
 *
 *  Created on: Apr 24, 2023
 *      Author: Mahdi Kaffel
 */
#include "stm32f407xx_i2c.h"


/********************** I2C Private functions **************************/
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx , uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx , uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);


/**************************************************************
 * @fn						- I2C_ExecuteAdressPhase
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx , uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/w bit set to w(0) (total 8 bits)
	pI2Cx->DR = SlaveAddr;

}

/**************************************************************
 * @fn						- I2C_ExecuteAdressPhase
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx , uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/w bit set to R(1) (total 8 bits)
	pI2Cx->DR = SlaveAddr;

}

/**************************************************************
 * @fn						- I2C_ClearADDRFlag
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//check device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		// device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if (pI2CHandle->RxSize == 1) {
				// Disable ACK
				I2C_ManagAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				// Clear ADDR flag
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void) dummy_read;
			}
		} else {
			// Clear ADDR flag
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void) dummy_read;
		}

	}else
	{
		// Device in Slave mode
		// Clear ADDR flag
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void) dummy_read;
	}
}
/**************************************************************
 * @fn						- I2C_Init
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;
	// Enable Clock Peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);
	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= ( RCC_GetPCLK1Value() / 1000000U ) ;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F) ;

	//Program the device own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14 );
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR Calculations
	uint16_t ccr_value = 0 ;
	tempreg = 0 ;
	if(pI2CHandle->I2C_Config.I2C_SclkSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = RCC_GetPCLK1Value() / (2 *  pI2CHandle->I2C_Config.I2C_SclkSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempreg |= 1 << 15 ;
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / ( 3 *  pI2CHandle->I2C_Config.I2C_SclkSpeed);
		}else
		{
			ccr_value = RCC_GetPCLK1Value() / ( 25 *  pI2CHandle->I2C_Config.I2C_SclkSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//	TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SclkSpeed <= I2C_SCL_SPEED_SM)
		{
			//mode is standard mode
			tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1 ;
		}else
		{
			//mode is Fast mode
			tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U)+ 1 ;
		}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F) ;

}
/**************************************************************
 * @fn						- I2C_PeriClockControl
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}

}

/**************************************************************
 * @fn						- I2C_DeInit
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}

/*
 *  IRQ Configuration and ISR handling
 */
/**************************************************************
 * @fn						- I2C_IRQConfig
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi )
 {
	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			//program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		} else if (IRQNumber > 64 && IRQNumber < 96) {
			//program ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	} else {
		if (IRQNumber <= 31) {
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			//program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		} else if (IRQNumber > 64 && IRQNumber < 96) {
			//program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}

	}
}

/**************************************************************
 * @fn						- I2C_IRQPriorityConfig
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority )
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*
 *  Other Peripheral Control APIs
 */

/**************************************************************
 * @fn						- I2C_PeripheralControl
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx , uint8_t EnOrDi)
 {
	if (EnOrDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/**************************************************************
 * @fn						- I2C_MasterSendData
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint8_t Len, uint8_t SlaveAddr ,uint8_t Sr)
{
	// 1. Generate Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// 2. Confirm that start generation is completed by checking the SB flag in the SR1
	// Note : Until SB is Cleared SCL Will be stretched (Pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_SB));
	// 3. Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx , SlaveAddr);
	// 4. confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_ADDR));
	// 5. Clear the ADDR flag according to its software sequence
	I2C_ClearADDRFlag(pI2CHandle);
	// 6. Send the data until Len becomes 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}
	// 7. When Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//	Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//	When BTF=1 SCL will be stretched (Pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_BTF));
	// 8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//	Note: Generating STOP , automatically clears the BTF
	if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

/**************************************************************
 * @fn						- I2C_MasterSendData
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint8_t Len, uint8_t SlaveAddr , uint8_t Sr)
{
	// 1. Generate Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// 2. Confirm that start generation is completed by checking the SB flag in the SR1
	// Note : Until SB is Cleared SCL Will be stretched (Pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_SB));
	// 3. Send the address of the slave with r/w bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx , SlaveAddr);
	// 4. confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_ADDR));
	// Procedure to read only 1 byte from slave
	if(Len == 1)
	{
		// Disable Acking
		I2C_ManagAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		// Wait Until RXNE becomes
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_RXNE));
		// Generate Stop Condition
		if(Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		// Read data in to buffer
		 *pRxbuffer = pI2CHandle->pI2Cx->DR;
	}

	if(Len > 1)
	{
		// Clear the ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle);
		// Read data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			// Wait until RXNE becomes 1
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
			// if last 2 bytes are remaining
			if (i == 2) {
				// Clear the Ack bit
				I2C_ManagAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				// Generate stop condition
				if(Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// Read the data from data register in to buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;
			// Increment the buffer address
			pRxbuffer++;
		}
	}
	// re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManagAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/**************************************************************
 * @fn						- I2C_GetFlagStatus
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
	if (pI2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**************************************************************
 * @fn						- I2C_ManagAcking
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_ManagAcking(I2C_RegDef_t *pI2Cx , uint8_t EnOrDi)
{
	if(EnOrDi == I2C_ACK_ENABLE)
	{
		// Enable Ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);

	}else
	{
		// Disable Ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

	}
}

/**************************************************************
 * @fn						- I2C_MasterSendDataIT
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint8_t Len, uint8_t SlaveAddr , uint8_t Sr)
 {
	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_RX) && (busystate != I2C_BUSY_IN_TX)) {
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		// Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		// Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		// Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/**************************************************************
 * @fn						- I2C_MasterReceiveDataIT
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint8_t Len, uint8_t SlaveAddr , uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_RX) && (busystate != I2C_BUSY_IN_TX)) {
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		// Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		// Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		// Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		// Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/**************************************************************
 * @fn						- I2C_MasterHandleRXNEInterrupt
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
 {
	if (pI2CHandle->RxSize == 1) {
		// Load data into DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		// Decrement TXlen
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize > 1) {
		if (pI2CHandle->RxSize == 2) {
			// Clear ACK bit
			I2C_ManagAcking(pI2CHandle->pI2Cx, DISABLE);
		}
		// Load data into DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		// Increment buffer address
		pI2CHandle->pRxBuffer++;
		// Decrement TXlen
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxLen == 0) {
		// Close the I2C data reception and notify the application

		// 1. Generate the stop condition
		if (pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		// 2. Close the I2C Rx
		I2C_CloseReceiveData(pI2CHandle);
		// 3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxLen > 0) {
		// Load data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		// Decrement TXlen
		pI2CHandle->TxLen--;
		// Increment buffer address
		pI2CHandle->pTxBuffer++;
	}
}
/**************************************************************
 * @fn						- I2C_SlaveSendData
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2C , uint8_t data)
{
	pI2C->DR = data;
}
/**************************************************************
 * @fn						- I2C_SlaveReceiveData
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR;
}
/**************************************************************
 * @fn						- I2C_EV_IRQHandling
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	//1. Handle for interrupt generated by SB event
	// Note : SB Flag only applicable in Master Mode
	if (temp1 && temp3) {
		// Interrupt is generated because of SB Flag
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}
	//2. Handle for interrupt generated by ADDR event
	//Note : When Master Mode : Address is sent
	//		 When Slave mode : Address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

	if (temp1 && temp3) {
		// ADDR Flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}
	//3. Handle for interrupt generated by BTF event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

	if (temp1 && temp3) {
		// BTF Flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			// make sure that TXE is also set
			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE)) {
				// BTF , TXE = 1
				if (pI2CHandle->TxLen == 0) {
					//1. Generate the STOP condition
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					//2. Reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);
					//3. Notify the application about transsmisiom complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE)) {
				// make sure that RXNE is also set

			}
		}
	}

	//4. Handle for interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For Master this flag will
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);

	if (temp1 && temp3) {
		// STOPF Flag is set
		// Clear the STOPF
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//3. Notify the application about STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}
	//5. Handle for interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);

	if (temp1 && temp3 && temp2) {
		// Check for device mode Master/Slave
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			// TXE Flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			// Slave
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}
	//6. Handle for interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);

	if (temp1 && temp3 && temp2) {
		// Check device mode
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			// RXNE Flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}else
		{
			if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}
/**************************************************************
 * @fn						- I2C_ER_IRQHandling
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1 , temp2;
	// Know the status of ITERREN
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN);
	/****************** Check for Bus Error *****************/
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		// this is bus error
		// Implement the code to clear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}
	/****************** Check for arbitration lost error *****************/
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO);
	if (temp1 && temp2)
	{
		// this is arbitration lost error
		// Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}
	/****************** Check for ACK failure  error *********************/
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF);
	if (temp1 && temp2)
	{
		// this is ACK failure  error
		// Implement the code to clear the ACK failure  error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}
	/****************** Check for Overrun/underrun error *********************/
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR);
	if (temp1 && temp2)
	{
		// this is Overrun/underrun error
		// Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}
	/****************** Check for Time out error ****************************/
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT);
	if (temp1 && temp2)
	{
		// this is Overrun/underrun error
		// Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}

/**************************************************************
 * @fn						- I2C_CloseSendData
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	// Disable ITEVTEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/**************************************************************
 * @fn						- I2C_CloseReceiveData
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );
	// Disable ITEVTEN
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxSize = 0;
	pI2CHandle->RxLen = 0;
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ManagAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);

}

/**************************************************************
 * @fn						- I2C_GenerateStartCondition
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}

/**************************************************************
 * @fn						- I2C_GenerateStopCondition
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral
 * @param[in]			    - ENABLE or DISABLE Macros		    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}


void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2C, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2C->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2C->CR2 |= (1 << I2C_CR2_ITERREN);
		pI2C->CR2 |= (1 << I2C_CR2_ITEVTEN);
	}else
	{
		pI2C->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2C->CR2 &= ~(1 << I2C_CR2_ITERREN);
		pI2C->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	}
}

__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandlee,uint8_t event)
{

}
