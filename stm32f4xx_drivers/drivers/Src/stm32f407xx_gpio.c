/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Apr 1, 2023
 *      Author: Mahdi Kaffel
 */

#include "stm32f407xx_gpio.h"


/**************************************************************
 * @fn						- GPIO_PeriClockControl
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
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi )
 {
	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}

}

/**************************************************************
 * @fn						- GPIO_Init
 *
 * @brief					- This function Initialize given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral	    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx , ENABLE);

	//1 . Configure the mode of GPIO Pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Non Interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		// Interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// Configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// Configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// configure both RTSR and FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		// Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);
		// Enable EXTI Interrupt delivery Using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


	}
	temp = 0;
	//2 . Configure the speed of GPIO Pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;
	//3 . Configure the Pull Up and Pull Down Settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	//4 . Configure the Output Type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	//5 . Configure the alt functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xf << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));

	}
}

/**************************************************************
 * @fn						- GPIO_Init
 *
 * @brief					- This function Initialize given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral	    -
 *
 * @return				    - None
 *
 * @Note					- None

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}

/*
 *  Data Read and Write
 */
/**************************************************************
 * @fn						- GPIO_Init
 *
 * @brief					- This function Initialize given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral	    -
 * @param[in]
 * @return				    - None
 *
 * @Note					- None

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}

/**************************************************************
 * @fn						- GPIO_ReadFromInputPort
 *
 * @brief					- This function Initialize given GPIO port
 *
 * @param[in]			    - Point
 *
 * @return				    - None
 *
 * @Note					- None

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/**************************************************************
 * @fn						- GPIO_Init
 *
 * @brief					- This function Initialize given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral	    -
 *
 * @return				    - None
 *
 * @Note					- None
 *
 */
void GPIO_WriteToOutputtPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/**************************************************************
 * @fn						- GPIO_WriteToOutputtPort
 *
 * @brief					- This function Initialize given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral	    -
 *
 * @return				    - None
 *
 * @Note					- None
 *
 */
void GPIO_WriteToOutputtPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
/**************************************************************
 * @fn						- GPIO_ToggleOutputPin
 *
 * @brief					- This function Initialize given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral	    -
 *
 * @return				    - None
 *
 * @Note					- None
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber);
}


/*
 *  IRQ Configuration and ISR handling
 */
/**************************************************************
 * @fn						- GPIO_IRQConfig
 *
 * @brief					- This function Initialize given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral	    -
 *
 * @return				    - None
 *
 * @Note					- None
 *
 ***************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi )
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
 * @fn						- GPIO_IRQPriorityConfig
 *
 * @brief					- This function Initialize given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral	    -
 *
 * @return				    - None
 *
 * @Note					- None
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4 ;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASEADDR + iprx ) |= ( IRQPriority << shift_amount );
}
/**************************************************************
 * @fn						- GPIO_IRQHandling
 *
 * @brief					- This function Initialize given GPIO port
 *
 * @param[in]			    - base address of the GPIO Peripheral	    -
 *
 * @return				    - None
 *
 * @Note					- None
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}
