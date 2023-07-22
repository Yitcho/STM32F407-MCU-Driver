/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Jun 3, 2023
 *      Author: Mahdi Kaffel
 */
#include "stm32f407xx_rcc_driver.h"

uint16_t APB2_PreScaler[4] = {2,4,8,16};
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,126,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};


uint32_t RCC_GetPLLOutputClock()
{
	return 0;
}

uint32_t RCC_GetPCLK1Value(void) {
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, AHBP, APB1;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if (clksrc == 0) {
		SystemClk = 16000000;
	} else if (clksrc == 1) {
		SystemClk = 8000000;
	} else if (clksrc == 2) {
		SystemClk = RCC_GetPLLOutputClock();
	}
	// for AHB
	temp = ( RCC->CFGR >> 4) & 0xF;

	if (temp < 8) {
		AHBP = 1;
	} else {
		AHBP = AHB_PreScaler[temp - 8];
	}
	// for APB1
	temp = ( RCC->CFGR >> 10) & 0x7;

	if (temp < 4) {
		APB1 = 1;
	} else {
		APB1 = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / AHBP) / APB1;

	return pclk1;
}

uint32_t RCC_GetPCLK2Value(void) {
	uint32_t pclk2, SystemClk;

	uint8_t clksrc, temp, AHBP, APB2;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if (clksrc == 0) {
		SystemClk = 16000000;
	} else if (clksrc == 1) {
		SystemClk = 8000000;
	} else if (clksrc == 2) {
		SystemClk = RCC_GetPLLOutputClock();
	}
	// for AHB
	temp = ( RCC->CFGR >> 4) & 0xF;

	if (temp < 8) {
		AHBP = 1;
	} else {
		AHBP = AHB_PreScaler[temp - 8];
	}

	// for APB2
		temp = ( RCC->CFGR >> 13) & 0x7;

		if (temp < 4) {
			APB2 = 1;
		} else {
			APB2 = APB2_PreScaler[temp - 4];
		}
	pclk2 = (SystemClk / AHBP) / APB2;

	return pclk2;
}
