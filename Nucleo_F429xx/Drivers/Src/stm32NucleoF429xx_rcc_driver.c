/*
 * stm32NucleoF429xx_rcc_driver.c
 *
 *  Created on: Mar 10, 2025
 *      Author: dgsma
 */

#include "stm32NucleoF429xx_rcc_driver.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};



uint32_t RCC_GetPCLK1Value()
{
	uint32_t PCLK1_Value;

	uint8_t Clk_Src, Ahb_Pre, Apb_Pre;
	uint32_t System_Clk;

	Clk_Src = ((RCC->CFGR >> 2) & 0x3);

	if (Clk_Src == 0)				// HSI
	{
		System_Clk = 16000000;
	}
	else if(Clk_Src == 1)			//HSE
	{
		System_Clk = 8000000;
	}
	else if (Clk_Src == 2)			//PLL
	{

	}

	uint8_t temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8)		// As per data sheet clock cannot be divided if AHB pre-scaler less than 8
	{
		Ahb_Pre = 1;
	}
	else
	{
		Ahb_Pre = AHB_PreScaler[temp - 8];
	}

	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 4)		// As per data sheet clock cannot be divided if APB pre-scaler less than 4
	{
		Apb_Pre = 1;
	}
	else
	{
		Apb_Pre = APB1_PreScaler[temp - 4];
	}

	PCLK1_Value = (((System_Clk) / Ahb_Pre) / Apb_Pre);

	return PCLK1_Value;
}



/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClock=0,tmp,pclk2;

	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clk_src == 0)
	{
		SystemClock = 16000000;
	}else
	{
		SystemClock = 8000000;
	}
	tmp = (RCC->CFGR >> 4 ) & 0xF;

	if(tmp < 0x08)
	{
		ahbp = 1;
	}else
	{
       ahbp = AHB_PreScaler[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;
	if(tmp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PreScaler[tmp-4];
	}

	pclk2 = (SystemClock / ahbp )/ apb2p;

	return pclk2;
}

uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}
