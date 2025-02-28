/*
 * stm32NucleoF429xxUSART_Driver.c
 *
 *  Created on: Feb 28, 2025
 *      Author: Raghavender Dornala
 */
#include <stm32NucleoF429xxUSART_Driver.h>


void USART_PeriClockControl(USART_RegDef_t * pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
		else if (pUSARTx == UART7)
		{
			UART7_PCLK_EN();
		}
		else if (pUSARTx == UART8)
		{
			UART8_PCLK_EN();
		}
	}
	else
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
		else if (pUSARTx == UART7)
		{
			UART7_PCLK_DI();
		}
		else if (pUSARTx == UART8)
		{
			UART8_PCLK_DI();
		}
	}
}

void USART_PeripheralControl(USART_RegDef_t * pUSARTx, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		pUSARTx->USART_CR1 |= (1 << USART_ENABLE_BITPOS);
	}
	else
	{
		pUSARTx->USART_CR1 &= ~(1 << USART_ENABLE_BITPOS);
	}
}

uint8_t GetFlagStatus(USART_RegDef_t * pUSARTx, uint32_t StatusFlagName)
{
	if (pUSARTx->USART_SR & StatusFlagName)
	{
		return SET;
	}

	return RESET;
}

void USART_ClearFlag(USART_RegDef_t * pUSARTx, uint8_t StatusFlagName)
{
}





