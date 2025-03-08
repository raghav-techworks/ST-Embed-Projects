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
		pUSARTx->USART_CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->USART_CR1 &= ~(1 << USART_CR1_UE);
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
	pUSARTx->USART_SR &= ~StatusFlagName;
}

void USART_SetbaudRate(USART_RegDef_t * pUSARTx, uint32_t BaudRate)
{
	uint32_t tempReg = 0;
	uint32_t PCLKx;

	uint32_t usartdiv;
	uint32_t M_Part, F_Part;


	if (pUSARTx == USART1 || pUSARTx == USART6)
	{
		// Get APB2 PCLK value

	}
	else
	{
		// Get APB1 PCLK Value
	}

	//Check for OVER8 configuration bit
	if (pUSARTx->USART_CR1 & (USART_CR1_OVER8 << 1))
	{
		//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	}
	else
	{
		//OVER8 = 0 , over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 *BaudRate));
	}

	M_Part = usartdiv/100;
	tempReg |= M_Part << USART_BRR_DIV_MA;



	tempReg |= F_Part;

	pUSARTx->USART_BRR = tempReg;
}

void USART_InIt(USART_Handle_t * pUSARTHandle)
{
	if (pUSARTHandle != NULL)
	{
		uint32_t tempReg = 0;

		/*********************** Configuration of CR1 Register ************************/

		// Enable the clock for USART
		USART_PeripheralControl(pUSARTHandle->pUSARTx, ENABLE);

		// USART Mode
		if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
		{
			tempReg |= (1 << USART_CR1_TE);
		}
		else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
		{
			tempReg |= (1 << USART_CR1_RE);
		}
		else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
		{
			tempReg |= (1 << USART_CR1_RE) | (1 << USART_CR1_TE);
		}

		// Parity Control
		if (pUSARTHandle->USART_Config.USART_ParityControl == USART_EVEN_PARITY_ENABLE)
		{
			tempReg |= (1 << USART_CR1_PCE);

			// Enable Even Parity
			tempReg &= ~(1 << USART_CR1_PS);
		}
		else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_ODD_PARITY_ENABLE)
		{
			tempReg |= (1 << USART_CR1_PCE);

			// Enable Odd Parity
			tempReg |= (1 << USART_CR1_PS);
		}

		// USART Word Length
		tempReg = (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

		pUSARTHandle->pUSARTx->USART_CR1 = tempReg;

		/***************** Configuration of CR2 Register **************************/
		tempReg = 0;

		// Number of Stop bits
		tempReg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

		pUSARTHandle->pUSARTx->USART_CR2 = tempReg;

		/***************** Configuration of CR3 Register **************************/
		tempReg = 0;

		if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
		{
			tempReg |= (1 << USART_CR3_CTSE);
		}
		if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
		{
			tempReg |= (1 << USART_CR3_RTSE);
		}
		if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
		{
			tempReg |= (1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE);
		}

		pUSARTHandle->pUSARTx->USART_CR3 = tempReg;

		/***************** Configuration of BRR Register (Baudrate) **************************/
		USART_SetbaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
	}
}


void USART_Tx(USART_Handle_t * pUSARTHandle, uint8_t * pTxBuffer, uint32_t length)
{
	uint16_t *pData;

	for (uint32_t i = 0; i < length; i++)
	{
		while(!GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			pData = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->USART_DR = (*pData & (uint16_t)0x1FF);

			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				pTxBuffer++;
			}

		}
		else
		{
			pUSARTHandle->pUSARTx->USART_DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	while(!GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

void USART_Rx(USART_Handle_t * pUSARTHandle, uint8_t * pRxBuffer, uint32_t length)
{
	for (uint32_t i = 0; i < length; i++)
	{
		while(!GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->USART_DR & (uint16_t)0x01FF);
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				*pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
				pRxBuffer++;
			}

		}
		else
		{
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
			}
			else
			{
				*pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0x7F);
			}
			pRxBuffer++;
		}
	}
}

