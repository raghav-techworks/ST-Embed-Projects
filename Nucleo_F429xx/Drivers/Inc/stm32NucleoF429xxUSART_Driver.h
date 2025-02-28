/*
 * stm32NucleoF429xxUSART_Driver.h
 *
 *  Created on: Feb 28, 2025
 *      Author: Raghavender Dornala
 */

#ifndef INC_STM32NUCLEOF429XXUSART_DRIVER_H_
#define INC_STM32NUCLEOF429XXUSART_DRIVER_H_

#include "stm32NucleoF429xx.h"
/*
 * Configuration Structure for USARTx peripheral
 */

typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

/*
 * Handle Structure for USARTx peripheral
 */

typedef struct
{
	USART_RegDef_t * pUSARTx;
	USART_Config_t USART_Config;
}USART_Handle_t;


#define USART_ENABLE_BITPOS				13				// Register CR1


/* USART_Mode */


/* USART_Baud when PCLK = 16MHz*/
#define USART_STD_BAUD_1200							1200
#define USART_STD_BAUD_2400							2400
#define USART_STD_BAUD_9600							9600
#define USART_STD_BAUD_19200						19200
#define USART_STD_BAUD_38400						38400
#define USART_STD_BAUD_57600						57600
#define USART_STD_BAUD_115200						115200
#define USART_STD_BAUD_230400						230400
#define USART_STD_BAUD_460800						460800
#define USART_STD_BAUD_921600						921600
#define USART_STD_BAUD_2M							2000000
#define USART_STD_BAUD_3M							3000000


/* USART_NoOfStopBits	-> CR2, BitPos-13:12*/
#define STOP_BIT_1						0
#define STOP_BIT_0_5					1
#define STOP_BIT_2						2
#define STOP_BIT_1_5					3


/* USART_WordLength */
#define USART_WORDLEN_8BITS				0
#define USART_WORDLEN_9BITS				1


/* USART_Parity Control Enable	->	CR1, BitPos-10*/
#define PARITY_CTL_ENABLE				1
#define PARITY_CTL_DISABLE				0

/* Parity Selection -> CR1, BitPos-9*/
#define EVEN_PARITY						0
#define ODD_PARITY						1

/* USART_HWFlowControl */
#define USART_HW_FLOW_CTRL_NONE				0
#define USART_HW_FLOW_CTRL_CTS				1
#define USART_HW_FLOW_CTRL_RTS				2
#define USART_HW_FLOW_CTRL_CTS_RTS			3


/******************************************API's Supported by USART Driver**************************/

/* USART Peripheral clock */
void USART_PeriClockControl(USART_RegDef_t * pUSARTx, uint8_t EnorDi);

/* USART port InIt and DeInIt */
void USART_InIt(USART_Handle_t * pUSARTHandle);	// requires pin details and configuration to enable particular pin
void USART_DeInIt(USART_RegDef_t * pUSARTx);			// register ()


/* USART port Data Send and Receive API's */
void USART_Tx(USART_RegDef_t * pUSARTx, uint8_t * pTxBuffer, uint32_t length);
void USART_Rx(USART_RegDef_t * pUSARTx, uint8_t * pRxBuffer, uint32_t length);
void USART_TxIT(USART_Handle_t * pUSARTHandle, uint8_t * pTxBuffer, uint32_t length);
void USART_RxIT(USART_Handle_t * pUSARTHandle, uint8_t * pRxBuffer, uint32_t length);


/* Interrupt handling */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t * pUSARTHandle);		// PinNumber to know from which pin IRQ is generated


/* Other Peripheral Control APIs */
void USART_PeripheralControl(USART_RegDef_t * pUSARTx, uint8_t EnorDi);
uint8_t GetFlagStatus(USART_RegDef_t * pUSARTx, uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t * pUSARTx, uint8_t StatusFlagName);

/* Application Callback */
void USART_ApplicationEventCallback(USART_Handle_t * pUSARTHandle, uint8_t AppEv);


#endif /* INC_STM32NUCLEOF429XXUSART_DRIVER_H_ */
