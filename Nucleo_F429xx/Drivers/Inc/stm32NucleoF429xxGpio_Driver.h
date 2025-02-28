/*
 * stm32NucleoF429xxGpio_Driver.h
 *
 *  Created on: 16-Jan-2024
 *      Author: dgsma
 */

#ifndef INC_STM32NUCLEOF429XXGPIO_DRIVER_H_
#define INC_STM32NUCLEOF429XXGPIO_DRIVER_H_

#include <stm32NucleoF429xx.h>

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


typedef struct
{
	GPIO_RegDef_t * pGPIOxBaseAddr;		/* Holds the base address of any GPIO port */
	GPIO_PinConfig_t GPIOPinConfig;
}GPIO_CofigHandle_t;


/*
 * @GPIO MODE TYPES
 * GPIO port Possible modes
 */
#define GPIO_MODE_INPUT		0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_ALTFUN	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_ITFT		4
#define GPIO_MODE_ITRT		5
#define GPIO_MODE_IT_RFT	6

/* GPIO Port Output types */
#define GPIO_OP_PP			0
#define GPIO_OP_OD			1

/* GPIO Port Output speed */
#define GPIO_LOW_SP			0
#define GPIO_MED_SP			1
#define GPIO_HI_SP			2
#define GPIO_VHI_SP			3

/* GPIO Pull-Up and Pull-Down*/
#define GPIO_NO_PU_PD		0
#define GPIO_PU				1
#define GPIO_PD				2
#define GPIO_RESERVED		3

/* GPIO Alternate functions */

#define GPIO_ALTFUN_0		0
#define GPIO_ALTFUN_1		1
#define GPIO_ALTFUN_2		2
#define GPIO_ALTFUN_3		3
#define GPIO_ALTFUN_4		4
#define GPIO_ALTFUN_5		5
#define GPIO_ALTFUN_6		6
#define GPIO_ALTFUN_7		7
#define GPIO_ALTFUN_8		8
#define GPIO_ALTFUN_9		9
#define GPIO_ALTFUN_10		10
#define GPIO_ALTFUN_11		11
#define GPIO_ALTFUN_12		12
#define GPIO_ALTFUN_13		13
#define GPIO_ALTFUN_14		14
#define GPIO_ALTFUN_15		15


#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15


/*
 *  Peripheral clock
 */
void GPIO_PeriClockControl(GPIO_RegDef_t * pGPIOx, uint8_t EnorDi);

/*
 *  GPIO port InIt and DeInIt
 */
void GPIO_InIt(GPIO_CofigHandle_t * pGPIOHandle);	// requires pin details and configuration to enable particulat pin
void GPIO_DeInIt(GPIO_RegDef_t * pGPIOx);			// register (RCC_AHB1RSTR - if you set this bit then all the GPIOx register will get reset)


/*
 *  Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber);				// reading single pin out of 16 pins
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t * pGPIOx);								// Port is of 16 pins
void GPIO_WriteToOutputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber, uint8_t Value);	// writing to a particular pin of 1 Byte
void GPIO_WriteToOutputPort(GPIO_RegDef_t * pGPIOx, uint16_t Value);					// writing to a port of 2 bytes for 16 pins
void GPIO_ToggleOutputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber);


/*
 *  Interrupt handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);		// PinNumber to know from which pin IRQ is generated



















#endif /* INC_STM32NUCLEOF429XXGPIO_DRIVER_H_ */
