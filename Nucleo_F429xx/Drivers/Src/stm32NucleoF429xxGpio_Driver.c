/*
 * stm32NucleoF429xxGpio_Driver.c
 *
 *  Created on: 16-Jan-2024
 *      Author: Raghavender Dornala
 */

#include <stm32NucleoF429xxGpio_Driver.h>





/*
 *  Peripheral clock
 */

void GPIO_PeriClockControl(GPIO_RegDef_t * pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
		else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		}
		else
		{
			GPIOK_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
		else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_DI();
		}
		else
		{
			GPIOK_PCLK_DI();
		}
	}
}


/*
 *  GPIO port InIt and DeInIt
 *  requires pin details and configuration to enable particular pin
 */
void GPIO_InIt(GPIO_CofigHandle_t * pGPIOHandle)
{
	//1. Configure the mode of GPIO
	uint32_t temp;

	if (pGPIOHandle->GPIOPinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		/* set pin mode based on the pin number. multiply 2 because each pin has two bits*/
		temp = pGPIOHandle->GPIOPinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOxBaseAddr->MODER &= ~(0x3 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber); // make sure clear the corresponding bits to set
		pGPIOHandle->pGPIOxBaseAddr->MODER |= temp;   /* use | to avoid effect in other bits, we need to change only the bits related to pin number */

	}
	else // Configure Interrupt in peripheral side
	{
		/* operate interrupt type mode */
		// 1. Configure the Edge Trigger
		if (pGPIOHandle->GPIOPinConfig.GPIO_PinMode <= GPIO_MODE_ITFT)
		{
			// configure FTSR for the selected pin
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
			// clear RTSR for selected pin (good practise)
			EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIOPinConfig.GPIO_PinMode <= GPIO_MODE_ITRT)
	    {
			// configure RTSR for the selected pin
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
			// clear FTSR for selected pin (good practise)
			EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
	    }
		else if (pGPIOHandle->GPIOPinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
		}

		// 2.Configure GPIO port selection in SYSCFG_EXTICR

		uint8_t temp1 = pGPIOHandle->GPIOPinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIOPinConfig.GPIO_PinNumber % 4;
		uint8_t PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOxBaseAddr);

		SYSCFG_PCLK_EN();
		SYSCFG->SYSCFG_EXTICR[temp1] |= (PortCode << (temp2 * 4 ) );

		//3. Enable the external interrupt delivery in IMR
		EXTI->EXTI_IMR |= (1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
	}

	//2. Configure the Speed
	temp = 0;
	temp = pGPIOHandle->GPIOPinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOxBaseAddr->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOxBaseAddr->OSPEEDR |= temp;


	//3. Configure the PuPd settings
	temp = 0;
	temp = pGPIOHandle->GPIOPinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOxBaseAddr->PUPDR &= ~(0x3 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOxBaseAddr->PUPDR |= temp;

	//4. Configure the OpType
	temp = 0;
	temp = pGPIOHandle->GPIOPinConfig.GPIO_PinOPType << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOxBaseAddr->OTYPER &= ~(0x1 << pGPIOHandle->GPIOPinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOxBaseAddr->OTYPER |= temp;

	//5. Configure the alternate functionality

	if (pGPIOHandle->GPIOPinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
	{
		uint8_t temp1 = (pGPIOHandle->GPIOPinConfig.GPIO_PinNumber / 8);
		uint8_t temp2 = (pGPIOHandle->GPIOPinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOxBaseAddr->AFRL[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOxBaseAddr->AFRL[temp1] |= (pGPIOHandle->GPIOPinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

// register (RCC_AHB1RSTR - if you set this bit then all the GPIOx register will get reset)
void GPIO_DeInIt(GPIO_RegDef_t * pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
	else if (pGPIOx == GPIOJ)
	{
		GPIOJ_REG_RESET();
	}
	else
	{
		GPIOK_REG_RESET();
	}
}


/*
 *  Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber)				// reading single pin out of 16 pins
{
	uint8_t Value;
	Value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return Value;
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t * pGPIOx)								// Port is of 16 pins
{
	uint8_t Value;
	Value = (uint16_t)pGPIOx->IDR;

	return Value;
}


void GPIO_WriteToOutputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber, uint8_t Value)	// writing to a particular pin of 1 Byte
{
	if(Value == SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t * pGPIOx, uint16_t Value)					// writing to a port of 2 bytes for 16 pins
{
	pGPIOx->ODR = Value;
}


void GPIO_ToggleOutputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 *  Interrupt handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
			*NVIC_ICER1 &= ~(1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
		else if (IRQNumber >= 96 && IRQNumber < 110)
		{
			*NVIC_ISER3 |= (1 << (IRQNumber % 96));
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
		else if (IRQNumber >= 96 && IRQNumber < 110)
		{
			*NVIC_ICER3 |= (1 << (IRQNumber % 96));
		}
	}


}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t ShiftAmount = (8 * iprx_section) + (8 - No_OF_PRI_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << ShiftAmount);
}


void GPIO_IRQHandling(uint8_t PinNumber)		// PinNumber to know from which pin IRQ is generated
{
	if (EXTI->EXTI_PR & (1 << PinNumber))		// I pending register set then clear
	{
		EXTI->EXTI_PR |= (1 << PinNumber);   // write 1 to pending register to clear the interrupt
	}
}
