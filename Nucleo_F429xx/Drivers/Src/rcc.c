/*
 * rcc.c
 *
 *  Created on: Feb 28, 2025
 *      Author: Raghavender Dornala
 */

#include "stm32NucleoF429xx.h"
#include <stdbool.h>


#define RCC_CSR_LSION_BITPOS			0
#define RCC_CSR_LSIRDY_BITPOS			1

#define RCC_CSR_LSEON_BITPOS			0
#define RCC_CSR_LSERDY_BITPOS			1


/* LSE Oscillator */

static inline void RCC_LSE_Enable(void)
{
	RCC->BDCR |= (1 << RCC_CSR_LSEON_BITPOS);
}

static inline bool RCC_LSE_IsReady(void)
{
	if ((RCC->BDCR & (1 << RCC_CSR_LSERDY_BITPOS)) == (RCC_CSR_LSERDY_BITPOS))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

static inline void RCC_LSE_Disable(void)
{
	RCC->BDCR &= ~(1 << RCC_CSR_LSEON_BITPOS);
}



/* LSI Oscillator */


static inline bool RCC_LSI_IsReady(void)
{
	if ((RCC->CSR & (1 << RCC_CSR_LSIRDY_BITPOS)) == (RCC_CSR_LSIRDY_BITPOS))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

static inline void RCC_LSI_Disable(void)
{
	RCC->CSR &= ~(1 << RCC_CSR_LSION_BITPOS);
}

static inline void RCC_LSI_Enable(void)
{
	RCC->CSR |= (1 << RCC_CSR_LSION_BITPOS);

	while(!RCC_LSI_IsReady());
}


