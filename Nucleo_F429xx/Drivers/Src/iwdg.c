/*
 * iwdg.c
 *
 *  Created on: Feb 28, 2025
 *      Author: Raghavender Dornala
 */

#include "iwdg.h"


void IWDG_AccessCommand(uint16_t IWDG_WriteAccess)
{
	IWDG->IWDG_KR = IWDG_WriteAccess;
}

void IWDG_SetPrescaler(uint8_t IWDG_Prescaler)
{
	IWDG->IWDG_PR = IWDG_Prescaler;
}

void IWDG_SetReload(uint16_t Reload)
{
	IWDG->IWDG_RLR = Reload;
}

void IWDG_ReloadCounter(void)
{
	IWDG->IWDG_KR = IWDG_RELOAD_VALUE;
}

void IWDG_Enable(void)
{
	IWDG->IWDG_KR = IWDG_ENABLE_VALUE;
}

FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_Flag)
{
	FlagStatus bitStatus = SR_RESET;

	if ((IWDG->IWDG_SR & IWDG_Flag)!= (uint32_t)SR_RESET)
	{
		bitStatus = SR_SET;
	}
	else
	{
		bitStatus = SR_RESET;
	}

	return bitStatus;
}






