/*
 * watchdog.c
 *
 *  Created on: Feb 28, 2025
 *      Author: Raghavender Dornala
 */

#include "watchdog.h"
#include "rcc.c"



static void Hardware_IWDGInit(Watchdog_Device_t * watchdog);
static void Hardware_IWDGKick(Watchdog_Device_t * watchdog);



static const Watchdog_Ops_t watchdog_Ops = {
	.init = Hardware_IWDGInit,
	.kick = Hardware_IWDGKick,
};

Watchdog_Device_t watchdog_dev = {
	.Ops = &watchdog_Ops,
};



void Hardware_IWDGInit(Watchdog_Device_t * watchdog)
{
	if (!RCC_LSI_IsReady())
	{
		RCC_LSI_Enable();
	}

	IWDG_AccessCommand(IWDG_ACCESS_VALUE);
	IWDG_SetReload(IWDG_RESET_VALUE);
	IWDG_SetPrescaler(IWDG_PR_DIVIDER_256_6);
	IWDG_Enable();
}


void Hardware_IWDGKick(Watchdog_Device_t * watchdog)
{
	IWDG_ReloadCounter();
}

void Watchdog_DriverInit(void)
{
	watchdog_dev.Ops->init(&watchdog_dev);
}

void Watchdog_Kick(void)
{
	watchdog_dev.Ops->kick(&watchdog_dev);
}

void Watchdog_DisableKick(int disable)
{

}

int Watchdog_IsKickDisabled(void)
{
	return 0;
}
