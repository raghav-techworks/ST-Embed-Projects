/*
 * watchdog.h
 *
 *  Created on: Feb 28, 2025
 *      Author: Raghavender Dornala
 */

#ifndef INC_WATCHDOG_H_
#define INC_WATCHDOG_H_

#include "stm32NucleoF429xx.h"
#include "iwdg.h"


typedef struct Watchdog_Device Watchdog_Device_t;

typedef struct
{
	void (*init)(Watchdog_Device_t * watchdog);
	void (*kick)(Watchdog_Device_t * watchdog);
}Watchdog_Ops_t;


struct Watchdog_Device
{
	const Watchdog_Ops_t * Ops;
};


void Watchdog_DriverInit(void);
void Watchdog_DisableKick(int disable);
int Watchdog_IsKickDisabled(void);
void Watchdog_Kick(void);






#endif /* INC_WATCHDOG_H_ */
