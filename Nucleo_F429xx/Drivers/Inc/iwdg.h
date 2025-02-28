/*
 * iwdg.h
 *
 *  Created on: Feb 28, 2024
 *      Author: Raghavender Dornala
 */

#ifndef DRIVERS_INC_IWDG_H_
#define DRIVERS_INC_IWDG_H_

#include<stdint.h>
#include "stm32NucleoF429xx.h"

#define __vo volatile



#define IWDG_ENABLE_VALUE				0xCCCC	/* Writing to IWDG_KR, Starts the WD((except if the H/W WD option is selected)) */
#define IWDG_RESET_VALUE				0XFFF   /* Writing to IWDG_RLR, */
#define IWDG_RELOAD_VALUE				0xAAAA	/* Must be written by software at regular intervals to IWDG_KR, otherwise the WD generates a reset when the counter reaches 0 */
#define IWDG_ACCESS_VALUE				0x5555  /* Writing to IWDG_KR, Enable access to the IWDG_PR and IWDG_RLR registers */

/* Status Registers -> IWDG_SR */

#define IWDG_SR_PVU_BITPOS		0				/* Watchdog Prescaler value update, can be updated only when RVU bit is reset.*/
#define IWDG_SR_RVU_BITPOS		1				/* Watchdog counter reload value update, can be updated only when PVU bit is reset.*/

//#define IWDG_SR_RESET			0
//#define IWDG_SR_SET				1

typedef enum {
	SR_RESET = 0,
	SR_SET = !RESET
}FlagStatus;

/* Status Registers -> IWDG_PR */

#define IWDG_PR_DIVIDER_4			((uint8_t)0x00)
#define IWDG_PR_DIVIDER_8			((uint8_t)0x01)
#define IWDG_PR_DIVIDER_16			((uint8_t)0x02)
#define IWDG_PR_DIVIDER_32			((uint8_t)0x03)
#define IWDG_PR_DIVIDER_64			((uint8_t)0x04)
#define IWDG_PR_DIVIDER_128			((uint8_t)0x05)
#define IWDG_PR_DIVIDER_256_6		((uint8_t)0x06)
#define IWDG_PR_DIVIDER_256_7		((uint8_t)0x07)


void IWDG_AccessCommand(uint16_t IWDG_WriteAccess);
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler);
void IWDG_SetReload(uint16_t Reload);
void IWDG_ReloadCounter(void);

void IWDG_Enable(void);

FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_Flag);

#endif

