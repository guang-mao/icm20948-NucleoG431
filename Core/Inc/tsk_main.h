/*
 * tsk_main.h
 *
 *  Created on: 2024年8月15日
 *      Author: 20210113
 */

#ifndef INC_TSK_MAIN_H_
#define INC_TSK_MAIN_H_
#include "main.h"

typedef enum
{
  INIT = 0,
  CALIBRA,
  RUN
} BOOTSTAGE_t;

void Setup();
void Loop();

extern BOOTSTAGE_t bootstage;
extern uint32_t bootTick;

#endif /* INC_TSK_MAIN_H_ */
