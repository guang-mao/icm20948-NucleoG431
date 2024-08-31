/*
 * dwt_delay.h
 *
 *  Created on: Aug 27, 2024
 *      Author: m2640
 */

#ifndef INC_DWT_DELAY_H_
#define INC_DWT_DELAY_H_


#ifdef __cplusplus
extern "C" {
#endif

#if defined(STM32F103xB)
#include "stm32f1xx_hal.h"
#elif defined(STM32F412Cx) || defined(STM32F446xx) || defined(STM32F401xC)
#include "stm32f4xx_hal.h"
#elif defined(STM32G431xx)
#include "stm32g4xx_hal.h"
#endif

__STATIC_INLINE uint32_t DWT_Init(void)
{
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

  /* 3 NO OPERATIION instructions */
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");

  /* Check if clock cycle counter has started */
  if ( DWT->CYCCNT )
  {
    return 0; /* clock cycle counter started */
  }
  else
  {
    return 1;
  }
}

/**
 * @brief This function provides a delay (in microseconds)
 * @param microseconds: delay in microseconds
 */
__STATIC_INLINE void delay_us(uint32_t us)
{
  uint32_t us_count_tic = us * ( SystemCoreClock / 1000000U );

  DWT->CYCCNT = 0U;

  while ( DWT->CYCCNT < us_count_tic );
}

__STATIC_INLINE uint32_t micros(void)
{
  return DWT->CYCCNT / ( SystemCoreClock / 1000000U );
}

#define delay_ms    HAL_Delay

#ifdef __cplusplus
}
#endif



#endif /* INC_DWT_DELAY_H_ */
