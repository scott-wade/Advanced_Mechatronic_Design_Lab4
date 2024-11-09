/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HARDWARE_STM_TIM3_H_
#define __HARDWARE_STM_TIM3_H_

#include <cstdint>
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Macros for Everyone--------------------------------------------------------*/
#define PIN_0   0
#define PIN_1   1
#define PIN_2   2
#define PIN_3   3
#define PIN_4   4
#define PIN_5   5
#define PIN_6   6
#define PIN_7   7
#define PIN_8   8
#define PIN_9   9
#define PIN_10  10

#define TIM3_BASE_ADDRESS ((uint32_t)0x40000400)
#define TIM3_CR1_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x00)
#define TIM3_CR2_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x04)
#define TIM3_SMCR_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x08)
#define TIM3_DIER_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x0c)
#define TIM3_SR_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x10)
#define TIM3_EGR_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x14)
#define TIM3_CCMR1_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x18)
#define TIM3_CCMR2_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x1c)
#define TIM3_CCER_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x20)
#define TIM3_CNT_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x24)
#define TIM3_PSC_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x28)
#define TIM3_ARR_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x2c)
#define TIM3_CCR1_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x34)
#define TIM3_CCR3_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x3c)


/*Function definitions---------------------------------------------------------*/
void init_tim3_output3_toggle(uint16_t interval, uint8_t OC_interrupt);
void init_tim3_incap(uint16_t interval);
void init_tim3_pwm(uint16_t interval, uint16_t ontime);
void init_tim3_interrupt(uint16_t autoReload, uint16_t compare);

#ifdef __cplusplus
}
#endif

#endif /*__HARDWARE_STM_TIM3_H_ */
