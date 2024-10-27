/**
  ******************************************************************************
  * @file    hardware_stm_timer3.c 
  * @author  stwade@andrew.cmu.edu
  * @version 1.0
  * @date    October-2024
  * @brief   Controls STM32F446ze Timer
  ******************************************************************************
  */

#include "hardware_stm_timer3.h"
#include "stm32f4xx_rcc_mort.h"
#include <cstdint>


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
#define TIM3_CCR3_REGISTER (uint32_t*)(TIM3_BASE_ADDRESS + 0x3c)



void init_tim3_output3_toggle(void){
    uint32_t* reg_ptr;
    // enable APB1 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    // clear update event flags in TIM3_SR (UIF?)
    reg_ptr = (uint32_t*)TIM3_SR_REGISTER;
    *reg_ptr = *reg_ptr & ~((uint32_t)(0b1));

    // Upload prescale value to TIM3_PSC
    reg_ptr = TIM3_PSC_REGISTER;
    *reg_ptr = (uint16_t)8999; // 10kHz timer

    // Set wanted period value to TIM3_AAR (should it be CCR3?)
    reg_ptr = TIM3_ARR_REGISTER;
    *reg_ptr = (uint16_t)10000; // every 10,000 cycles, aka 1Hz, trigger
    // Set mode to toggle in CCMR2
    /*
    The output pin can keep its level (OCxM=000), be set
    active (OCxM=001), be set inactive (OCxM=010) or can toggle (OCxM=011) on match.
    */
    reg_ptr = TIM3_CCMR2_REGISTER;
    *reg_ptr = *reg_ptr & ~((uint16_t)0b111000); // clear prior bits
    *reg_ptr = *reg_ptr | (uint16_t)0b011000; // set toggle to OC3M

    // Disable preload register (idk what this does rn)


    // Output compare 1 fast enable ?
    

    // Configure channel as output (CCER register,  CC3E set)
    reg_ptr = TIM3_CCER_REGISTER;
    *reg_ptr = *reg_ptr & ~((uint16_t)0x100);// is this line necessary?
    *reg_ptr = *reg_ptr | (uint16_t)0x100;

    // Enable timer by setting CEN bit in TIM3_CR1
    reg_ptr = TIM3_CR1_REGISTER;
    *reg_ptr = *reg_ptr | (uint16_t)0b1;

}
