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



void init_tim3_output3_toggle(uint16_t interval, uint8_t OC_interrupt){
    //interval should be in units of 100us
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
    *reg_ptr = (uint16_t)interval; // every 10,000 cycles, aka 1Hz, trigger
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

    // Set interrupt enable CCxIE and/or CCxDE
    reg_ptr = TIM3_DIER_REGISTER;
    if (OC_interrupt > 0){
        *reg_ptr = *reg_ptr | (uint16_t)0b1000;
    }else{
         *reg_ptr = *reg_ptr & (uint16_t)0b0111;
    }
    
    // Enable timer by setting CEN bit in TIM3_CR1
    reg_ptr = TIM3_CR1_REGISTER;
    *reg_ptr = *reg_ptr | (uint16_t)0b1;

}

void init_tim3_incap(void){
    uint32_t* reg_ptr;
    //Write CC1S bits in TIMx_CCMR1 register to select the input (01 for TI1) 
    reg_ptr = (uint32_t*)TIM3_CCMR1_REGISTER;
    *reg_ptr = *reg_ptr & ~((uint32_t)0b11); //clear bits
    *reg_ptr = *reg_ptr | (uint32_t)0b01; //set bits

    //Write ICxF bits (7:4) in TIMx_CCMR1 register to select the lowpass filter duration, say 8 samples (0011)
    *reg_ptr = *reg_ptr & ~((uint32_t)0xf0); //clear bits
    *reg_ptr = *reg_ptr | (uint32_t)0x30; //0011
    //Write IC1PS bits (3:2) to 00 in the TIMx_CCMR1 register (disable prescale)
    // 0b00xx
    *reg_ptr = *reg_ptr & ~((uint32_t)0b1100); //clear bits

    //Write CC1P and CC1NP bits to 00 in the TIMx_CCER register to select rising edge
    // CC1P is bit 1, CC1NP is bit 3, the order of bits is CC1NP/CC1P 
    reg_ptr = (uint32_t*)TIM3_CCER_REGISTER;
    *reg_ptr = *reg_ptr & ~((uint32_t)0b1010); //clear bits

    //Set CC1E bit in TIMx_CCER register (enable capture)
    // bit 0
    *reg_ptr = *reg_ptr | (uint32_t)0b1; //set bit

    //Optionally, to enable interrupt or DMA, set CC1IE or CC1DE in TIMx_DIER 

    // Enable timer by setting CEN bit in TIM3_CR1
    reg_ptr = TIM3_CR1_REGISTER;
    *reg_ptr = *reg_ptr | (uint16_t)0b1;
}

void init_tim3_pwm(void){

}
