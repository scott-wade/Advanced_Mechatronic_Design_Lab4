/**
  ******************************************************************************
  * @file    hardware_stm_interrupt.c 
  * @author  stwade@andrew.cmu.edu
  * @version 1.0
  * @date    October-2024
  * @brief   Controls STM32F446ze Interrupts
  ******************************************************************************
  */

#include "hardware_stm_timer3.h"
#include "stm32f4xx_rcc_mort.h"
#include <cstdint>

#include "hardware_stm_gpio.h"

/* MACRO definitions----------------------------------------------------------*/
#define SYSTEM_CONTROL_BASE_ADDRESS (0xE000E000)
#define NVIC_BASE_ADDRESS (SYSTEM_CONTROL_BASE_ADDRESS + 0x100)
#define NVIC_INTERRUPT_SET_ENABLE_REGISTER_0_31 (NVIC_BASE_ADDRESS)
#define NVIC_INTERRUPT_SET_ENABLE_REGISTER_32_63 (NVIC_BASE_ADDRESS+0x4)
#define NVIC_INTERRUPT_SET_ENABLE_REGISTER_64_95 (NVIC_BASE_ADDRESS+0x8)
#define NVIC_INTERRUPT_CLEAR_ENABLE_REGISTER_0_31 (NVIC_BASE_ADDRESS + 0x80)
#define NVIC_INTERRUPT_CLEAR_ENABLE_REGISTER_32_63 (NVIC_INTERRUPT_CLEAR_ENABLE_REGISTER_0_31 + 0x4)
#define NVIC_INTERRUPT_CLEAR_ENABLE_REGISTER_64_95 (NVIC_INTERRUPT_CLEAR_ENABLE_REGISTER_0_31 + 0x8)
#define NVIC_INTERRUPT_SET_PENDING_REGISTER_0_31 (NVIC_BASE_ADDRESS + 0x100)
#define NVIC_INTERRUPT_SET_PENDING_REGISTER_32_63 (NVIC_INTERRUPT_SET_PENDING_REGISTER_0_31 + 0x4)
#define NVIC_INTERRUPT_SET_PENDING_REGISTER_64_95 (NVIC_INTERRUPT_SET_PENDING_REGISTER_0_31 + 0x8)
#define NVIC_INTERRUPT_CLEAR_PENDING_REGISTER_0_31 (NVIC_BASE_ADDRESS + 0x180)
#define NVIC_INTERRUPT_CLEAR_PENDING_REGISTER_32_63 (NVIC_INTERRUPT_CLEAR_PENDING_REGISTER_0_31 + 0x4)
#define NVIC_INTERRUPT_CLEAR_PENDING_REGISTER_64_95 (NVIC_INTERRUPT_CLEAR_PENDING_REGISTER_0_31 + 0x8)
#define TIM3_INTERRUPT_BIT (0x20000000)
#define EXTI9_5_INTERRUPT_BIT (0x800000)
//For external interrupts:
#define SYSCFG_BASE_ADDRESS ((uint32_t)(0x40013800))
#define SYSCFG_EXTERNAL_INTERRUPT_REGISTER_2 (SYSCFG_BASE_ADDRESS + 0x0C)
#define SYSCFG_EXTERNAL_INTERRUPT_6_BITS ((uint32_t)0xF00) //flags for External interrupt register 2
#define SYSCFG_EXTERNAL_INTERRUPT_6_PORTC ((uint32_t)0x200)
//External interrupt controller :
#define EXTERNAL_INTERRUPT_CONTROLLER_BASE_ADDRESS ((uint32_t)(0x40013C00))
#define EXTERNAL_INTERRUPT_CONTROLLER_MASK_REGISTER (EXTERNAL_INTERRUPT_CONTROLLER_BASE_ADDRESS)
#define EXTERNAL_INTERRUPT_CONTROLLER_MASK_REGISTER_EXTI6 ((uint32_t)0x40) //flags for external interrupt controller mask register
#define EXTERNAL_INTERRUPT_CONTROLLER_RTSR (EXTERNAL_INTERRUPT_CONTROLLER_BASE_ADDRESS+0x08)
#define EXTERNAL_INTERRUPT_CONTROLLER_RTSR_EXTI6 ((uint32_t)0x40)
#define EXTERNAL_INTERRUPT_CONTROLLER_FTSR (EXTERNAL_INTERRUPT_CONTROLLER_BASE_ADDRESS+0x0C)
#define EXTERNAL_INTERRUPT_CONTROLLER_FTSR_EXTI6 ((uint32_t)0x40)
#define EXTERNAL_INTERRUPT_CONTROLLER_PENDING_REGISTER (EXTERNAL_INTERRUPT_CONTROLLER_BASE_ADDRESS+0x14)
#define EXTERNAL_INTERRUPT_CONTROLLER_PENDING_EXTI6 ((uint32_t)0x40)


void enableNVIC_Timer3(void){
    uint32_t* reg_pointer_32;
    reg_pointer_32 = (uint32_t *)NVIC_INTERRUPT_SET_ENABLE_REGISTER_0_31;
    *reg_pointer_32 = TIM3_INTERRUPT_BIT;
}

void enableEXTI6OnPortC(void){
    uint32_t * reg_pointer_32;
    /*Init GPIO 6 C as input*/
    initGpioC6AsInput();
    /*As a test, Init GPIO B0 as output for debugging*/
    initGpioB0AsOutput();
    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    /*map EXTI6 to port C bit 6*/
    reg_pointer_32 = (uint32_t *)SYSCFG_EXTERNAL_INTERRUPT_REGISTER_2;
    //clear EXTI6
    *reg_pointer_32 = *reg_pointer_32 & ~SYSCFG_EXTERNAL_INTERRUPT_6_BITS;
    //set EXTI6 to Port C
    *reg_pointer_32 = *reg_pointer_32 | SYSCFG_EXTERNAL_INTERRUPT_6_PORTC;
    /*un-mask EXTI6*/
    reg_pointer_32 = (uint32_t *)EXTERNAL_INTERRUPT_CONTROLLER_MASK_REGISTER;
    *reg_pointer_32 = *reg_pointer_32 | EXTERNAL_INTERRUPT_CONTROLLER_MASK_REGISTER_EXTI6;
    /*trigger on rising edge*/
    reg_pointer_32 = (uint32_t *)EXTERNAL_INTERRUPT_CONTROLLER_RTSR;
    *reg_pointer_32 = *reg_pointer_32 | EXTERNAL_INTERRUPT_CONTROLLER_RTSR_EXTI6;
    /* set the NVIC to respond to EXTI9_5*/
    reg_pointer_32 = (uint32_t *)NVIC_INTERRUPT_SET_ENABLE_REGISTER_0_31;
    *reg_pointer_32 = EXTI9_5_INTERRUPT_BIT;
}



void EXTI9_5_IRQHandler(void){
    /* Handle external interrupt events on lines 5-9 */

    uint32_t * reg_pointer_32;
    
    //check which interrupt fired:
    reg_pointer_32 = (uint32_t *)EXTERNAL_INTERRUPT_CONTROLLER_PENDING_REGISTER;
    if ((*reg_pointer_32 & EXTERNAL_INTERRUPT_CONTROLLER_PENDING_EXTI6)>0) // EXTI6
    {
        //clear the interrupt:
        *reg_pointer_32 = EXTERNAL_INTERRUPT_CONTROLLER_PENDING_EXTI6;
        //toggle the LED:
        toggleGPIOB0();
    }

}
