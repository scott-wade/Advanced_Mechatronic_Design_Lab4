/**
  ******************************************************************************
  * @file    hardware_stm_gpio.c 
  * @author  mortamar@andrew.cmu.edu
  * @version 1.0
  * @date    Septembr-2021
  * @brief   Controls STM32F446ze GPIO
  ******************************************************************************
  */

#include "hardware_stm_gpio.h"
#include "stm32f4xx_rcc_mort.h"
#include <cstdint>


//led 1 is connected to PB0. 
// GPIO B addresses: 0x4002 0400 - 0x4002 07FF
// GPIO C addresses: 0x4002 0800 - 0x4002 0BFF


/* MACRO definitions----------------------------------------------------------*/
//Port B addresses:
#define PORTB_BASE_ADDRESS ((uint32_t)0x40020400)        //The first address in memory corresponding to Port B (this is in the user manual!)
// I gave you the first one, now you fill in the rest, check in the user manual what is the offset from the base address for each register!
// GPIO B has addresses 0x4002 0400 - 0x4002 07FF
#define PORTB_MODER_REGISTER (PORTB_BASE_ADDRESS + 0x00) //replace the question mark with the correct offset!
#define PORTB_OTYPER_REGISTER (PORTB_BASE_ADDRESS + 0x04)
#define PORTB_OSPEEDR_REGISTER (PORTB_BASE_ADDRESS + 0x08)
#define PORTB_PUPDR_REGISTER (PORTB_BASE_ADDRESS + 0x0C)
#define PORTB_IDR_REGISTER (PORTB_BASE_ADDRESS + 0x10)
#define PORTB_ODR_REGISTER (PORTB_BASE_ADDRESS + 0x14)
#define PORTB_BSRRL_REGISTER (PORTB_BASE_ADDRESS + 0x18)
#define PORTB_BSRR_REGISTER (PORTB_BASE_ADDRESS + 0x18)
#define PORTB_BSRRH_REGISTER (PORTB_BASE_ADDRESS + 0x1A)
#define PORTB_LCKR_REGISTER (PORTB_BASE_ADDRESS + 0x1C)
#define PORTB_AFR1_REGISTER (PORTB_BASE_ADDRESS + 0x20)
#define PORTB_AFR2_REGISTER (PORTB_BASE_ADDRESS + 0x24)
#define PORTB_OSPEEDR_REGISTER (PORTB_BASE_ADDRESS + 0x08)

//Port C addresses:
#define PORTC_BASE_ADDRESS ((uint32_t)0x40020800)  //The first address in memory corresponding to Port B (this is in the user manual!)
// I gave you the first one, now you fill in the rest, check in the user manual what is the offset from the base address for each register!
// GPIO B has addresses 0x4002 0400 - 0x4002 07FF
#define PORTC_MODER_REGISTER (PORTC_BASE_ADDRESS + 0x00) //replace the question mark with the correct offset!
#define PORTC_OTYPER_REGISTER (PORTC_BASE_ADDRESS + 0x04)
#define PORTC_OSPEEDR_REGISTER (PORTC_BASE_ADDRESS + 0x08)
#define PORTC_PUPDR_REGISTER (PORTC_BASE_ADDRESS + 0x0C)
#define PORTC_IDR_REGISTER (PORTC_BASE_ADDRESS + 0x10)
#define PORTC_ODR_REGISTER (PORTC_BASE_ADDRESS + 0x14)
#define PORTC_BSRRL_REGISTER (PORTC_BASE_ADDRESS + 0x18)
#define PORTC_BSRR_REGISTER (PORTC_BASE_ADDRESS + 0x18)
#define PORTC_BSRRH_REGISTER (PORTC_BASE_ADDRESS + 0x1A)
#define PORTC_LCKR_REGISTER (PORTC_BASE_ADDRESS + 0x1C)
#define PORTC_AFR1_REGISTER (PORTC_BASE_ADDRESS + 0x20)
#define PORTC_AFR2_REGISTER (PORTC_BASE_ADDRESS + 0x24)
#define PORTC_OSPEEDR_REGISTER (PORTC_BASE_ADDRESS + 0x08)


//flags MODER Register:

//flags OTYPER Register:

//flags OSPEEDR Register:

//flags PUPDR Register:

//input data register:

//flags AFR1 Register:

//flags ODR Register:


/* function definitions----------------------------------------------------------*/

void initGpioC6AsInput( void )
{
    uint32_t  * reg_pointer; 
    /* GPIOC Peripheral clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    /* GPIOC Pin 6 as input*/
    uint32_t PINC6_RESET = ~((uint32_t) 0x3000);
    reg_pointer = (uint32_t *)PORTC_MODER_REGISTER;
    *reg_pointer = *reg_pointer & PINC6_RESET; // reset pinC moder pin
    *reg_pointer = *reg_pointer & (uint32_t) 0x3000; // set pinC moder to output

    //No need to configure push-pull or speed for input pins
    
    /*Configure pulled-down*/
    uint32_t PINC6_PUPDR_PD = (uint32_t) 0x2000;
    reg_pointer = (uint32_t *)PORTC_PUPDR_REGISTER;
    *reg_pointer = *reg_pointer & PINC6_RESET; // reset pinc pupdr
    *reg_pointer = *reg_pointer | PINC6_PUPDR_PD; // set pinc pupdr to PD

}

void initGpioC6AsAF2 (void)
{
    uint32_t* reg_pointer; 
    /* GPIOC Peripheral clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    /* GPIOC Pin 6 as input*/
    uint32_t PINC6_RESET = ~((uint32_t) 0x3000);
    reg_pointer = (uint32_t *)PORTC_MODER_REGISTER;
    *reg_pointer = *reg_pointer & PINC6_RESET; // reset pinC moder pin
    *reg_pointer = *reg_pointer | (uint32_t) 0x2000; // set pinC moder to AF

    //No need to configure push-pull or speed for input pins
    
    /*Configure pulled-down*/
    uint32_t PINC6_PUPDR_PD = (uint32_t) 0x2000;
    reg_pointer = (uint32_t *)PORTC_PUPDR_REGISTER;
    *reg_pointer = *reg_pointer & PINC6_RESET; // reset pinc pupdr
    *reg_pointer = *reg_pointer | PINC6_PUPDR_PD; // set pinc pupdr to PD

    // Configure alternate function 2
    /*Select AF2*/
    uint32_t PINC6_AF2_SET = (uint32_t)0x2000000;
    reg_pointer = (uint32_t *)PORTC_AFR1_REGISTER;
    *reg_pointer = *reg_pointer & ~((uint32_t)0xF000000);
    *reg_pointer = *reg_pointer | PINC6_AF2_SET;

}


void initGpioB0AsOutput( void )
{
    uint32_t  * reg_pointer;
    /* GPIOB Peripheral clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    /* GPIOB0 configured as output */
    uint32_t PINB0_MODER_RESET = ~(uint32_t)0b11;
    reg_pointer = (uint32_t *)PORTB_MODER_REGISTER;
    *reg_pointer = *reg_pointer & PINB0_MODER_RESET; // reset pinB moder pin
    *reg_pointer = *reg_pointer | (uint32_t) 0b01; // set pinB moder to output
    /*GPIOB0 configured as push-pull */
    reg_pointer = (uint32_t *)PORTB_OTYPER_REGISTER;
    uint32_t PINB0_OTYPER_PUSHPULL = ~((uint32_t) 0x1);
    *reg_pointer = *reg_pointer & PINB0_OTYPER_PUSHPULL;
    /*GPIOB0 configured floating */
    reg_pointer = (uint32_t *)PORTB_PUPDR_REGISTER;
    uint32_t PINB0_PUPDR_FLOATING = ~((uint32_t) 0b11);
    *reg_pointer = *reg_pointer & PINB0_PUPDR_FLOATING;
    /* GPIOB0 driven high to start out with */
    reg_pointer = (uint32_t *)PORTB_ODR_REGISTER;
    *reg_pointer = *reg_pointer | 0b01;
    
}

void initGpioB0AsAF2( void )
{
    uint32_t  * reg_pointer;
    /* GPIOB Peripheral clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    /* GPIOB0 configured as AF */
    uint32_t PINB0_MODER_RESET = ~(uint32_t)0b11;
    reg_pointer = (uint32_t *)PORTB_MODER_REGISTER;
    *reg_pointer = *reg_pointer & PINB0_MODER_RESET; // reset pinB0 moder pin
    *reg_pointer = *reg_pointer | (uint32_t) 0b10; // set pinB0 moder to AF
    /*Select AF2*/
    uint32_t PINB0_AF2_SET = (uint32_t)0b0010;
    reg_pointer = (uint32_t *)PORTB_AFR1_REGISTER;
    *reg_pointer = *reg_pointer & ~((uint32_t)0b1111);
    *reg_pointer = *reg_pointer | PINB0_AF2_SET;
    /*Set high speed pin*/
    reg_pointer = (uint32_t *)PORTB_OSPEEDR_REGISTER;
    *reg_pointer = *reg_pointer | ((uint32_t)0b0011);
    /*GPIOB0 configured as push-pull */
    reg_pointer = (uint32_t *)PORTB_OTYPER_REGISTER;
    uint32_t PINB0_OTYPER_PUSHPULL = ~((uint32_t) 0x1);
    *reg_pointer = *reg_pointer & PINB0_OTYPER_PUSHPULL;
    /*GPIOB0 configured floating */
    reg_pointer = (uint32_t *)PORTB_PUPDR_REGISTER;
    uint32_t PINB0_PUPDR_FLOATING = ~((uint32_t) 0b11);
    *reg_pointer = *reg_pointer & PINB0_PUPDR_FLOATING;
    /* GPIOB0 driven high to start out with */
    reg_pointer = (uint32_t *)PORTB_ODR_REGISTER;
    *reg_pointer = *reg_pointer | 0b01;
    
}



void toggleGPIOB0( void )
{
    uint32_t value;
    uint32_t  * reg_pointer;

    //get the current value of the pin 
    reg_pointer = (uint32_t *)PORTB_ODR_REGISTER;
    value = *reg_pointer & 0b01;
   
    if (value > 0) // if high, clear pin
    {
        reg_pointer = (uint32_t *)PORTB_ODR_REGISTER;
        *reg_pointer = *reg_pointer & 0b00;
    }
    else // if low, set pin
    {
        reg_pointer = (uint32_t *)PORTB_ODR_REGISTER;
        *reg_pointer = *reg_pointer | 0b01;
    } 
}



void setGPIOB0( void )
{
    uint32_t  * reg_pointer;
    reg_pointer = (uint32_t *)PORTB_ODR_REGISTER;
    *reg_pointer = *reg_pointer | 0b01;      
}
void clearGPIOB0( void )
{
    uint32_t  * reg_pointer;
    reg_pointer = (uint32_t *)PORTB_ODR_REGISTER;
    *reg_pointer = *reg_pointer & 0b00;
}

uint32_t checkGPIOC6(void)
{
    uint32_t PINC6_MASK = (uint32_t)0x40; // mask for only pin 6
    uint32_t valueC6;
    uint32_t* reg_pointer = (uint32_t*)PORTC_IDR_REGISTER;
    valueC6 = *reg_pointer & PINC6_MASK;
    return valueC6;
}