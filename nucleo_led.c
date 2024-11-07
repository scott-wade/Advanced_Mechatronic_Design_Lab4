/**
  ******************************************************************************
  * @file    nucleo_led.c 
  * @author  mortamar@andrew.cmu.edu
  * @version 1.0
  * @date    Septembr-2021
  * @brief   Controls the LED's on the nucleo board
  ******************************************************************************
  */

#include "hardware_stm_gpio.h"
#include "hardware_stm_timer3.h"
#include "nucleo_led.h"

/************************************
* Initializes LED1 on the nucleo Board which is connected to Port B Pin 0
*************************************/
void init_LED1(void )
{
    initGpioB0AsOutput();
}
/************************************
* Toggles LED1 
*************************************/
void toggle_LED1( void )
{
    toggleGPIOB0();
}

void check_and_set_LED()
{
    static uint32_t previous_state;
    uint32_t value = checkGPIOC6();
    if (value > 0){
        if (previous_state != 1){
            setGPIOB0();
            previous_state = 1;
        }
    }
    else {
        if (previous_state == 1){
            clearGPIOB0();
            previous_state = 0;
        }
        
    }
}

void initC6(void){
    initGpioC6AsInput();
}

void init_tim3_output3_toggle_LED(void){
    //Enable GPIO B0 for its Alternate Function 2
    initGpioB0AsAF2();
    // configure timer3 channel3 as a toggle output comparison
    init_tim3_output3_toggle((uint16_t)10000, (uint8_t)1);
}