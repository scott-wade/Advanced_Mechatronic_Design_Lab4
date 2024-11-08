#include "main.h"
#include "debug_mort.h"
#include "nucleo_led.h"
#include "hardware_stm_gpio.h"
#include "hardware_stm_timer3.h"

int main (void)
{
    /* Initializations */
    init_tim3_incap(50000);
    initGpioC6AsAF2();
    debugprintHelloWorld();


    while(1){ // loop
        uint16_t tim3_sr_mask = (uint16_t)0b10; // check TIM3_SR's CC1IF bit
        if ((*TIM3_SR_REGISTER & tim3_sr_mask) > 0){ 
           printf("timer read %u\n", (uint16_t) *TIM3_CCR1_REGISTER); // print value in CCR3
        }
    }

}

