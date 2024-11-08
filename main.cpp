#include "main.h"
#include "debug_mort.h"
#include "nucleo_led.h"
#include "hardware_stm_gpio.h"
#include "hardware_stm_timer3.h"

int main (void)
{
    /* Initializations */
    init_tim3_pwm(40000, 20000);
    initGpioB0AsAF2();
    debugprintHelloWorld();


}

