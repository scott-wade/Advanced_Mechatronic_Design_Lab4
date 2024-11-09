#include "main.h"
#include "debug_mort.h"
#include "nucleo_led.h"
#include "hardware_stm_gpio.h"
#include "hardware_stm_timer3.h"
#include "hardware_stm_interrupt.h"

int main (void)
{
    /* Initializations */
    enableEXTI6OnPortC();
    debugprintHelloWorld();

}

