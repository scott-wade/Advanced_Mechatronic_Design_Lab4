#include "main.h"
#include "debug_mort.h"
#include "nucleo_led.h"

int main (void)
{
    /* Initializations */
    init_tim3_output3_toggle_LED();
    debugprintHelloWorld();

}

