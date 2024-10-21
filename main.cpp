#include "main.h"
#include "debug_mort.h"
#include "nucleo_led.h"

int main (void)
{
    /* Initializations */
    init_LED1(); //uncomment once you have filled in the function
    initC6();
    while(1)
    {
      
      check_and_set_LED(); //uncomment once you have filled in the function
      debugprintHelloWorld();
      for(int i=0; i<5; i++){
          debugprint(i);
      }
    
    }
}

