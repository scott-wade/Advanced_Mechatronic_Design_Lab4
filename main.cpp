#include "main.h"
#include "debug_mort.h"
#include "nucleo_led.h"

int main (void)
{
    /* Initializations */
    init_LED1(); //uncomment once you have filled in the function
    while(1)
    {
      
      toggle_LED1(); //uncomment once you have filled in the function
      debugprintHelloWorld();
      for(int i=0; i<100; i++){
          debugprint(i);
      }
    
    }
}

