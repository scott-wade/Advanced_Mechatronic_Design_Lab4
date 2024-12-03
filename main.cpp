#include "main.h"
#include "debug_mort.h"
#include "nucleo_led.h"
#include "hardware/hardware_stm_gpio.h"
#include "hardware/hardware_stm_timer3.h"
#include "hardware/hardware_stm_interrupt.h"
#include "queue.h"


/* Initialize state machine variables */ 
uint8_t RED_VAL, GREEN_VAL, BLUE_VAL;
Lab4_State CURRENT_STATE = IDLE;
uint8_t BUTTONPUSHED = 0;


int main (void)
{
    /* Initializations */
    // create event queue
    Queue* event_queue = createQueue(sizeof(Lab4_Event));
    Lab4_Event current_event;

    // initialize GPIO pins
        // PB_0 as output
        initGpioBiAsOutput(PIN_0);
        // PB_1 as output
        initGpioBiAsOutput(PIN_1);
        // PB_2 as output
        initGpioBiAsOutput(PIN_2);
        // PA_6 as input
        initGpioA6AsInput();
        // PC_6 as input
        initGpioC6AsInput();

    // initialize timer
        // Configure Tim3 with an interval of 20ms
        init_tim3_output3_toggle(200, 0);
        // Pause and reset Tim3
        disable_tim3();
        clear_tim3_ch3_flag();

    //initialize state vars
    RED_VAL = 0;
    GREEN_VAL = 0;
    BLUE_VAL = 0;

    // print hello world
    debugprintHelloWorld();

    /* Loop */ 
    while(1){
        // check for events
        eventChecker(event_queue);

        // handle the first event in the queue
        while(!isEmpty(event_queue)){
            current_event = *(Lab4_Event*)dequeue(event_queue);
            eventHandler(current_event);
        }

    }

}


void eventChecker(Queue* event_queue){
    /* Checks for events and adds them to the event queue */ 
    
    // check for a button press event
    if ((checkGPIOA6() > 0) && (BUTTONPUSHED == 0) ){
        Lab4_Event new_event = BUTTONPUSH;
        enqueue(event_queue, &new_event);
        BUTTONPUSHED = 1;
    }
    // clear the BUTTONPUSHED flag on button not pushed
    if ((checkGPIOA6() == 0) && (BUTTONPUSHED == 1) ){
        BUTTONPUSHED = 0;
    }

    // check for a timeout event
    if (read_tim3_ch3_flag() > 0 ){
        Lab4_Event new_event = TIMEOUT;
        enqueue(event_queue, &new_event);
    }

}


void eventHandler(Lab4_Event current_event){
    /* Handles events */ 

    switch(current_event){
        case BUTTONPUSH:
            if (CURRENT_STATE == IDLE){
                // Transition to Red LED On.
                CURRENT_STATE = LED_RED_ON;
                printf("Transitioing to LED RED\n");
                // Reset vars for LED values.
                RED_VAL = 0;
                GREEN_VAL = 0;
                BLUE_VAL = 0;
                // Turn on red LED.
                setGPIO(PORT_B, PIN_0);
                // Turn on Tim3.
                enable_tim3();

            }
            break;
        case TIMEOUT:
            switch(CURRENT_STATE){
                case IDLE: 
                    // disable tim3
                    disable_tim3();
                    // clear tim3 overflow flag
                    clear_tim3_ch3_flag();
                    printf("Diabling Timer\n");
                    break; 
                case LED_RED_ON:
                    // red_val = readGPIOC6
                    RED_VAL = checkGPIOC6();
                    // Turn off red LED, turn on green LED.
                    clearGPIO(PORT_B, PIN_0);
                    setGPIO(PORT_B, PIN_1);
                    // Transition to Green LED On.
                    CURRENT_STATE = LED_GREEN_ON;
                    printf("Transitioing to LED GREEN\n");
                    // Clear Tim3 overflow flag.
                    clear_tim3_ch3_flag();
                    break;
                case LED_GREEN_ON:
                    // Green_val = readGPIO(C6)
                    GREEN_VAL = checkGPIOC6();
                    // Turn off green LED, turn on blue LED.
                    clearGPIO(PORT_B, PIN_1);
                    setGPIO(PORT_B, PIN_2);
                    // Transition to Blue LED On.
                    CURRENT_STATE = LED_BLUE_ON;
                    printf("Transitioing to LED BLUE\n");
                    // Clear Tim3 overflow flag.
                    clear_tim3_ch3_flag();
                    break;
                case LED_BLUE_ON:
                    //Blue_val = readGPIO(C6)
                    BLUE_VAL = checkGPIOC6();
                    //Turn off blue LED.
                    clearGPIO(PORT_B, PIN_2);
                    //Display output according to truth table.
                    display_output();
                    //Disable Tim3.
                    disable_tim3();
                    //Clear Tim3 overflow flag. 
                    clear_tim3_ch3_flag();
                    //Transition to IDLE.
                    CURRENT_STATE = IDLE;
                    printf("Transitioing to IDLE\n");
                    break;
            }
            break;
    }

}

void display_output(void){
    printf("Finished measurement! \n");
    printf("Have RED %u, GREEN %u, BLUE %u\n", RED_VAL, GREEN_VAL, BLUE_VAL);
    if ((RED_VAL == 0) && (GREEN_VAL > 0) && (BLUE_VAL > 0)){
        printf("Gummy bear is RED \n");
    }
    if ((RED_VAL > 0) && (GREEN_VAL == 0) && (BLUE_VAL > 0)){
        printf("Gummy bear is GREEN \n");
    }
    if ((RED_VAL == 0) && (GREEN_VAL == 0) && (BLUE_VAL > 0)){
        printf("Gummy bear is ORANGE \n");
    }
    if ((RED_VAL == 0) && (GREEN_VAL == 0) && (BLUE_VAL == 0)){
        printf("Gummy bear is WHITE \n");
    }
}
