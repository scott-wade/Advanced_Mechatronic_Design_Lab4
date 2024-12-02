/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H_
#define __MAIN_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_mort2.h"
#include "queue.h"

/* MACROS for everyone--------------------------------------------------------*/
typedef enum{
    BUTTONPUSH,
    TIMEOUT
}Lab4_Event;

typedef enum{
    IDLE,
    LED_RED_ON,
    LED_GREEN_ON,
    LED_BLUE_ON
}Lab4_State;


/*Function definitions---------------------------------------------------------*/


/*Function definitions---------------------------------------------------------*/
void eventChecker(Queue* event_queue);
void eventHandler(Lab4_Event current_event);
void display_output(void);


#ifdef __cplusplus
}
#endif

#endif /*__MAIN_H */
