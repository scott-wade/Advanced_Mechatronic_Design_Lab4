/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HARDWARE_STM_GPIO_H_
#define __HARDWARE_STM_GPIO_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../main.h"


/* Macros for Everyone--------------------------------------------------------*/
#define PIN_0   0
#define PIN_1   1
#define PIN_2   2
#define PIN_3   3
#define PIN_4   4
#define PIN_5   5
#define PIN_6   6
#define PIN_7   7
#define PIN_8   8
#define PIN_9   9
#define PIN_10  10

#define PORT_A 0
#define PORT_B 1
#define PORT_C 2


/*Function definitions---------------------------------------------------------*/
void initGpioB0AsOutput( void );
void initGpioC6AsInput( void );
void initGpioA6AsInput( void );
void initGpioB0AsAF2( void );
void initGpioC6AsAF2( void );

// generalized output init
void initGpioBiAsOutput(uint8_t pin);

// GPIOB0 set/clear/toggle
void toggleGPIOB0( void );
void setGPIOB0( void );
void clearGPIOB0( void );

// generalized GPIO set/clear
void setGPIO(uint8_t port, uint8_t pin);
void clearGPIO(uint8_t port, uint8_t pin);

// check PC_6 and PA_6
uint32_t checkGPIOC6(void);
uint32_t checkGPIOA6(void);

#ifdef __cplusplus
}
#endif

#endif /*__GPIO_H */
