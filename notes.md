# Notes for Lab 4

## Next steps
 - 


### Timer as an input capture

CCxIF can be cleared by software by writing it to 0 or by reading the captured data stored in the TIMx_CCRx register. 

x Timer setup:
    Write CC1S bits in TIMx_CCMR1 register to select the input (01 for TI1) 
    Write ICxF bits in TIMx_CCMR1 register to select the lowpass filter duration, say 8 samples (11)
    Write CC1P and CC1NP bits to 00 in the TIMx_CCER register to select rising edge
    Write IC1PS bits to 00 in the TIMx_CCMR1 register (disable prescale)
    Set CC1E bit in TIMx_CCER register (enable capture)
    Optionally, to enable interrupt or DMA, set CC1IE or CC1DE in TIMx_DIER 
x Configuring a GPIO pin to link it to a timer pin:
    Configure the pin as an Alternate Function pin, choose pin according to the AF table
        AF2 for pin PC6 binds it to TIM3_CH1
    Configure the pin as an input pin similarly to a normal input pin
x Reading the captured value:
    x Using software, check for a value in CCxIF
    x If the CCxIF pin is 1, read the value in TIMx_CCRx
        Should automatically clear the interrupt flag (?) check this when implementing

### Part 5. LED using PWM mode
    ● Setup PortB pin 0 as an output so you can access LED1.
    ● Setup Timer 3 channel 3 in pwm mode and map the output to PortB pin 0.
    ● Toggle the LED with a frequency of ~0.25Hz

    Setting up Tim3 Ch3 in PWM mode:
        x1. Enable the APB1 clock:
        x• RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
        x2. Clear the update event flag in the status register TIM3_SR
        x3. Upload the pre-scale value to TIM3_PSC
        x4. Set the wanted period value to the autoreload register TIM3_ARR
        x5. Select PWM Mode 1 for Timer 3 channel 3 by writing 110 to the OC3M bit fields in the CCMR2 register
        x 6. Set channel 3 of Timer 3 to an output by clearing the CC3S bits in the CCMR2 register
        x 7. Set the value to compare to in CCR3, which will set the duty cycle. For example if CCR3 = autoreload/2, then the duty cycle will be
        50%.
        x 8. Enable the Preload register for Timer 3 channel 3 by setting the OC3PE bit in the CCMR2 register
        x 9. Enable the TIM3 channel 3 by setting the CC3E bit in the CCER register
        x10. Enable the timer subsystem by setting the CEN bit in TIM3_CR1


        ### Part 6. Timer as output compare interrupt

        Can reuse a lot of the stuff from the toggle function
        Need to Set the CCxIE and/or CCxDE bits if an interrupt and/or a DMA request is to be
        generated.
        Don't use the timer to directly set the GPIO pin values. (Don't tie a GPIO pin to it)
        Instead, use it to trigger interrupt events in the software.

        ● Setup PortB pin 0 as an output so you can access LED1.
        ● Setup Timer 3 channel 3 in output compare mode and enable the interrupts both
        on output compare as well as Overflow.
        ● Toggle the LED with a frequency of ~0.25Hz using the timer’s interrupt service
        routines from output compare and overflow.

### Part 6. TIM3 CH3 Output Compare Interrupt
    ● Setup PortB pin 0 as an output so you can access LED1.
    ● Setup Timer 3 channel 3 in output compare mode and enable the interrupts both
     on output compare as well as Overflow.
    ● Toggle the LED with a frequency of ~0.25Hz using the timer’s interrupt service
    routines from output compare and overflow
    Enable the correct interrupt for tim3 Ch3 in the NVIC
    Configure Tim3 CH3 as an output compare with the interrupts enabled 
    In the interrupt service: 
        Make the normal interrupt turn the LED on
        Make the overflow interrupt turn the LED off and reset both interrupts 

### Part 7. External interrupt 6 using portC pin 6
    ● Setup PortB pin 0 as an output so you can access LED1.
    ● Setup PortC pin 6 as an input.
    ● Enable External interrupt 6 to PortC pin 6.
    ● Toggle LED1 every time you see a rising edge on PortC pin 6. (Toggle in the
    interrupt service routine

    - make a new function to configure portc pin  as an interrupt
        - first configure the pin as an input 
        - then enable the EXTI tied to that pin
            enable EXTI6 (bit 6 in EXTI_IMR)
        - and select PC6 as the pin tied to external interript line 6
            EXTI6[3:0] bits in SYSCFG_EXTICR2 register
                What is the register's addresss?
            0010 are correct values for pin C
            bits are [27:24]
        - select rising/falling edge config
            - set bit  in EXTI_RTSR to 1 to enable rising edge
            - set bit in EXTI_FTSR to enable falling edge
        - then enable EXTI6 in NVIC
            - set bit 23 in vector table

## Part 2: State Machine Pseudocode

    ### Pseudocode
        ```
        void init(void){
            // initialize 3 output pins for LEDs
            // initialize 1 input pin for phototransistor
            // initialize 1 input pin for button press
            // initialize TIM3 in output compare mode to trigger events
                // with a period of 20ms
        }

        void eventservice(void){
            // declare static state and event flag variables
            static int state = 0;
            static unsigned char timerflag = 0;
            static unsigned char buttonflag = 0;
            // check timer event flag register
                // on timeout & state R:
                    // global red_pass = checkGPIOC6();
                    // set state to G
                    // clear red LED
                    // set green LED
                    // start TIM3
                // on timeout & state G:
                    // global green_pass = checkGPIOC6();
                    // set state to B
                    // clear green LED
                    // set blue LED
                    // start TIM3
                // on timeout & state B
                    // global blue_pass = checkGPIOC6();
                    // set state to Output
                    // clear blue LED
                    // print correct color according to truth table
            
            // check button input
                // on button input & state output:
                    \\ set state to R
                    \\ set red LED
                    \\ start TIM3 
                    \\ print 'sampling'

        }

        ```

    ###



