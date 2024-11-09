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
    * MACRO definitions----------------------------------------------------------*/
    #define SYSTEM_CONTROL_BASE_ADDRESS (0xE000E000)
    #define NVIC_BASE_ADDRESS (SYSTEM_CONTROL_BASE_ADDRESS + 0x100)
    #define NVIC_INTERRUPT_SET_ENABLE_REGISTER_0_31 (NVIC_BASE_ADDRESS)
    #define NVIC_INTERRUPT_SET_ENABLE_REGISTER_32_63 (NVIC_BASE_ADDRESS+0x4)
    #define NVIC_INTERRUPT_SET_ENABLE_REGISTER_64_95 (NVIC_BASE_ADDRESS+0x8)
    #define TIM3_INTERRUPT_BIT (0x20000000)
    void enableNVIC_Timer3(void)
    {
    uint32_t * reg_pointer_32;
    reg_pointer_32 = (uint32_t *)NVIC_INTERRUPT_SET_ENABLE_REGISTER_0_31;
    *reg_pointer_32 = TIM3_INTERRUPT_BIT;
    }
Configure Tim3 CH3 as an output compare with the interrupts enabled 
    * MACRO definitions----------------------------------------------------------*/
    #define TIM3_BASE_ADDRESS ((uint32_t)0x40000400)
    // Timer 3 control register 1
    #define TIM3_CR1_REGISTER_1 (TIM3_BASE_ADDRESS + 0x00)
    //flags for CR1 register:
    #define COUNTER_ENABLE_BIT (uint16_t)0x01
    // Timer 3 status register
    #define TIM3_STATUS_REGISTER (TIM3_BASE_ADDRESS + 0x10)
    //flags for Status register:
    #define TIM_UIF 0x01 //timer 3 overflow flag
    #define TIM_CH1_CC1IF 0x02 //timer channel 1 capture/compare event
    #define TIM_CH3_CC3IF 0x8 //timer channel 3 capture/compare event
    //timer 3 interrupt enable register
    # define TIM3_INTERRUPT_ENABLE_REGISTER (TIM3_BASE_ADDRESS + 0x0C)
    //flags for interrupt enable register:
    #define TIM_CH3_CC_INTERRUPT_ENABLE 0x8 //timer channel 3 capture/compare interrupt
    #define TIM_UPDATE_INTERRUPT_ENABLE 0x1 //timer overflow or event interrupt
    //Capture compare enable register
    #define TIM3_CAPTURE_COMPARE_ENABLE_REGISTER (TIM3_BASE_ADDRESS + 0x20)
    //flags for TIM3_CCER registers for output:
    #define TIM3_CCER_CC3E (0x0100)
    //Capture compare mode registers
    #define TIM3_CAPTURE_COMPARE_MODE_1_REGISTER (TIM3_BASE_ADDRESS + 0x18)
    #define TIM3_CAPTURE_COMPARE_MODE_2_REGISTER (TIM3_BASE_ADDRESS + 0x1C)
    //flags for Capture compare mode register
    #define TIM_CCMR13_OCPE (0b00001000) // enable preload register channels 1 and 3
    // Compare, autoreload and Prescaler registers
    #define TIM3_COMPARE_1_REGISTER (TIM3_BASE_ADDRESS + 0x34)
    #define TIM3_COMPARE_2_REGISTER (TIM3_BASE_ADDRESS + 0x38)
    #define TIM3_COMPARE_3_REGISTER (TIM3_BASE_ADDRESS + 0x3C)
    #define TIM3_COMPARE_4_REGISTER (TIM3_BASE_ADDRESS + 0x40)
    #define TIM3_PRESCALER_REGISTER (TIM3_BASE_ADDRESS + 0x28)
    #define TIM3_AUTORELOAD_REGISTER (TIM3_BASE_ADDRESS + 0X2C)
    void initTimer3ToInterrupt( void )
    {
    uint16_t * reg_pointer_16;
    uint16_t prescalervalue2, autoreloadvalue;
    /* Timer 3 APB clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    /*enable the interrupt that would go to timer 3*/
    enableNVIC_Timer3();
    /* Compute Prescale and Autorreload */
    prescalervalue2 = 99; //Frequency of clock is 90 MHz
    autoreloadvalue = 65500;
    /* Clear any pending flags in the status register */
    reg_pointer_16 = (uint16_t *)TIM3_STATUS_REGISTER;
    *reg_pointer_16 = 0;
    /* Set Prescale and Autorreload */
    reg_pointer_16 = (uint16_t *)TIM3_PRESCALER_REGISTER;
    *reg_pointer_16 = prescalervalue2;
    reg_pointer_16 = (uint16_t *)TIM3_AUTORELOAD_REGISTER;
    *reg_pointer_16 = autoreloadvalue;
    /* Set Compare Value */
    reg_pointer_16 = (uint16_t *)TIM3_COMPARE_3_REGISTER;
    *reg_pointer_16 = autoreloadvalue/4;
    /* Enable Preload Register (Don’t HAVE to, but good practice) */
    reg_pointer_16 = (uint16_t *)TIM3_CAPTURE_COMPARE_MODE_2_REGISTER;
    *reg_pointer_16 = *reg_pointer_16 | TIM_CCMR13_OCPE;
    /*enable the TIM3 channel 3 counter and keep the default configuration for channel polarity*/
    reg_pointer_16 = (uint16_t *)TIM3_CAPTURE_COMPARE_ENABLE_REGISTER;
    *reg_pointer_16 = *reg_pointer_16 | TIM3_CCER_CC3E;
    /*enable interrupt on capture compare channel 3*/
    reg_pointer_16 = (uint16_t *)TIM3_INTERRUPT_ENABLE_REGISTER;
    *reg_pointer_16 = (TIM_CH3_CC_INTERRUPT_ENABLE | TIM_UPDATE_INTERRUPT_ENABLE);
    /*enable timer 3*/
    reg_pointer_16 = (uint16_t *)TIM3_CR1_REGISTER_1;
    *reg_pointer_16 = *reg_pointer_16 | COUNTER_ENABLE_BIT;
    }

In the interrupt service: 
    Make the normal interrupt turn the LED on
    Make the overflow interrupt turn the LED off and reset both interrupts 

    void TIM3_IRQHandler(void)
    {
    uint16_t * reg_pointer_16_sr;
    uint16_t * reg_pointer_16_dier;
    reg_pointer_16_sr = (uint16_t *)TIM3_STATUS_REGISTER;
    reg_pointer_16_dier = (uint16_t *)TIM3_INTERRUPT_ENABLE_REGISTER;
    //check which interrupts fired and if they were supposed to fire, then clear the flags so they don’t keep firing,
    // then perform actions according to these interrupts
    //check if Output Compare 3 triggered the interrupt:
    if (( (*reg_pointer_16_sr & TIM_CH3_CC3IF) >0) && ( (*reg_pointer_16_dier & TIM_CH3_CC_INTERRUPT_ENABLE) >0))
    {
    //clear interrupt
    *reg_pointer_16_sr = ~((uint16_t)TIM_CH3_CC3IF);
    //perform action
    clearGPIOB0();
    }
    //check if Overflow triggered the interrupt: I.e. Timer Counter 3 >= Autorreload value
    if (( (*reg_pointer_16_sr & TIM_UIF) >0) && ( (*reg_pointer_16_dier & TIM_UPDATE_INTERRUPT_ENABLE) >0))
    {
    //clear interrupt
    *reg_pointer_16_sr = ~((uint16_t)TIM_UIF);
    //perform action
    setGPIOB0();
    }
    }





### Part 7. External interrupt 6 using portC pin 6
● Setup PortB pin 0 as an output so you can access LED1.
● Setup PortC pin 6 as an input.
● Enable External interrupt 6 to PortC pin 6.
● Toggle LED1 every time you see a rising edge on PortC pin 6. (Toggle in the
interrupt service routine


