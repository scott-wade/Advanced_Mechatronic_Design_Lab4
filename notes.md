# Notes for Lab 4

## Next steps
 - configure GPIO C6 as its AF2 (bind to TIM3_CH1)
 - in main, do a while loop checking the value of SR (?)
   - if triggered, print it in the console 


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

Reading the captured value:
    Using software, check for a value in CCxIF
    If the CCxIF pin is 1, read the value in TIMx_CCRx
        Should automatically clear the interrupt flag (?) check this when implementing

### Part 5. LED using PWM mode
● Setup PortB pin 0 as an output so you can access LED1.
● Setup Timer 3 channel 3 in pwm mode and map the output to PortB pin 0.
● Toggle the LED with a frequency of ~0.25Hz

Setting up Tim3 Ch3 in PWM mode:


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


### Part 7. External interrupt 6 using portC pin 6
● Setup PortB pin 0 as an output so you can access LED1.
● Setup PortC pin 6 as an input.
● Enable External interrupt 6 to PortC pin 6.
● Toggle LED1 every time you see a rising edge on PortC pin 6. (Toggle in the
interrupt service routine


