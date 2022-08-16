# WinchFirmware
Winch firmware. 


1. Currently incorporates Time based Winch Down for a height of ~16m.
  
    Uses 2 timer interrupts in the inputCapture mode to keep track of the pulses to obtain counts, used in the Winch Up sequence.
    Sys_tick for the heart beat and other time related checks.
  
    External Interrupts:
    * One Ext interrupt for the Spring Thing.
    * One Ext interrupt for the Bay Roof.
    * One Ext interrupt for the Bay Door(Optinal).

    Uses ADC in the DMA mode for capturing the currenst sensor data. 

    Uses One more timer interrupt in the input caputre mode to Start the Winch Sequence by the pixhawk FCU only if a pre-defined (specific) signal is sent.  

  
2. Encoder Count based feedBack for the Winch Up Sequence.

3. Intefaced with the current sensor.

4. Test Data 
  * Test Data which includes Raw PWM, Enocoder Counts, Current measurement data.


What's Working:

1. Bombay Door Mechanism 
2. Time Based winch down with the lighter payload ie. .5-1kg
3. Winch Up for the lighter payload is working. 

4. Winch Down with the Spring thing tested for the heavier payload 2-3kg.
5. Winch Up sequence based on the Feedback Enocder count. Verified.



What's in Progress:
1. It  would be nice for it to winch up just after the spring gets triggered to avoid slack.

TODO:

1. DeBouncing Effect with the interrupt mode, possibly using the TimePeriodElaspsed Callback
   in order to dampen out the effect.
 
2. Figure out why is the close door false triggering.

Near Future:
1. PID Based winch Up sequence.


Future:
1. Improving the mechanical parts to incorperate complete PID/Any other control algorithm based Winch Control system.
2. Serial Protocol based communication between PixHawk and the WinchController. (UAV-CAN for example.)
3. Ability for the WinchController to decipher the MAVLINK Packets. 






## STM32F446 PORT MAPPING

PC0 --> Winch MotorDirection Pin

PA6 --> Winch Motor PWM Pin

Gnd --> Winch Gnd Pin


PA8 --> Bombay Door Direction Pin

PA7 --> Bombay Door PWM Pin

Gnd --> Bombay Door Gnd Pin


PC1 --> Spring Thing ext Int Pin

PC2 --> Bay Roof ext Int Pin


PA0 --> Encoder Tim2 Int Pin 1

PA1 --> Encoder Tim2 Int Pin 2


PA4 --> ADC DMA Pin for Current Sensor


PB6 --> PixHawk Pin(Start Sequence)

Misc:
During testing communication on com port

PA2 --> UART Tx 

PA3 --> UART Rx

# Hardware Used 
1. STM32F446 Microcontroller -> Winch Controller
2. 2 Channel Cytron -> Motor Driver  
3. 12V DC Motor -> Winch Motor
4. 12V DC Motor -> Bombay Door Motor 
5. ACS712 Sensor -> Current Sensor 
6. Hall Effect Encoder -> Encoder 
7. Limit Switch -> Spring Thing 
8. Limit Switch -> Bombay Door
9. Battery -> 12V Li-Ion
