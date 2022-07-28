# WinchFirmware
Winch firmware. 


1. Currently incorporates Time based Winch Down for a height of ~16m.
  
    Uses 2 timer interrupts in the inputCapture mode to keep track of the pulses to obtain counts, used in the Winch Up sequence.
    Sys_tick for the heart beat and other time related checks.
  
    External Interrupts:
    * One Ext interrupt for the Spring Thing.
    * One Ext interrupt for the Bay Roof.
    * One Ext interrupt for the Bay Door(Optinal).

  
2. Encoder Count based feedBack for the Winch Up Sequence.

What's Working:

1. Bombay Door Mechanism 
2. Time Based winch down with the lighter payload ie. .5-1kg
3. Winch Up for the lighter payload is working. 

4. Winch Down with the Spring thing tested for the heavier payload.
5. Winch Up sequence based on the Feedback Enocder count. Verified.

What's in Progress:
1. It  would be nice for it to winch up just after the spring gets triggered to avoid slack.

TODO:

1. DeBouncing Effect with the interrupt mode, possibly using the TimePeriodElaspsed Callback
   in order to dampen out the effect.
 
2. Figure out why is the close door false triggering.

Near Future:
1. PID Based winch Up sequence.
2. Interfacing the Current Sensor. 


Future:
1. Improving the mechanical parts to incorperate complete PID/Any other control algorithm based Winch Control system.
2. Misc.
