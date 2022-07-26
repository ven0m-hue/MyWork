# WinchFirmware
Winch firmware. 


1. Currently incorporates Time based Winch Down for a height of ~16m.
  
    Uses 2 timer interrupts in the inputCapture mode to keep track of the pulses to obtain counts, used in the Winch Up sequence.
    Sys_tick for the heart beat and other time related checks.
  
    External Interrupts:
    1. One Ext interrupt for the Spring Thing.
    2. One Ext interrupt for the Bay Roof.
    3. One Ext interrupt for the Bay Door(Optinal).

  
2. Encoder Count based feedBack for the Winch Up Sequence.



What's Working:

1. Bombay Door Mechanism 
2. Time Based winch down with the lighter payload ie. .5-1kg
3. Winch Up for the lighter payload is working. 


What's in Progress:

1. Winch Down with the Spring thing is left out for the heavier payload.
2. Winch Up sequence based on the Feedback Enocder count. Not yet verified and casuing troubles. 

Near Future:
1. PID Based winch Up sequence.
2. Interfacing the Current Sensor. 


Future:
1. Improving the mechanical parts to incorperate complete PID/Any other control algorithm based Winch Control system.
2. Misc.
