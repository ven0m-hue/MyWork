# WinchFirmware
Winch firmware. 


1. Currently incorporates Time based Winch Down for a height of ~16m.
  
    Uses 2 timer interrupts in the inputCapture mode to keep track of the pulses to obtain counts, used in the Winch Up sequence.
    Sys_tick for the heart beat and other time related checks.
  
    External Interrupts:
    1. One Ext interrupt for the Spring Thing.
    2. One Ext interrupt for the Bay Roof.
    3. One Ext interrupt for the Bay Door.

  
2. Encoder Count based feedBack for the Winch Up Sequence.
