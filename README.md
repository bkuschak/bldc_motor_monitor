# bldc_motor_monitor
This is a motor monitor for the LMS5200 mini-lathe, LMS3990 mini-mill and similar tools that use the XMT-DRV-500C(3) / ZM3404 BLDC motor controller.

This project provides a small 2x16 line LCD display of:
- Spindle RPM and direction
- Spindle revolution counter
- Motor current

It consists of a small PCB hosting the LCD, Arduino Micro R3, and connectors that mate inline with the BLDC sensor cable. It uses an 
optional toroidal transformer to sense AC line current.

This project was motivated by the need to keep an accurate count of spindle revolutions while winding coils on a mini-lathe. 
Since the BLDC motor controller already has a means of tracking spindle revolutions, it was easy to tap into this circuit rather than 
adding external encoders or hall-effect sensors.  

I haven't been able to find much good documentation on this motor controller, but it does seem to be pretty widely used.  
Here's a very low-quality schematic.  
