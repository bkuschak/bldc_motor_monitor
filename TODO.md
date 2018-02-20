## TODO

Several improvements should be made on the next rev:
- Fix swapped pins 1,2 on the LCD panel.
- DNS the I2C header, since it interferes with the forward/reverse switch.
- Add a DIP switch setting for forward/reverse sense, since lathe and mill use different direction for 'forward'.
- Add an output header to provide an (open-collector?) rotation and index pulses for use with a CNC controller.
- Avoid FRAM data corruption.
- Consider alternative OLED display with faster response.
