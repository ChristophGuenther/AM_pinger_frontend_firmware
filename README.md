# Pinger Frontend Firmware
 Firmware for the pinger frontend of the IceCube acoustic module.
 Written for Atmega 328p microcontroller (see https://www.microchip.com/en-us/product/ATmega328P).
 The pinger front end receives commands via SPI from the Mini-Mainboard and is responsible for generating the high-voltage pulses that drive the piezo trnasducer to generate acoustic signals. Main tasks of the firmware are:
 * control the switching regulator that charges the capacitors to ~300V
 * read out their voltage
 * control the discharge of them
 * control the relais that connect either the receiver or emitter circuit to the piezo
 * control the full bridge driver that generates the HV rectangle singals that drive the piezo 
 
 # Change notes:
  * based on APU pinger frontend firmware
  * changed from I2C to SPI communication
  * changed external clock from 20MHz to 12MHz
  * added discharge control
  * added new HV DC/DC control (now managed by dedicated chip)
  * number of sends can be set
  * delay after sending can be set
  * select between waveform or sine send mode
  * set duration of sines
# ToDos:
  * make use of ICM trigger / time signals
  * capacitor voltage readout
# Registers:
 * ...
