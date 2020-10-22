# Electronic Speed Control Software
A motor driver board was created for a remote-controlled continuous tread vehicle.  

The repository for the EAGLE schematics is "electronic-speed-controller-pcb".

The board contains a DRV8833 motor driver, atmega328p microcontroller and a HM-11 Bluetooth Low Energy module.

This repository is the C-code that was used to control the motors with PWM and USART RX / TX for the HM-11 module.

It is primarily based on a state machine where a direction/speed control signal is received from a bluetooth control device.

The state machine output is Pulse Width Modulation(PWM) to the motor driver for direction/speed.
