
This embedded development project builds an approachable Mbed RTOS example with multiple threads, showcasing important serial communication protocols: I2C, SPI, UART. 
A Nucleo F401RE development board is the base. The code is written in C++.
It utilises a gyro for I2C, a shift register with an LCD for SPI and a BLE passthrough for UART. Written in C++.

What the embedded system will do? -> watch the attached demo video to get a quick impression
It will be to continousley process the tilt of the gyroscope and display the angle of pitch and roll on the LCD (using both I2C and SPI respectivley).
It will be able to receive a set of bluetooth instructions from a phone to switch an LED on and off (using I2C).
It will be able to read out message received via blutooth via a serial monitor setup (using USB serial).

Parts: 
- Nucelo-F401RE (any Cortex M ARM based development board with sufficent serial connection will suffice)
- 8-bit shift register (a Texas Instruments SN74HC595 was used)
- 1602A LCD (this is an inexpensive LCD, here operated in 4-bit mode)
- 10k Ohm Potentionometer (to manage contrast of LCD)
- MPU9250 Gyro (used for I2C)
- 2 x 2.2k Ohms resistors (as pull ups for I2C connection)
- Small LED, either with inbuild resistor or get an extra resistor to wiring in series with it
- DFR0781 Bluetooth Module with BLE pass-through (used for UART and inexpesive; you may of course use any other BLE module, but ensure you match the frequency)
- Jumper cables
- 2 x Breadboard

Wiring:
To be added soon

