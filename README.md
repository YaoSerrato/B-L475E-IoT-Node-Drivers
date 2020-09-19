# B-L475E MCU drivers #

This repository contains the peripheral drivers for the [STM32L475VGT6](https://www.st.com/en/microcontrollers-microprocessors/stm32l475vg.html) microcontroller featured in the [B-L475E IoT Node](https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html) development kit. This is currently a work in progress which considers two main stages:

1. Development of the MCU peripheral drivers (RCC, GPIO, SPI, I2C, USART, ADC, etc).
2. Integration of the developed MCU drivers to the various external modules the development kit posses (Bluetooth module, WiFi module, 3-axis magnetometer, 3D accelerometer, 3D gyroscope, etc).


## Project structure ##
Up to now, there is progress in the development of the following drivers:
* Reset and Clock Control (RCC)
* General-Purpose I/Os (GPIO)
* Serial Peripheral Interface (SPI)
* Universal Synchronous/Asynchronous Receiver Transmitter (USART/UART)


## Development environment ##
Currently I am developing the project on the STM32CubeIDE v1.4.0.
