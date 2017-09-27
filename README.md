# WAV_player

# Overview

Project is based on STM32F4DISCOVERY.

It is possible to play 8 bit .wav files that are stored on the SD card formatted to FAT32.

# Description

We use SPI for sending data from SD card to STM32. FAT32 support provides the FatFs library. A cyclic list has been implemented to store information about files on the SD card. DMA is used to send sound samples to the DAC. The current time of playing song is displayed on the 8 segment display. To control the volume of songs we use module with volume control. Voltage from volume control is appplying to ADC.

**LED diodes signals** 

Green LED indicates playback. Red LED indicates stopping playback. Blue LED indicates switching the song backwards. Orange LED indicates switching the song forward. 

# Tools

CooCox CoIDE 1.7.8

# How to run

**hardware needed to use :**

1. STM32F407G-DISC1

2. SD card reader module

3. SD Card formatted to FAT32

4. module with speaker, amplifier for example LM386M and volume control

5. module with at least 3 push buttons

6. (optionally) module with at least triple 8 segment display

**connection method**

1. SD card reader module

- GND  ----->  GND
- 5V   ----->  5V
- SDCS ----->  PB11
- MOSI ----->  PB15
- SCK  ----->  PB13
- MISO ----->  PB14
- GND  ----->  GND

2. module with speaker, amplifier and volume control.

- VCC  ----->  3V
- GND  ----->  GND
- 5V   ----->  5V
- ADC1 ----->  PA1
- AIN  ----->  MOSI

and power the speaker (5V).

3. push buttons

- G   ----->  VDD
- K0  ----->  PA5
- K1  ----->  PA7
- K2  ----->  PA8

4. module with 8 segment displays

- a   ----->  PE8
- b   ----->  PE9
- c   ----->  PE10
- d   ----->  PE11
- e   ----->  PE12
- f   ----->  PE13
- g   ----->  PE14
- h   ----->  PE15
- VCC ----->  3V
- 3   ----->  PE4
- 2   ----->  PE3
- 1   ----->  PE2

To start playback press K0 (PA5) button. 

# How to compile

Enough download the project, unpack and compile it with CooCox CoIDE.

# Future improvements

Sometimes while switching tracks, there are loud disruptions.

# Attributions

http://elm-chan.org/fsw/ff/00index_e.html

# License

MIT

# Credits

Przemysław Łapicz

Marek Rybicki

The project was conducted during the Microprocessor Lab course held by the Institute of Control and Information Engineering, Poznan University of Technology.

Supervisor: Marek Kraft


