# WAV_player
# Overview
It is possible to play 8 bit .WAV files.We use SPI for sending data from SD card to STM32F407vg. SD card support is provided by FATFS.
We use DMA to transfer samples from .WAV file to Digital Analog Converter
