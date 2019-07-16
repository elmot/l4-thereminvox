Digital Thereminvox
====

[Thereminvox(aka theremin,  ætherphone/etherphone, thereminophone 
or termenvox)]() is an pure electronic musical instrument, which 
does not have nor strings, neither buttons. It reacts on *thereminist* hands
positions.  

![Leon Theremin and his thereminvox](https://upload.wikimedia.org/wikipedia/commons/7/74/Lev_Termen_playing_-_cropped.jpg)

The instrument was invented by Russian electronics engineer and inventor 
[Leon Theremin](https://en.wikipedia.org/wiki/L%C3%A9on_Theremin) in 1920.

Here you may find a video where Leon [demonstrates his instrument](https://www.youtube.com/watch?v=_3H5JbkPXpw).

This project is written in memory of Leon Theremin, almost 100 years after his invention.

The project is digital implementation of a similar idea.

Palms positions are detected by two VL53L1X sensors and then converted into sound.

Hardware Components
====

* [Nucleo64-L476RG MCU board](https://www.st.com/en/evaluation-tools/nucleo-l476rg.html)
* [X-NUCLEO-53L1A1 sensor shield board](https://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32-nucleo-expansion-boards/stm32-ode-sense-hw/x-nucleo-53l1a1.html)
* Minijack 3.5mm plug and cable
* Any speaker with linear input
* Some cables
* USB power supply

Software used
==== 
* [CLion IDE](https://jetrbrains.com/clion)
* [ARM GCC Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
* [MinGW-w64](https://mingw-w64.org/doku.php)
* [OpenOCD](http://openocd.org/)
* [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)