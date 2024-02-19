NRF24L01 + STM8 Compatible C Library (Linux, Windows)
=====================================
This was ported from the mirf library in [esp-idf-mirf](https://github.com/nopnop2002/esp-idf-mirf), which is a port in itself. This only works with the STM8 peripherals library usinng Platformio's SDCC compatible version.

Documentation
=============
Documentation at https://nicogutz.github.io/NRF24_STM8_Library

How to build STM8 MIRF
=====================================

1. (Windows) Install STLink [Drivers](https://www.st.com/en/development-tools/stsw-link009.html). _Make a new account with ST before downloading, their confirmation Email from this site never reaches for some reason._
2. [Install PlatformIO](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html)
3. Install the platformio extension on VSCode.
4. (Linux) Add the udev rules [here](https://docs.platformio.org/en/stable/core/installation/udev-rules.html#platformio-udev-rules) 
5. Clone this repo and open with VSCode.
```shell
$ git clone https://github.com/nicogutz/NRF24_STM8_Library
```
6. Make sure [env:stm8sblack] is selected on the PlatformIO environments.
7. Connect STLink and STM8, build and upload using the buttons at the bottom.

Usage
=====
Please see [MIRF](src/MIRF.md) for information, the equivalent ESP example used here is [Ping-Pong](https://github.com/nopnop2002/esp-idf-mirf/tree/master/Ping-Pong).

Debugging
==========
Debugging is currently only possible using Linux, there is a core issue with SDCC. This might get fixed in the future.
For debuging to work at version 6.11, follow the instructions at https://github.com/platformio/platform-ststm8/issues/61

Notes regarding SPL Setup
=========================

Please see the `src/stm8s_conf.h` file for activating more SPL modules, if you wish to expand the functionality of this example. Only modules (like ADC, UART, etc.) that are activated in the configuration file are compiled in. In this example, only the GPIO module is active. Activating unused modules will result in a higher flash usage that will make even compilation even impossible for smaller chips, care must be taken.
