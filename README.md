 
  NRF24L+ STM8 Compatible Library
=====================================

  This was ported from the mirf library in esp-idf-mirf, which is a port in itself.
  This only works with the STM8 peripherals library usinng Platformio's SDCC compatible
  version.
 
  MIT License
 
  Copyright (c) 2023 nicogutz
 
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
 
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
 
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 
.  Copyright 2018-present PlatformIO <contact@platformio.org>
  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.



How to build PlatformIO based project
=====================================

1. [Install PlatformIO Core](https://docs.platformio.org/page/core.html)
2. 
2. Download [development platform with examples](https://github.com/platformio/platform-ststm8/archive/develop.zip)
3. Extract ZIP archive
4. Run these commands:

```shell
# Change directory to example
$ cd platform-ststm8/examples/spl-blink

# Build project
$ pio run

# Upload firmware
$ pio run --target upload

# Build specific environment
$ pio run -e stm8sdisco

# Upload firmware for the specific environment
$ pio run -e stm8sdisco --target upload

# Clean build files
$ pio run --target clean
```

Notes regarding SPL setup
=========================

Please see the `src/stm8s_conf.h` file for activating more SPL modules, if you wish to expand the functionality of this example. Only modules (like ADC, UART, etc.) that are activated in the configuration file are compiled in. In this example, only the GPIO module is active. Activating unused modules will result in a higher flash usage that will make even compilation even impossible for smaller chips, to care must be taken.
