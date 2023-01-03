= NMEA2000_esp32_twai library for ESP32 boards =

Inherited object for use NMEA2000 library for ESP32 Boards.
See also https://github.com/ttlappalainen/NMEA2000[NMEA2000] library.

To use this library, you will also need NMEA2000 library.

The library defines as default Tx pin to GPIO 32 and Rx pint to GPIO 34. You can 
change these with defines:

  #define ESP32_CAN_TX_PIN GPIO_NUM_32
  #define ESP32_CAN_RX_PIN GPIO_NUM_34

before including NMEA2000_CAN.h or NMEA2000_esp32_twai.h

== Thanks ==

Thanks to Timo Lappalainen for his implementation of ESP32 CAN driver that served me as an inspiration.
Since his work was published the ESP32 IDF implemented the official support for CAN (they call it TWAI) controller,
so I decided to implement N2K CAN layer using the official APIs

== License ==


2022 Copyright (c) Sergei Podshivalov  All right reserved.

Author: Sergei Podshivalov

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
