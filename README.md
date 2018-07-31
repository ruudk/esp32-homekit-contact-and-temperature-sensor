# ESP32 Homekit Demo

Just playing around with [esp-homekit](https://github.com/maximkulkin/esp-homekit) and a breadboard.

Breadboard has a reed switch and a DHT11 temperature sensor.

## How to install

1. Initialize and sync all submodules (recursively):
```shell
git submodule update --init --recursive
```
2. Remove 2 files that are not needed:
```shell
rm components/homekit/src/mdnsresponder.*
```
3. Copy wifi.h.sample -> wifi.h and edit it with correct WiFi SSID and password.
4. Install [esp-idf](https://github.com/espressif/esp-idf) by following [instructions on esp-idf project page](https://github.com/espressif/esp-idf#setting-up-esp-idf). At the end you should have xtensa-esp32-elf toolchain in your path and IDF_PATH environment variable pointing to esp-idf directory.
5. Configure project:
```
make menuconfig
```
There are many settings there, but at least you should configure "Serial flasher config -> Default serial port".
Also, check "Components -> HomeKit" menu section.

6. Build example:
```shell
make all
```
7. To prevent any effects from previous firmware (e.g. firmware crashing right at
   start), highly recommend to erase flash:
```shell
    make erase_flash
```
8. Upload firmware to ESP32:
```shell
    make flash
    make monitor
```
To exit the monitor, simply press `CONTROL + ]`
