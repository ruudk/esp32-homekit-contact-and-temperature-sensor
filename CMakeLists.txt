cmake_minimum_required(VERSION 3.10)
project(led)

set(CMAKE_CXX_STANDARD 11)

include_directories(main)
include_directories(/Users/Ruud/esp/esp-idf/components/tcpip_adapter/include/)
include_directories(/Users/Ruud/esp/esp-idf/components/esp32/include/)
include_directories(/Users/Ruud/esp/esp-idf/components/driver/include/)
include_directories(/Users/Ruud/esp/esp-idf/components/log/include/)
include_directories(/Users/Ruud/esp/esp-idf/components/nvs_flash/include/)
include_directories(/Users/Ruud/esp/esp-idf/components/freertos/include/)
include_directories(/Users/Ruud/esp32-homekit/esp-homekit-demo/)
include_directories(/Users/Ruud/esp32-homekit/esp-homekit-demo/components/homekit/include/)

add_executable(led
        main/main.c
        main/DHT22.c
        Makefile
        )
