# 05_UART_RGB

Este código es para un programa en un ESP32 que utiliza FreeRTOS para manejar la comunicación UART y controlar un LED RGB.

Se un RGB cátodo comun conectado a los GPIO 12, 13 y 14, junto a sus resistencias de protección.

Se emplean librerias personalizadas `rgb.h` y `ESP32Utilidades.h`. La primera se usa para el manejo del timer y la segunda guarda la inicialización y el envio basico del UART.

> Se explica más detalladamente en el [main.c](https://github.com/Ever-Alarcon/sistemas-tiempo-real/blob/main/UART-RGB/main/main.c).

El comando a recibir es `LEDR101`, la cuarta letra puede ser R, G o B, y el número es una de tres cifras entre 0 y 255.

## UART

El ESP32-WROOM-32 tiene 3 UART's diferentes:

* UART0: GPIO1(Tx) y GPIO3(Rx). Conectado al USB diracto al PC.
* UART1: GPIO5(Tx) y GPIO4(Rx). Conectado a SPI (SP3-SP2), no recomendado.
* UART2: GPIO17(Tx) y GPIO16(Rx).

> Si se trabaja desde una herramienta para la comunicacion serial desde el PC, se recomienda ir al SDK Configuration Editor (menuconfig) y poner ESP System Settings en None.
