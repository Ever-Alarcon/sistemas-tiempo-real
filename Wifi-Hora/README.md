# ESP32 Wi-Fi APSTA y SNTP

Este proyecto configura un ESP32 para operar en modo APSTA (Access Point y Station) y sincroniza la hora actual utilizando el protocolo NTP (Network Time Protocol). El código establece una red Wi-Fi, se conecta a un servidor NTP y obtiene la hora actual, además de ejecutar una tarea que muestra la hora en intervalos regulares.

## Descripción

### Funcionalidades
* Modo APSTA: El ESP32 actúa como un punto de acceso y estación al mismo tiempo.
* Sincronización de Tiempo: Utiliza el protocolo SNTP para obtener la hora actual desde un servidor NTP.
* Tarea Programada: Muestra la hora actual cada cierto intervalo de tiempo.

### Definiciones y Configuración
* Modo STA (Station):
- `ESP_WIFI_SSID`: SSID de la red Wi-Fi a la que se conecta.
- `ESP_WIFI_PASS`: Contraseña de la red Wi-Fi.
* Modo AP (Access Point):
- `ESP_WIFI_MY_SSID`: SSID del punto de acceso.
- `ESP_WIFI_MY_PASS`: Contraseña del punto de acceso.
- `ESP_WIFI_CHANNEL`: Canal del punto de acceso.
- `MAX_STA_CONN`: Número máximo de estaciones que se pueden conectar al AP.

### Funciones Principales
1. `NVS_init()`: Inicializa la memoria no volátil (NVS) para el almacenamiento de configuraciones.

2. `wifi_init()`: Configura el ESP32 para funcionar en modo APSTA. Inicializa la red Wi-Fi y maneja eventos de conexión.

3. `initialize_sntp()`: Configura y activa el cliente SNTP para sincronizar el tiempo con un servidor NTP.

4. `obtain_time()`: Obtiene la hora actual del servidor NTP y ajusta la zona horaria a la de Colombia (UTC-5).

5. `task_time()`: Tarea FreeRTOS que muestra la hora actual en intervalos regulares definidos por un parámetro flotante.

6. `app_main()`: Función principal que inicializa el sistema, configura Wi-Fi, obtiene la hora y crea la tarea task_time.



## Notas
Asegúrate de reemplazar los valores de las credenciales de Wi-Fi y otros parámetros de configuración con los adecuados para tu red.

La tarea `task_time` se ejecuta en un intervalo que se puede ajustar en `delay_in_seconds_task_time`.

Recuerda que el tamaño de la flash puede marcarse como 4MB.

El monitor muestra algo como:

## Salida de Consola

```plaintext
I (515) main_task: Calling app_main()
I (545) wifi : ESP_WIFI_MODE_APSTA
I (585) wifi_init: rx ba win: 6
I (585) wifi_init: tcpip mbox: 32
I (585) wifi_init: udp mbox: 6
I (585) wifi_init: tcp mbox: 6
I (595) wifi_init: tcp tx win: 5760
I (595) wifi_init: tcp rx win: 5760
I (605) wifi_init: tcp mss: 1440
I (605) wifi_init: WiFi IRAM OP enabled
I (605) wifi_init: WiFi RX IRAM OP enabled
I (625) phy_init: phy_version 4791,2c4672b,Dec 20 2023,16:06:06
I (725) wifi : wifi_init_sta finished.
I (725) wifi : wifi_init_softap finished. SSID:AccessPoint_18 password:qwertyumm channel:1
I (725) esp_netif_lwip: DHCP server started on interface WIFI_AP_DEF with IP: 192.168.4.1
I (745) wifi : retry to connect to the AP
I (745) wifi : connect to the AP fail
I (3585) wifi : retry to connect to the AP
I (3585) wifi : connect to the AP fail
I (4645) esp_netif_handlers: sta ip: 192.168.1.76, mask: 255.255.255.0, gw: 192.168.1.254
I (4645) wifi : got ip:192.168.1.76
I (4645) wifi : connected to ap SSID:Xx password:Diego257b13
I (4655) wifi : Inicializando SNTP
I (4655) wifi : Esperando obtener la hora... (1/10)
I (7665) wifi : Hora actual obtenida correctamente.
I (7665) wifi : La hora es: Mon Sep  9 23:14:07 2024

I (7665) main_task: Returned from app_main()
I (10185) wifi : La fecha es: Mon Sep  9 23:14:09 2024

I (12695) wifi : La fecha es: Mon Sep  9 23:14:12 2024

I (15205) wifi : La fecha es: Mon Sep  9 23:14:14 2024

I (17715) wifi : La fecha es: Mon Sep  9 23:14:17 2024

I (20225) wifi : La fecha es: Mon Sep  9 23:14:19 2024

I (22735) wifi : La fecha es: Mon Sep  9 23:14:22 2024

I (25115) esp_netif_lwip: DHCP server assigned IP to a client, IP is: 192.168.4.2
I (25245) wifi : La fecha es: Mon Sep  9 23:14:24 2024

I (27745) wifi : La fecha es: Mon Sep  9 23:14:27 2024

I (30255) wifi : La fecha es: Mon Sep  9 23:14:29 2024

I (32765) wifi : La fecha es: Mon Sep  9 23:14:32 2024

I (35275) wifi : La fecha es: Mon Sep  9 23:14:34 2024

I (37785) wifi : La fecha es: Mon Sep  9 23:14:37 2024

I (40295) wifi : La fecha es: Mon Sep  9 23:14:39 2024

I (42795) wifi : La fecha es: Mon Sep  9 23:14:42 2024

I (45305) wifi : La fecha es: Mon Sep  9 23:14:44 2024

I (47815) wifi : La fecha es: Mon Sep  9 23:14:47 2024

I (50325) wifi : La fecha es: Mon Sep  9 23:14:49 2024

I (52825) wifi : La fecha es: Mon Sep  9 23:14:52 2024

I (55335) wifi : La fecha es: Mon Sep  9 23:14:54 2024

I (57835) wifi : La fecha es: Mon Sep  9 23:14:57 2024

I (60345) wifi : La fecha es: Mon Sep  9 23:14:59 2024

I (62855) wifi : La fecha es: Mon Sep  9 23:15:02 2024

I (65365) wifi : La fecha es: Mon Sep  9 23:15:04 2024

