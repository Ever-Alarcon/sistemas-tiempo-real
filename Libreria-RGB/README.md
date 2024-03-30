Este programa presenta una integración de dos utilidades: el manejo de PWM (Modulación de Ancho de Pulso) y la creación de una librería propia para manejar 2 leds RGB.

## Aclaraciones sobre el Código

> Es importante asegurarse de que todos los archivos `.c`, tanto del programa principal como de la librería, estén incluidos en el archivo `CMakelist`.

En la sección inicial del programa principal se definen los pines correspondientes a cada LED. Se recomienda conectar una resistencia de protección a cada uno de ellos y conectar el terminal común a tierra.


Para que la librería funcione correctamente, es necesario inicializar el temporizador con la función `RGB_TIMER_INIT`. la libreia esta hecha para manejar dos RGBs cada uno presenta su propia funcion de inicialización.


La librería opera utilizando el Timer 0, con canales que van del 0 al 5. La resolución del temporizador se ha ajustado a 8 bits para controlar el ciclo de trabajo, permitiendo que los valores se asemejen a lo que se espera en el formato de imágenes. La función que se encarga de modificar el ciclo de trabajo es `RGB_CHANGE`.
