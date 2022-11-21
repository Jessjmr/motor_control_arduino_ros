# motor_control_arduino_ros

Arreglo: Libreria de Arduino para compilar correctamente

https://github.com/ros-drivers/rosserial/issues/518

** Modificar el archivo msg.h que se encuentra en /Rosserial_Arduino_Library/src/ros
** Linea 40

Quitar
< #include <cstring>
---
Poner
> #include <string.h>

** Linea 69

Quitar
<     std::memcpy(&val, &f, sizeof(val));
---
Poner
>     memcpy(&val, &f, sizeof(val));

** Linea 184

Quitar
<     std::memcpy(f, &val, sizeof(val));
---
Poner
>     memcpy(f, &val, sizeof(val));

Arreglo: Configuración de Arduino Due con ROS serial

https://github.com/ros-drivers/rosserial/issues/113

** Modificar el archivo ArduinoHardware.h que se encuentra en /Rosserial_Arduino_Library/src
** Revisar que la siguiente linea este correcta, que diga UART y no USART
 #include <UARTClass.h>  // Arduino Due
 ** Modificar la linea 75 para que quede de la siguiente forma
 iostream = &Serial;
 ** Lamentablemente, este cambio hará que no puedas usar un Arduino Leonardo.
 ** En el código de Arduino
 
 #define USE_USBCON // Add this line before the #include <ros.h> line of code for Arduino Due
 #include <ros.h>
 
