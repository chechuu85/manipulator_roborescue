
# API DEL ROBOT

Sirve como puente para para la comunicación del software con el manipulador. 


Las clases que se han estado utilizando son:

## motor_dinamixel.hpp / motor_dinamixel.cpp

### Introducción
La clase DinamixelMotor actúa como una capa de abstracción de bajo nivel fundamental para el control y la monitorización de servomotores Dynamixel dentro del ecosistema de un robot. 
Su propósito principal es gestionar y simplificar la comunicación directa con el hardware a través del protocolo serie, haciendo uso extensivo de la librería dynamixel_sdk. Está diseñada 
para integrarse dentro de un nodo superior de ROS 2, proporcionando un puente eficiente entre los mensajes estructurados del ecosistema de ROS (como los paquetes de telemetría y 
comandos de actuación) y los registros de memoria internos del servomotor.

Dentro de la arquitectura del sistema robótico, esta clase encapsula la complejidad de un único motor, administrando independientemente tres flujos de información críticos: telemetría 
(estado actual real del hardware), referencias (objetivos de control deseados por la lógica superior) y actuación (comandos que finalmente se envían al motor tras pasar por filtros de seguridad). 
Esta separación permite que el controlador de alto nivel delegue la gestión de excepciones de hardware y la sincronización de paquetes.

El nodo/clase soporta modalidades de funcionamiento flexibles, destacando el control por velocidad (Velocity Mode) y el control por posición (Position Mode). Además, soporta operaciones 
de lectura/escritura tanto individuales como en bloque (Bulk/Sync Read-Write), lo que optimiza drásticamente el ancho de banda del bus cuando existen múltiples motores conectados en cadena. 
Incorpora características de seguridad críticas embebidas, como la saturación de comandos fuera de rango y paradas de emergencia lógicas basadas en límites predefinidos de posición y velocidad.

### Diagrama de Flujo

```mermaid
graph LR
    %% Estilos personalizados
    classDef nodeStyle fill:#000000,stroke:#00FF00,stroke-width:3px,color:#FFFFFF;
    
    MotorClass[Clase: DinamixelMotor]:::nodeStyle
```

### Arquitectura y funciones principales

1. Inicialización y Destrucción
- DinamixelMotor(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, uint8_t motor_id)
Constructor principal de la clase. Inicializa la vinculación con las interfaces de comunicación y asigna el identificador del motor. Guarda las variables de entrada en las variables interna de la clase. 

Entradas: dynamixel::PortHandler* port: puerto del canal de comunicaciones, dynamixel::PacketHandler* packet: paquete de recepción de datos, uint8_t motor_id: id del motor al cual queremos llegar
Salidas: void
Parámetros implícitos usados: Variables de clase portHandler, packetHandler, e id.

- ~DinamixelMotor(): Destructor de la clase. Garantiza un apagado seguro relajando el motor antes de su destrucción.
Entradas: void
Salidas: void
Parámetros implícitos usados: Llama a set_torque_state(false).

- set_torque_state(bool state): Habilita o deshabilita el torque (resistencia eléctrica y fuerza física) del motor en el hardware.

Entradas: bool state: indica si se quiere activar/desactivar el torque del motor
Salidas: void
Parámetros implícitos usados: READ_TORQUE_ADDRESS, portHandler, packetHandler, id. Lanza una excepción de tiempo de ejecución en caso de fallo de comunicación.

- set_mode(char mode): Configura el modo operativo en el registro de hardware interno del motor.
Entradas: char mode ('v'/'V' para velocidad, 'p'/'P' para posición)
Salidas: void
Parámetros implícitos usados: VELOCITY_MODE, POSITION_MODE, OPERATING_MODE.

2. Lectura Variables (Telemetría)
- add_ID_to_sync_read(dynamixel::GroupSyncRead* groupSyncRead): Añade el ID de este motor a una lista agrupada para leer datos de múltiples motores de forma simultánea en un solo paquete.

Entradas: dynamixel::GroupSyncRead* groupSyncRead: paquete para recibir todos los datos juntos 
Salidas: void
Parámetros implícitos usados: id. Depende de que el puntero no sea nulo.

- read_all_parameters(dynamixel::GroupSyncRead* groupSyncRead)
Recupera el bloque completo de telemetría (posición, velocidad, corriente y temperatura) desde un paquete sincronizado o ejecutando lecturas secuenciales individuales si se pasa un puntero nulo (fallback).

Entradas: dynamixel::GroupSyncRead* groupSyncRead: paquete para recibir todos los datos juntos
Salidas: void
Parámetros implícitos usados: telemetry_motor (escribe datos aquí), constantes READ_POSITION_ADDRESS, READ_VELOCITY_ADDRESS, READ_CURRENT_ADDRESS, READ_TEMPERATURE_ADDRESS.

- read_position / read_velocity / read_current / read_temperature / read_torque_state
Conjunto de funciones granulares para la lectura de variables específicas. Soportan modo síncrono e individual de manera análoga a la lectura total. En el paquete read_torque_state debería usar
también groupSyncRead pero no se usa por motivo desconocido o porque directamente no se implementó.

Entradas: dynamixel::GroupSyncRead* groupSyncRead (o void en read_torque_state)
Salidas: void
Parámetros implícitos usados: telemetry_motor, Direcciones correspondientes de memoria Dynamixel.

3. Control de movimiento
- set_velocity(dynamixel::GroupSyncWrite* groupSyncWrite)
Envía el comando de velocidad al motor. Implementa un bloque lógico de protección que anula la velocidad (fuerza a 0) si se está intentando mover el motor más allá de sus límites físicos posicionales predefinidos.

Entradas: dynamixel::GroupSyncWrite* groupSyncWrite
Salidas: void
Parámetros implícitos usados: actuation_motor.velocity, actuation_motor.position, MAX_POSITION, MIN_POSITION, WRITE_VELOCITY_ADDRESS.

- set_position(dynamixel::GroupSyncWrite* groupSyncWrite)
Envía la consigna de posición al motor. Antes de transmitir el byte, acota o trunca el comando (actuation_motor.position) para garantizar que siempre esté en el intervalo seguro [MIN_POSITION, MAX_POSITION].

Entradas: dynamixel::GroupSyncWrite* groupSyncWrite: paquete para enviar todos los datos juntos
Salidas: void
Parámetros implícitos usados: actuation_motor.position, WRITE_POSITION_ADDRESS, MAX_POSITION, MIN_POSITION.

4. Gestión de hardware
- set_position_limits() / set_velocity_limits()
Escribe directamente en la EEPROM / RAM del motor los umbrales máximos permitidos a nivel de hardware, otorgando una capa redundante de protección adicional.

Entradas: void
Salidas: void
Parámetros implícitos usados: POSITION_LIMIT, VELOCITY_LIMIT, MAX_POSITION, MAX_VELOCITY.

- get_id/set_id: funciones sin implementar al ya tener el id del motor apuntado. Dudosa utilidad de las funciones. 

### Variables globales
Las variables e interfaces utilizadas pertenecen a la abstracción de la clase y gestionan el estado y la configuración general del entorno del motor:

1. Gestión de comunicación

- dynamixel::PortHandler* portHandler: Puntero gestionado por el SDK de Dynamixel para el manejo asíncrono del puerto serie físico (e.g. /dev/ttyUSB0).

- dynamixel::PacketHandler* packetHandler: Controlador del SDK encargado del empaquetado, desempaquetado, sumas de verificación (checksum) e interpretación del protocolo propio de Dynamixel (Protocolo 1.0 o 2.0).

- dynamixel::GroupSyncRead / dynamixel::GroupSyncWrite: Clases de interfaz del SDK pasadas por referencia para optimizar llamadas al bus con múltiples motores.

2. Variables de Control:

- uint8_t id: Identificador físico de red del motor en el bus RS-485/TTL.

- MAX_VELOCITY, MAX_POSITION, MIN_POSITION: Límites operacionales seguros para el motor (definidos en rpm y grados, condicionados en software).

- CONV_RPM_TO_TICK, CONV_DEG_TO_TICK: Tasas de conversión utilizadas para traducir las unidades del Sistema Internacional (SI) al valor crudo de pulsos/ticks que el microcontrolador del motor espera.

- Direcciones de Memoria (Macros): Variables como OPERATING_MODE, WRITE_VELOCITY_ADDRESS, READ_CURRENT_ADDRESS, etc., que actúan como punteros físicos a la EEPROM/RAM del Dynamixel según su datasheet oficial.

3. Estructuras de Datos:

- telemetry_motor: Objeto del tipo de ROS 2 manipulator_msgs::msg::DinamixelMotorData. Actúa como un contenedor pasivo que almacena la última posición, velocidad, temperatura y corriente leídas con
éxito desde el hardware.

- ref_params_motor: Estructura para almacenar la consigna teórica pura objetivo que demanda el sistema superior.

- actuation_motor: Estructura modificada para el motor. Contiene los valores procesados y validados (tras aplicar las limitaciones lógicas y físicas como std::min/max) que están listos
para ser empaquetados y enviados al hardware.


## motor_rozum.hpp / motor_rozum.cpp

### Introducción
La clase RozumMotor actúa como la interfaz principal de control y comunicación para los actuadores de Rozum Robotics dentro de un ecosistema ROS 2. Su propósito fundamental 
es abstraer la complejidad de la API de bajo nivel (api.h) y la gestión del bus CAN, proporcionando una interfaz orientada a objetos que facilita la integración del hardware con 
el resto del sistema robótico.

Dentro de la arquitectura del robot, este componente se sitúa en la capa de abstracción de hardware (Hardware Abstraction Layer). Juega el papel crítico de traducir los comandos 
de alto nivel (como referencias de posición o velocidad provenientes de un controlador o planificador) en tramas de datos inteligibles para el motor, al mismo tiempo que recopila 
el estado físico real del actuador para cerrar los lazos de control y monitorizar la telemetría del sistema.

El nodo/clase opera bajo dos modalidades principales de control: control por velocidad y control por posición. Incorpora características de seguridad críticas, como la limitación 
de posición mediante rangos predefinidos y la gestión eficiente del ancho de banda del bus CAN mediante un sistema de caché de parámetros, evitando la saturación de la red durante 
lecturas de telemetría a alta frecuencia.


### Diagrama de Flujo
```mermaid
graph LR
    %% Estilos personalizados
    classDef nodeStyle fill:#000000,stroke:#00FF00,stroke-width:3px,color:#FFFFFF;
    
    MotorClass[Clase: RozumMotor]:::nodeStyle
```

### Arquitectura y funciones principales

1. Inicialización y Destrucción
- RozumMotor(rr_can_interface_t* interface, int hardware_id) (Constructor): Inicializa el motor a través de la interfaz CAN proporcionada y establece las variables
de telemetría a sus valores por defecto (cero). Lanza una excepción si el motor no se inicializa correctamente.

Entradas: rr_can_interface_t* interface, int hardware_id
Salidas: void (Constructor)
Parámetros implícitos usados: iface, motor, id, telemetry_motor

- ~RozumMotor() (Destructor)
Garantiza un apagado seguro del hardware. Pone el motor en estado pre-operacional para desactivar la fuerza del actuador, libera los recursos de memoria y anula el
puntero para evitar problemas de punteros colgantes.

Entradas: void
Salidas: void 
Parámetros implícitos usados: motor

- activate(): Transiciona el estado del motor a "operacional", permitiendo que responda a comandos de movimiento. Verifica la validez del puntero antes de actuar.

Entradas: void
Salidas: void
Parámetros implícitos usados: motor, id

- setup_telemetry_cache(): Configura los registros internos del bus CAN para que los parámetros clave (posición, velocidad, corriente y temperatura)
se almacenen en caché automáticamente. Esto reduce la latencia en lecturas consecutivas.

Entradas: void
Salidas: void
Parámetros implícitos usados: motor

- update_cache(): Ejecuta la solicitud de actualización de todos los valores cacheados desde el motor hacia la memoria del ordenador anfitrión.

Entradas: void
Salidas: void
Parámetros implícitos usados: motor

- read_all_parameters(): Lee simultáneamente la posición, velocidad, corriente y temperatura desde la memoria caché y las formatea/almacena en la estructura ROS 2 telemetry_motor.

Entradas: void
Salidas: void
Parámetros implícitos usados: motor, telemetry_motor

- read_position(), read_velocity(), read_current(), read_temperature(): Funciones individuales para leer un parámetro específico desde la caché y guardarlo
en la variable correspondiente de la estructura de telemetría.

Entradas: void para todas.
Salidas: void para todas.
Parámetros implícitos usados: motor, telemetry_motor

- set_velocity(): Envía un comando de velocidad al motor. Implementa una lógica de seguridad crítica: si el motor está excediendo los límites de posición (MAX_POSITION o MIN_POSITION)
y se le ordena moverse en esa dirección, la velocidad se sobreescribe a 0.

Entradas: void
Salidas: void
Parámetros implícitos usados: motor, actuation_motor, MAX_POSITION, MIN_POSITION, id

- set_position(): Envía un comando de posición al motor. Acota (clampea) el valor objetivo utilizando std::max y std::min para garantizar que la referencia enviada nunca exceda el rango seguro establecido.

Entradas: void
Salidas: void
Parámetros implícitos usados: motor, actuation_motor, MAX_POSITION, MIN_POSITION, id

- set_velocity_limits(): Establece el límite de velocidad máxima permitida a nivel de firmware en el controlador del motor.

Entradas: void
Salidas: void
Parámetros implícitos usados: motor, MAX_VELOCITY, id

- get_id(): (Por implementar en el código). Debería retornar el ID actual del motor.

Entradas: void
Salidas: int
Parámetros implícitos usados: Ninguno activo (devuelve 0 por defecto).

- set_id(int new_id): (Por implementar en el código). Debería modificar y guardar el ID en la memoria flash del hardware.

Entradas: int new_id
Salidas: void
Parámetros implícitos usados: Ninguno activo.

### Variables globales
1. Variables de gestión datos motor
- telemetry_motor: Mensaje de tipo manipulator_msgs::msg::RozumMotorData. Representa el Publisher de estado (salida de datos del motor).
- ref_params_motor: Mensaje de tipo manipulator_msgs::msg::RozumMotorData. Representa el Subscriber de control conceptual para planificadores.
- actuation_motor: Mensaje de tipo manipulator_msgs::msg::RozumMotorData. Representa el Subscriber de comandos directos para enviar consignas inmediatas al motor.

2. Interfaces o Librerías Externas:
- rr_servo_t* motor: Puntero principal de la API de Rozum (api.h) que representa la instancia física del actuador.
- rr_can_interface_t* iface: Puntero a la interfaz de hardware CAN, requerida para el enrutamiento de paquetes hacia el motor físico.


3. Variables de Control:
- uint8_t id: Identificador del nodo CAN del motor en el bus.
- const int MAX_VELOCITY: Límite máximo de velocidad de seguridad configurado en 50 rpm.
- const int MAX_POSITION: Límite cinemático superior de seguridad establecido en 200 grados.
- const int MIN_POSITION: Límite cinemático inferior de seguridad establecido en 0 grados.





