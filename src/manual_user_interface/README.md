# CONTROL MANUAL DEL ROBOT

## keyboard.hpp/keyboard.cpp

### Introducción

Este paquete proporciona un nodo de ROS 2 (keyboard_node) diseñado para la teleoperación en tiempo real de un brazo robótico manipulador. Este nodo implementa una Interfaz Gráfica de Usuario (GUI) utilizando la librería SDL2.

El nodo actúa como el "cerebro de control manual" del sistema, traduciendo las pulsaciones del usuario en comandos de movimiento precisos. Cuenta con una máquina de estados que permite controlar el robot en tres modalidades diferentes:

1. Espacio Articular: Control directo del teclado y velocidad independiente para cada uno de los 8 motores del brazo.

2. Espacio Cartesiano: Movimiento del efector final (TCP) en el espacio 3D (Traslación X, Y, Z y Rotación Roll, Pitch, Yaw), permitiendo alternar entre el marco de referencia de la base o de la herramienta.

3. Modo Trayectoria: Una interfaz tipo Teach Pendant para guardar puntos en el espacio y ordenar la ejecución de trayectorias planificadas.

### Diagrama de Flujo (Mermaid)

```mermaid
graph LR
    A[/Teclado Físico<br/] -->|Hardware| B((keyboard_node))
    B -->|sensor_msgs/JointState| C[/input_articular/]
    B -->|manipulator_msgs/HiperTwist| D[/input_cartesian/]
    B -->|std_msgs/String| E[/input_instr_trayectory/]
    
    classDef node fill:#1e1e1e,stroke:#4CAF50,stroke-width:2px,color:#fff;
    classDef topic fill:#025997,stroke:#fff,stroke-width:1px,color:#fff;
    classDef hw fill:#7f8c8d,stroke:#fff,stroke-width:1px,color:#fff;
    
    class B node;
    class C,D,E topic;
    class A hw;
```



### Arquitectura y funciones principales

1. Inicialización y Destrucción
- KeyboardNode() (Constructor): Inicializa los subsistemas de video y fuentes de la librería SDL2 y crea la ventana gráfica del "Panel de Control". Además, declara el parámetro del temporizador, configura tres publicadores de ROS 2 (para comandos articulares, cartesianos y de trayectoria) y dimensiona los vectores de los mensajes.

Entradas: void
Salidas: void
Parametros implícitos usados: timer_period_ms (control velocidad timer_. proviene del launch) 

- ~KeyboardNode() (Destructor): Garantiza un cierre limpio y seguro del nodo. Se encarga de liberar la memoria de las texturas, cerrar la ventana gráfica y apagar los subsistemas de SDL para evitar fugas de memoria (memory leaks) al detener la ejecución.

Entradas: void
Salidas: void
Parametros implícitos usados: void


2. Bucle Principal de Control
- timer_callback(): Se ejecuta de forma cíclica según el parámetro de entrada 'timer_period_ms'(por defecto cada 20ms). Su función es capturar el estado completo del hardware (teclado) mediante SDL_PumpEvents() en cada interrupción para controlar el estado del nodo y publicar la información que deseé el usuario en cada momento. Actualiza en cada ciclo el mensaje que debe aparecer por pantalla

Entradas: void
Salidas: void
Parametros implícitos usados: timer_ (interrupción llama función), SDL (registra entradas por teclado)


3. Modos de operación
- articular_mode(const Uint8 *state): Se activa cuando el robot está en Modo Manual y Espacio Articular. Mapea teclas específicas para aumentar o disminuir la velocidad de referencia y asigna comandos de movimiento directos a cada articulación de forma independiente, diferenciando entre cada tipo de motor. 

Entradas: state (const Uint8*): puntero que apunta a variable que indica estado del telado
Salidas: void
Parametros implícitos usados: void


- cartesian_mode(const Uint8 *state): Se activa en Modo Manual y Espacio Cartesiano. Traduce las pulsaciones del usuario en un mensaje tipo Twist (velocidades lineales en X, Y, Z y angulares en Roll, Pitch, Yaw). También gestiona la apertura/cierre de la garra y alterna el sistema de coordenadas de referencia (BASE o TCP).

Entradas: state (const Uint8*): puntero que apunta a variable que indica estado del telado
Salidas: void
Parametros implícitos usados: void


- trajectory_mode(const Uint8 *state): Se activa en el Modo Trayectoria. En lugar de enviar comandos de velocidad continua, envía Strings (cadenas de texto) de alto nivel ("SAVE", "HOME", "PLAY") a los nodos encargados de la planificación de movimiento, asegurándose de enviar el comando una sola vez por pulsación.

Entradas: state (const Uint8*): puntero que apunta a variable que indica estado del telado
Salidas: void
Parametros implícitos usados: void


3. Motor Gráfico (UI)

- render_text(const std::string &text, int x, int y, SDL_Color color): Función auxiliar de renderizado. Toma un texto, unas coordenadas y un color, genera una textura temporal utilizando SDL_ttf y la estampa en el buffer de la pantalla gráfica.

Entradas: text (const std::string &): referencia a cadena de texto; x (int): coordenada en el eje X de la ventana; y (int): coordenada en el eje Y de la ventana; color (SDL_Color):
Salidas: void
Parametros implícitos usados: void 


- render_ui(): Dibuja toda la interfaz de usuario en la ventana. Es altamente dinámica: evalúa las variables booleanas de la máquina de estados y pinta en pantalla únicamente los controles, teclas y parámetros (como el marco de referencia actual) que son válidos para el modo en el que se encuentra el operador en ese preciso instante.

Entradas: void
Salidas: void
Parametros implícitos usados: lee directamente los parámetros globales


4. Entrada de Ejecución

- main(int argc, char * argv[]):
La función estándar de C++ y ROS 2. Inicializa el entorno de ROS (rclcpp::init), instancia el KeyboardNode y lo mantiene vivo y escuchando eventos mediante rclcpp::spin() hasta que el usuario decide cerrar el programa.

Entradas: argc (int): indica el número de argumentos o palabras pasadas por la terminal al ejecutar el nodo; argv (char * []): Contiene los textos exactos que el usuario escribió en la terminal al lanzar el comando de ROS 2.
Salidas: return 0 (int): 
Parametros implícitos usados: lee directamente los parámetros globales 

### Parámetros globales


## joystick.hpp/joystick.cpp

Nodo por implementar, debería hacer mismas funciones que el teclado para realizadas por el joystick


## kdlCartesianToJoint.hpp/kdlCartesianToJoint.cpp



## kdlJointToCartesian.hpp/kdlJointToCartesian.cpp
