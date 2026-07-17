#include <iostream>
#include <chrono>
#include <thread>
#include <stdexcept>
// Incluye tu cabecera donde está definida la clase DinamixelMotor y las constantes
#include "API_robot/motor_dinamixel.hpp"

int main() {
    std::cout << "--- Inicializando Nodo de Control Dynamixel ---" << std::endl;

    // 1. Inicialización de los gestores de comunicación
    // Instanciamos el puerto en "/dev/u2d2_dyn" y el protocolo 2.0
    dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler("/dev/u2d2_dyn");
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

    // 2. Abrir el puerto y configurar el bus RS-485
    if (!portHandler->openPort()) {
        std::cerr << "Error crítico: No se pudo abrir el puerto serial." << std::endl;
        return -1;
    }
    
    if (!portHandler->setBaudRate(57600)) {
        std::cerr << "Error crítico: No se pudo configurar el baudrate a 57600." << std::endl;
        return -1;
    }

    std::cout << "Puerto abierto correctamente a 57600 bps." << std::endl;

    // 3. Instanciar el objeto motor
    // Utilizamos el ID_1, que corresponde al valor 1 en tus definiciones
    DinamixelMotor motor_prueba(portHandler, packetHandler, 1); // poner ID correcto

    try {
        // 4. Configuración inicial del hardware. El id no está implementado
        // std::cout << "Configurando motor ID " << (int)motor_prueba.get_id() << "..." << std::endl;
        
        // Asignamos el modo de velocidad, que equivale al valor 1.
        motor_prueba.set_mode(VELOCITY_MODE);
        
        // Activamos la potencia del motor (Torque ON)
        motor_prueba.set_torque_state(true); 

        // 5. Bucle continuo de lectura (Telemetría)
        std::cout << "Iniciando lectura de sensores (Lectura individual directa)..." << std::endl;
        
        // Ejecutamos un bucle de 50 iteraciones como ejemplo
        for (int iteracion = 0; iteracion < 50; ++iteracion) {
            
            // Le pasamos 'nullptr' a las funciones para forzar la lectura individual sin GroupSyncRead
            motor_prueba.read_position(nullptr);
            motor_prueba.read_velocity(nullptr);
            motor_prueba.read_current(nullptr);
            motor_prueba.read_temperature(nullptr);

            // Imprimir los datos estructurados en la consola
            std::cout << "[Iteración " << iteracion << "] "
                      << "Pos: " << motor_prueba.telemetry_motor.position << "° | "
                      << "Vel: " << motor_prueba.telemetry_motor.velocity << " RAW | "
                      << "Carga: " << motor_prueba.telemetry_motor.current << " mA | "
                      << "Temp: " << (int)motor_prueba.telemetry_motor.temperature << " °C" << std::endl;

            // Retardo de 50ms para simular el ciclo de control (ej. 20 Hz)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

    } catch (const std::exception& e) {
        // Captura y reporte de cualquier excepción lanzada por la clase DinamixelMotor
        std::cerr << "\nExcepción de Hardware: " << e.what() << std::endl;
    }

    // 6. Apagado seguro (Shutdown)
    std::cout << "\nDesconectando el motor y cerrando el puerto..." << std::endl;
    
    // Deshabilitamos el torque por seguridad
    motor_prueba.set_torque_state(false); 
    
    // Cerramos el puerto físico de comunicaciones
    portHandler->closePort();

    std::cout << "Programa finalizado correctamente." << std::endl;
    return 0;
}