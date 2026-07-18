#include <iostream>
#include <thread>
#include <chrono>
#include <array>
#include <atomic>

// Inclusiones de las APIs base (Ajusta las rutas a tu proyecto)
#include "./Rozum-Servo-Drives-API/c/include/api.h"
#include <dynamixel_sdk/dynamixel_sdk.h>[cite: 6]

// Inclusión de la nueva clase global
#include "API_robot/global_manipulator.hpp"

// Variable atómica para controlar el bucle (equivalente a tu variable Running original)
std::atomic<bool> Running(true);

int main() {
    std::cout << "Iniciando sistema de control del brazo manipulador..." << std::endl;

    // ==========================================
    // 1. INICIALIZACIÓN DE PUERTOS
    // ==========================================
    
    // Interfaz CAN para los motores Rozum 
    rr_can_interface_t* rozum_iface = rr_init_interface("/dev/rozum_api");
    if (!rozum_iface) {
        std::cerr << "Error al abrir la interfaz CAN de Rozum." << std::endl;
        return -1;
    }

    // Handlers para los motores Dynamixel 
    dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler("/dev/u2d2_dyn");
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!portHandler->openPort()) {
        std::cerr << "Error al abrir el puerto de Dynamixel." << std::endl;
        return -1;
    }
    if (!portHandler->setBaudRate(57600)) {
        std::cerr << "Error al configurar el BaudRate de Dynamixel." << std::endl;
        return -1;
    }

    // ==========================================
    // 2. ARRANQUE DEL ROBOT
    // ==========================================
    
    // Instanciamos la clase global pasándole las interfaces de hardware ya abiertas
    GlobalManipulator robot(rozum_iface, portHandler, packetHandler);
    
    // Esta función activa el torque, modos de operación y arranca el hilo paralelo de Dynamixel
    robot.init(); 
    std::cout << "Robot inicializado correctamente. Entrando al bucle de control." << std::endl;

    // Arrays para enviar comandos
    std::array<float, 3> arm_target_vels = {0.0f, 0.0f, 0.0f};
    std::array<float, 5> claw_target_vels = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // ==========================================
    // 3. BUCLE DE CONTROL PRINCIPAL
    // ==========================================
    
    // Simulamos el bucle "full_manual_mode" original
    while (Running) {
        
        // A. AQUÍ LEERÍAS EL MANDO O TECLADO (Omitido por brevedad)
        // read_keyboard() o read_controller() modificarían tus objetivos de velocidad
        
        // Simulamos dar una pequeña velocidad a algunos motores para el ejemplo
        arm_target_vels = {5.0f, 2.0f, 0.0f};
        claw_target_vels = {10.0f, 10.0f, 0.0f, 0.0f, 0.0f};

        // B. ENVIAR COMANDOS DE MOVIMIENTO
        // Esto envía la velocidad al brazo en el hilo principal y despierta al hilo secundario 
        // para que envíe las velocidades a la garra de manera simultánea.
        robot.set_velocities(arm_target_vels, claw_target_vels);

        // C. LEER TELEMETRÍA (Variables)
        // Esto lee las posiciones del brazo en este hilo, mientras el hilo paralelo lee las de Dynamixel
        robot.read_positions();
        robot.read_velocities();
        
        // D. RETARDO DE ESTABILIZACIÓN
        // Retardo de 20ms para darle tiempo al sistema y a los motores de responder a la actuación, 
        // tal como se hacía originalmente con SDL_Delay(20).
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        
        // Condición de salida de prueba (puedes quitarla, es para que el ejemplo no sea infinito)
        static int contador = 0;
        if (contador++ > 100) Running = false; 
    }

    // ==========================================
    // 4. CIERRE SEGURO Y LIMPIEZA
    // ==========================================
    std::cout << "\nApagando sistema..." << std::endl;

    // Detiene el hilo de forma segura, apaga el torque de la garra y desactiva los motores
    robot.deinit(); 
    
    // Cierra el puerto serie de Dynamixel
    portHandler->closePort();

    std::cout << "Programa finalizado correctamente." << std::endl;
    return 0;
}