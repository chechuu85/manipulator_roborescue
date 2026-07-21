#include <iostream>
#include <thread>
#include <chrono>
#include <array>
#include <atomic>

// Inclusiones de las APIs base
#include "api.h"
#include <dynamixel_sdk/dynamixel_sdk.h>

// Inclusión de ROS2 y la nueva clase del nodo global
#include "rclcpp/rclcpp.hpp"
#include "API_robot/global_manipulator.hpp"

// Variable atómica para controlar el bucle global de ejecución
extern std::atomic<bool> Running; // O la que uses en tu proyecto

int main(int argc, char * argv[]) {
    std::cout << "Iniciando sistema de control del brazo manipulador con ROS2..." << std::endl;

    // INICIALIZAR EL ENTORNO DE ROS2
    rclcpp::init(argc, argv);

    // ==========================================
    // INICIALIZACIÓN DE PUERTOS FÍSICOS
    // ==========================================
    
    // Interfaz CAN para los motores Rozum 
    rr_can_interface_t* rozum_iface = rr_init_interface("/dev/ttyACM0");
    if (!rozum_iface) {
        std::cerr << "Error al abrir la interfaz CAN de Rozum." << std::endl;
        return -1;
    }

    // Handlers para los motores Dynamixel 
    dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
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
    // ARRANQUE DEL ROBOT Y NODO ROS2
    // ==========================================
    
    // Instanciamos la clase global que hereda de rclcpp::Node
    auto robot = std::make_shared<GlobalManipulator>(rozum_iface, portHandler, packetHandler);
    
    // Activa el torque, modos de operación y arranca el hilo interno de Dynamixel
    robot->init(); 
    std::cout << "Robot inicializado correctamente." << std::endl;

    // 4. LANZAR EL SPIN DE ROS2 EN UN HILO SECUNDARIO EXCLUSIVO
    // Esto evita que rclcpp::spin() bloquee el hilo principal del brazo/Rozum.
    // El nodo procesará sus suscripciones (ej. "keyboard_input") y publicaciones en este hilo.
    std::thread ros_thread([robot]() {
        rclcpp::spin(robot);
    });

    // ==========================================
    // HILO PRINCIPAL: CONTROL DE ROZUM Y GUI
    // ==========================================
    // Aquí mantienes el control absoluto del hilo principal para los motores Rozum,
    // la lectura local (si aplica) y la interfaz visual SDL.
    
    std::array<float, 3> arm_target_vels = {0.0f, 0.0f, 0.0f};
    std::array<float, 5> claw_target_vels = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // Bucle principal (ej. tu modo manual o bucle de renderizado)
    while (Running) {
        
        // A. Aquí puedes gestionar eventos locales o dejar que el nodo ROS2 
        // reciba los comandos externos y los sincronice con el brazo.
        
        // B. Envío y control de velocidades para Rozum en el hilo principal
        // robot->set_velocities(...);

        // C. Retardo de estabilización del bucle (ej. 20ms como tu SDL_Delay)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // ==========================================
    // CIERRE SEGURO Y LIMPIEZA ORDENADA
    // ==========================================
    std::cout << "\nApagando sistema..." << std::endl;

    // Detenemos ROS2 para que el spin termine
    rclcpp::shutdown();

    // Esperamos a que el hilo de ROS2 finalice limpiamente
    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    // Detiene el hilo de Dynamixel, quita los torques y desactiva dispositivos
    robot->deinit(); 
    
    // Cierra el puerto serie de Dynamixel
    portHandler->closePort();

    std::cout << "Programa finalizado correctamente." << std::endl;
    return 0;
}