#ifndef DYNAMIXEL_CLAW_HPP
#define DYNAMIXEL_CLAW_HPP

#include "API_robot/motor_dinamixel.hpp" // Asegúrate de ajustar la ruta
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <array>
#include <stdexcept>

class DynamixelClaw {
private:
    // --- Gestión de comunicación ---
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;

public:
    // --- Motores de la garra ---
    // Según la configuración, la garra usa los IDs 1, 2, 3, 5 y 12
    DinamixelMotor motor1;
    DinamixelMotor motor2;
    DinamixelMotor motor3;
    DinamixelMotor motor5;
    DinamixelMotor motor12;

    // --- Constructor y Destructor ---
    DynamixelClaw(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, 
                  uint8_t id1 = 1, uint8_t id2 = 2, uint8_t id3 = 3, uint8_t id5 = 5, uint8_t id12 = 12);
    ~DynamixelClaw();

    // --- Inicialización y Estado ---
    void set_torque_all(bool state);       // Activa/desactiva el torque de los 5 motores
    void set_mode_all(char mode);       // Cambia el modo de operación (ej. VELOCITY_MODE)

    // --- Lectura de Variables Síncrona (Telemetría) ---
    // Estas funciones leen los datos de los 5 motores en un solo paquete para reducir latencia
    void read_all_parameters();
    void read_positions(); 
    void read_velocities();
    void read_currents();
    void read_temperatures();

    // --- Control de Movimiento Síncrono ---
    // Reciben un array con los objetivos (orden: motor1, motor2, motor3, motor5, motor12)
    void set_velocities(const std::array<float, 5>& target_velocities);
    void set_positions(const std::array<float, 5>& target_positions);

    // --- Gestión de Hardware ---
    void set_limits_all(); // Configura límites de posición y velocidad
};

#endif // DYNAMIXEL_CLAW_HPP