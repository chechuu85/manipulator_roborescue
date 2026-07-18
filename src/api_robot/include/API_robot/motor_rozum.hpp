#ifndef ROZUM_MOTOR_HPP
#define ROZUM_MOTOR_HPP

#include "api.h"
#include "manipulator_msgs/msg/rozum_motor_data.hpp"
#include <stdint.h>
#include <stdexcept>


class RozumMotor {
private:
    // --- Punteros de la API de Rozum ---
    rr_servo_t* motor;             // Puntero que se mete en el motor
    rr_can_interface_t* iface;     // Puntero a la interfaz CAN para enviar y recibir mensajes del motor
    uint8_t id;                     // ID del motor

    // --- Limites del motor y escalas de unidades ---
    const int MAX_VELOCITY = 50;        // Velocidad (rpm) // 55 en apuntes
    const int MAX_POSITION = 200;       // Posición (grados) Por definir
    const int MIN_POSITION = 0;         // Posición (grados) Por definir


public:
    // --- Variables de telemetría motor ---
    manipulator_msgs::msg::RozumMotorData telemetry_motor;     // Estructura para almacenar los datos de telemetría del motor
    manipulator_msgs::msg::RozumMotorData ref_params_motor;    // Objetivo de referencia para control
    manipulator_msgs::msg::RozumMotorData actuation_motor;     // Valores de actuación enviados al motor


    // --- Constructor y Destructor ---
    RozumMotor(rr_can_interface_t* interface, int hardware_id);
    ~RozumMotor();

    // --- Inicialización y Estado ---
    void activate();               // Sustituye a activate_motor_arm
    void setup_telemetry_cache();  // Configura la caché para reducir carga en el bus CAN
    void update_cache();           // Actualiza los valores cacheados

    // --- Lectura de Variables (Telemetría) ---
    void read_all_parameters();   // Sustituye a read_all_arm
    void read_position();         // Sustituye a read_pos_arm
    void read_velocity();         // Sustituye a read_vel_arm
    void read_current();          // Sustituye a read_current_arm
    void read_temperature();      // Sustituye a read_temp_arm

    // --- Control de Movimiento ---
    // void control_PID();
    void set_velocity(); // Sustituye a set_velocity_arm
    void set_position(); // Sustituye a set_position_arm


    // --- Gestión de Hardware ---
    void set_velocity_limits();    // Establece los límites de velocidad del motor 
    int get_id();                  // Sustituye a get_motor_id
    void set_id(int new_id);       // Modifica y guarda el ID en memoria flash

};

#endif // ROZUM_MOTOR_HPP

