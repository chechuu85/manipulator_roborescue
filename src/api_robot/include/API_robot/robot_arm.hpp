#ifndef ROZUM_ARM_HPP
#define ROZUM_ARM_HPP

#include "API_robot/motor_rozum.hpp" // Asegúrate de ajustar la ruta según tu proyecto
#include <array>
#include <stdexcept>

class RozumArm {
private:
    // --- Punteros de la API de Rozum ---
    rr_can_interface_t* iface;

public:
    // --- Motores del brazo ---
    RozumMotor motor1;
    RozumMotor motor2;
    RozumMotor motor3;

    // --- Constructor y Destructor ---
    // Recibe la interfaz CAN y los IDs correspondientes a los 3 motores del brazo
    RozumArm(rr_can_interface_t* interface, int id_motor1, int id_motor2, int id_motor3);
    ~RozumArm();

    // --- Inicialización y Estado ---
    void activate_all();               // Activa los 3 motores
    void setup_telemetry_cache_all();  // Configura la caché para los 3 motores
    void update_cache_all();           // Actualiza los valores cacheados de los 3 motores

    // --- Lectura de Variables (Telemetría) ---
    void read_all_parameters();   // Lee todos los parámetros de los 3 motores
    void read_positions();        
    void read_velocities();       
    void read_currents();         
    void read_temperatures();     

    // --- Control de Movimiento ---
    // Reciben un array con los valores objetivo para el motor 1, 2 y 3 respectivamente
    void set_velocities(const std::array<float, 3>& target_velocities);
    void set_positions(const std::array<float, 3>& target_positions);

    // --- Gestión de Hardware ---
    void set_velocity_limits_all();    // Establece los límites de velocidad para los 3 motores
};

#endif // ROZUM_ARM_HPP