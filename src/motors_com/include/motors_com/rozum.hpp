#ifndef ROZUM_MOTOR_HPP
#define ROZUM_MOTOR_HPP

#include "Rozum-Servo-Drives-API/c/include/api.h" 
#include <stdint.h>

typedef struct {
    float position;      // 4 bytes: Posición (grados o radianes). Requiere alta precisión decimal.
    float velocity;      // 4 bytes: Velocidad (grados/s o rad/s). Requiere precisión decimal para trayectorias suaves.
    int16_t current;     // 2 bytes: Corriente (Amperios o miliamperios). Un int16_t (-32,768 a 32,767) es perfecto si se maneja en mA.
    int8_t temperature;  // 1 byte: Temperatura en ºC. Un int8_t permite de -128ºC a 127ºC, margen de sobra para el sobrecalentamiento de un motor.
} RozumMotorData;        // Estructura almacenar datos del motor

class RozumMotor {
private:
    // --- Punteros de la API de Rozum ---
    rr_servo_t* motor;             // Puntero que se mete en el motor
    rr_can_interface_t* iface;     // Puntero a la interfaz CAN para enviar y recibir mensajes del motor
    
    // --- Variables de telemetría motor ---
    RozumMotorData telemetry_motor; // Estructura para almacenar los datos de telemetría del motor

public:
    // --- Constructor y Destructor ---
    RozumMotor(rr_can_interface_t* interface, int hardware_id);
    ~RozumMotor();

    // --- Inicialización y Estado ---
    void activate();               // Sustituye a activate_motor_arm
    void setup_telemetry_cache();  // Configura la caché para reducir carga en el bus CAN
    void update_cache();           // Actualiza los valores cacheados

    // --- Lectura de Variables (Telemetría) ---
    void read_all();              // Sustituye a read_all_arm
    void read_position();         // Sustituye a read_pos_arm
    void read_velocity();         // Sustituye a read_vel_arm
    void read_current();          // Sustituye a read_current_arm
    void read_temperature();      // Sustituye a read_temp_arm

    // --- Control de Movimiento ---
    // Incluyo un parámetro de límite para replicar la lógica de seguridad que tenías
    void set_velocity(float target_vel, float max_pos_limit); // Sustituye a set_velocity_arm[cite: 3]
    void set_position(float target_pos, float max_pos_limit); // Sustituye a set_position_arm[cite: 3]

    // --- Gestión de Hardware ---
    int get_id();                  // Sustituye a get_motor_id[cite: 3]
    void set_id(int new_id);       // Modifica y guarda el ID en memoria flash[cite: 3]
};

#endif // ROZUM_MOTOR_HPP

