#ifndef DINAMIXEL_MOTOR_HPP
#define DINAMIXEL_MOTOR_HPP

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <stdint.h>
#include <stdexcept>
#include <vector>

// Estructura similar a la de Rozum para mantener consistencia
typedef struct {
    float position;       // 4 bytes: Posición (grados o radianes). Requiere alta precisión decimal.
    float velocity;       // 4 bytes: Velocidad (grados/s o rad/s). Requiere precisión decimal para trayectorias suaves.
    int16_t current;      // 2 bytes: Corriente (Amperios o miliamperios). Un int16_t (-32,768 a 32,767) es perfecto si se maneja en mA.
    int8_t temperature;   // 1 byte: Temperatura en ºC. Un int8_t permite de -128ºC a 127ºC, margen de sobra para el sobrecalentamiento de un motor.
    bool torque_state;    // 1 byte: Estado del torque (activado/desactivado). Un bool es suficiente para este propósito. 
} DinamixelMotorData;

class DinamixelMotor {
private:
    // --- Gestión de comunicación ---
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    uint8_t id;

    // --- Limites del motor y escalas de unidades ---
    const int MAX_VELOCITY = 233;           // Velocidad (rpm)
    const float CONV_RPM_TO_TICK = 1/0.229; 
    const int MAX_POSITION = 200;           // Posición (grados) Por definir
    const int MIN_POSITION = 0;             // Posición (grados) Por definir
    const float CONV_DEG_TO_TICK = 1/0.088; // No se sabe si está bien o mal

    // --- Address table for Dynamixel --- 
    #define OPERATING_MODE 11
    #define POSITION_MODE 3
    #define VELOCITY_MODE 1
    #define READ_TORQUE_ADDRESS 64
    #define READ_CURRENT_ADDRESS 126
    #define READ_VELOCITY_ADDRESS 128
    #define READ_POSITION_ADDRESS 132
    #define READ_TEMPERATURE_ADDRESS 146
    #define WRITE_VELOCITY_ADDRESS 104
    #define WRITE_POSITION_ADDRESS 116
    #define POSITION_LIMIT 4
    #define VELOCITY_LIMIT 48

public:

    // --- Variables ---
    DinamixelMotorData telemetry_motor;
    DinamixelMotorData ref_params_motor;    // Objetivo de referencia para control
    DinamixelMotorData actuation_motor;     // Valores de actuación enviados al motor


    // --- Constructor y destructor ---
    DinamixelMotor(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, uint8_t motor_id);
    ~DinamixelMotor();

    // --- Inicialización y Estado ---
    void set_torque_state(bool state);       // Habilitar Torque
    void set_mode(uint8_t mode);                  // Ejemplo: Velocity o Position mode

    // --- Lectura de Variables (Telemetría) ---
    void read_torque_state();
    void update_telemetry(); // Lectura síncrona o BulkRead
    void add_ID_to_sync_read(dynamixel::GroupSyncRead* groupSyncRead);
    void read_position(dynamixel::GroupSyncRead* groupSyncRead);
    void read_velocity(dynamixel::GroupSyncRead* groupSyncRead);
    void read_current(dynamixel::GroupSyncRead* groupSyncRead);
    void read_temperature(dynamixel::GroupSyncRead* groupSyncRead);

    // --- Control de Movimiento ---
    // void control_PID();
    void set_velocity(dynamixel::GroupSyncWrite* groupSyncWrite);
    void set_position(dynamixel::GroupSyncWrite* groupSyncWrite);

    // --- Gestión de Hardware ---
    void set_position_limits();
    void set_velocity_limits();
    void get_id();
    void set_id(uint8_t new_id);
};

#endif // DINAMIXEL_MOTOR_HPP