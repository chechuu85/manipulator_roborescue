#include "API_robot/robot_arm.hpp"

// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================

RozumArm::RozumArm(rr_can_interface_t* interface, int id_motor1, int id_motor2, int id_motor3)
    : iface(interface), 
      motor1(interface, id_motor1), 
      motor2(interface, id_motor2), 
      motor3(interface, id_motor3) 
{
    if (iface == nullptr) {
        throw std::runtime_error("Error al inicializar RozumArm: La interfaz CAN es nula.");
    }
}

RozumArm::~RozumArm() {
    // Los destructores de motor1, motor2 y motor3 se llamarán automáticamente
}

// ==========================================
// INICIALIZACIÓN Y ESTADO
// ==========================================

void RozumArm::activate_all() {
    motor1.activate();
    motor2.activate();
    motor3.activate();
}

void RozumArm::setup_telemetry_cache_all() {
    motor1.setup_telemetry_cache();
    motor2.setup_telemetry_cache();
    motor3.setup_telemetry_cache();
}

void RozumArm::update_cache_all() {
    motor1.update_cache();
    motor2.update_cache();
    motor3.update_cache();
}

// ==========================================
// LECTURA DE VARIABLES (TELEMETRÍA)
// ==========================================

void RozumArm::read_all_parameters() {
    update_cache_all(); // Aseguramos que la caché esté actualizada antes de leer
    motor1.read_all_parameters();
    motor2.read_all_parameters();
    motor3.read_all_parameters();
}

void RozumArm::read_positions() {
    update_cache_all();
    motor1.read_position();
    motor2.read_position();
    motor3.read_position();
}

void RozumArm::read_velocities() {
    update_cache_all();
    motor1.read_velocity();
    motor2.read_velocity();
    motor3.read_velocity();
}

void RozumArm::read_currents() {
    update_cache_all();
    motor1.read_current();
    motor2.read_current();
    motor3.read_current();
}

void RozumArm::read_temperatures() {
    update_cache_all();
    motor1.read_temperature();
    motor2.read_temperature();
    motor3.read_temperature();
}

// ==========================================
// CONTROL DE MOVIMIENTO
// ==========================================

void RozumArm::set_velocities(const std::array<float, 3>& target_velocities) {
    // Asignar los objetivos de actuación a las estructuras internas
    motor1.actuation_motor.velocity = target_velocities[0];
    motor2.actuation_motor.velocity = target_velocities[1];
    motor3.actuation_motor.velocity = target_velocities[2];

    // Ejecutar el comando para los tres motores
    motor1.set_velocity();
    motor2.set_velocity();
    motor3.set_velocity();
}

void RozumArm::set_positions(const std::array<float, 3>& target_positions) {
    // Asignar los objetivos de actuación a las estructuras internas
    motor1.actuation_motor.position = target_positions[0];
    motor2.actuation_motor.position = target_positions[1];
    motor3.actuation_motor.position = target_positions[2];

    // Ejecutar el comando para los tres motores
    motor1.set_position();
    motor2.set_position();
    motor3.set_position();
}

// ==========================================
// GESTIÓN DE HARDWARE
// ==========================================

void RozumArm::set_velocity_limits_all() {
    motor1.set_velocity_limits();
    motor2.set_velocity_limits();
    motor3.set_velocity_limits();
}