#include "motors_com/rozum.hpp"

// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================

RozumMotor::RozumMotor(rr_can_interface_t* interface, int hardware_id){
    // Inicializa la interfaz CAN y el motor con el ID proporcionado
    iface = interface;                              
    motor = rr_servo_init(iface, hardware_id);

    // Comprobar que el motor se inicializa correctamente
    if (motor == nullptr){
        throw std::runtime_error("Error al inicializar el motor Rozum con ID: " + std::to_string(hardware_id));
    }

    // Inicializa la estructura de telemetría a valores por defecto
    telemetry_motor.position = 0.0f;
    telemetry_motor.velocity = 0.0f;
    telemetry_motor.current = 0;
    telemetry_motor.temperature = 0;

}

RozumMotor::~RozumMotor(){
    // Comprobar que el puntero no esté vacío
    if (motor != nullptr) {
        
        rr_servo_deactivate(motor);     // Desactiva la fuerza del motor antes de liberar recursos
        rr_servo_deinit(motor);         // Libera los recursos del motor 
        motor = nullptr;                // Evita el uso de punteros colgantes 
    }
}

// ==========================================
// INICIALIZACIÓN Y ESTADO
// ==========================================

 void RozumMotor::activate(){
    // Comprobar que el puntero del motor no sea nulo antes de intentar activarlo
    if (motor == nullptr) {
        throw std::runtime_error("Error: Intentando activar un motor nulo (no inicializado).");
    }

    // Activar el motor y comprobar que se activa correctamente
    if (rr_servo_activate(motor) != RR_SUCCESS) {
        throw std::runtime_error("Error al activar el motor Rozum con ID: " + std::to_string(rr_servo_get_id(motor)));
    }
 }

void RozumMotor::setup_telemetry_cache(){
    if (motor == nullptr) return; // Seguridad primero

    // Mantener actualizados los valores en memoria caché del motor para lectura rápida
    rr_param_cache_setup_entry(motor, APP_PARAM_POSITION_RO, true);
    rr_param_cache_setup_entry(motor, APP_PARAM_VELOCITY_RO, true);
    rr_param_cache_setup_entry(motor, APP_PARAM_CURRENT_INPUT_RO, true);
    rr_param_cache_setup_entry(motor, APP_PARAM_TEMPERATURE_RO, true);

}

void RozumMotor::update_cache(){
    if (motor == nullptr) return;

    // Actualiza la caché del ordenador con los datos del motor. Si falla, no se sobreescribe los valores 
    if (rr_param_cache_update(motor) != RR_SUCCESS) {
        return; 
    }
}

// ==========================================
// LECTURA DE VARIABLES (TELEMETRÍA)
// ==========================================

void RozumMotor::read_all(){
    update_cache(); // Asegura datos actualizados 

    // Leer los valores cacheados y almacenarlo en la estructura de telemetría 
    rr_read_cached_parameter(motor, APP_PARAM_POSITION_RO, &telemetry_motor.position);
    rr_read_cached_parameter(motor, APP_PARAM_VELOCITY_RO, &telemetry_motor.velocity);
    rr_read_cached_parameter(motor, APP_PARAM_CURRENT_INPUT_RO, &telemetry_motor.current);
    rr_read_cached_parameter(motor, APP_PARAM_TEMPERATURE_RO, &telemetry_motor.temperature);
}

void RozumMotor::read_position() {
    update_cache();
    rr_read_cached_parameter(motor, APP_PARAM_POSITION_RO, &telemetry_motor.position);
}

void RozumMotor::read_velocity() {
    update_cache();
    rr_read_cached_parameter(motor, APP_PARAM_VELOCITY_RO, &telemetry_motor.velocity);
}

void RozumMotor::read_current() {
    update_cache();
    rr_read_cached_parameter(motor, APP_PARAM_CURRENT_INPUT_RO, &telemetry_motor.current);
}

void RozumMotor::read_temperature() {
    update_cache();
    rr_read_cached_parameter(motor, APP_PARAM_TEMPERATURE_RO, &telemetry_motor.temperature);
}

// ==========================================
// CONTROL DE MOVIMIENTO
// ==========================================

// void RozumMotor::set_velocity(float target_vel, float max_pos_limit) ...
// void RozumMotor::set_position(float target_pos, float max_pos_limit) ...

// ==========================================
// GESTIÓN DE HARDWARE
// ==========================================

// int RozumMotor::get_id() ...
// void RozumMotor::set_id(int new_id) ...

