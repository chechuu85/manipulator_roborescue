#include "API_robot/motor_rozum.hpp"

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
        rr_deinit_servo(motor);         // Libera los recursos del motor 
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

void RozumMotor::read_all_parameters(){
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

void RozumMotor::set_velocity(){
    if (motor == nullptr) return;

    // Detener el motor si se exceden los límites de posición
    if (actuation_motor.velocity > 0 && (actuation_motor.position > MAX_POSITION || actuation_motor.position < MIN_POSITION)){
        actuation_motor.velocity = 0; 
    }

    // Enviar el comando de velocidad 
    rr_servo_set_velocity(motor, actuation_motor.velocity);

}

void RozumMotor::set_position(){
    if (motor == nullptr) return;

    // Tener en cuenta los límites de posición
    actuation_motor.position = std::max(std::min(actuation_motor.position, (float)MAX_POSITION), (float)MIN_POSITION);
    
    rr_ret_status_t res_motor = rr_servo_set_position(motor, actuation_motor.position);
    if (res_motor != RR_SUCCESS){
        throw std::runtime_error("Error al establecer la posición del motor Rozum con ID: " + std::to_string(rr_servo_get_id(motor)));
    }
}


// ==========================================
// GESTIÓN DE HARDWARE
// ==========================================

void RozumMotor::set_velocity_limits(){
    if (motor == nullptr) return;

    rr_ret_status_t res_arm = rr_servo_set_velocity_limits(motor, MAX_VELOCITY);
    if (res_arm != RR_SUCCESS){
        throw std::runtime_error("Error al establecer los límites de velocidad del motor Rozum con ID: " + std::to_string(rr_servo_get_id(motor)));
    }

}

int RozumMotor::get_id(){
    /*
    Por programar
    */
}

void RozumMotor::set_id(int new_id){

    /*
    Por programar
    */
}

