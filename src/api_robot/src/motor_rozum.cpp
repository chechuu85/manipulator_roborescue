#include "API_robot/motor_rozum.hpp"

// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================

RozumMotor::RozumMotor(rr_can_interface_t* interface, int hardware_id){
    // Inicializa la interfaz CAN y el motor con el ID proporcionado
    iface = interface;                              
    motor = rr_init_servo(iface, hardware_id);
    id = hardware_id;

    // Comprobar que el motor se inicializa correctamente
    if (motor == nullptr){
        throw std::runtime_error("Error al inicializar el motor Rozum con ID: " + std::to_string(id));
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
        
        rr_servo_set_state_pre_operational(motor);      // Desactiva la fuerza del motor antes de liberar recursos
        rr_deinit_servo(&motor);                        // Libera los recursos del motor 
        motor = nullptr;                                // Evita el uso de punteros colgantes 
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
    if (rr_servo_set_state_operational(motor) != RET_OK) {
        throw std::runtime_error("Error al activar el motor Rozum con ID: " + std::to_string(id));
    }
 }

void RozumMotor::setup_telemetry_cache(){
    if (motor == nullptr) return; // Seguridad primero

    // Mantener actualizados los valores en memoria caché del motor para lectura rápida
    rr_param_cache_setup_entry(motor, APP_PARAM_POSITION, true);    
    rr_param_cache_setup_entry(motor, APP_PARAM_VELOCITY, true);
    rr_param_cache_setup_entry(motor, APP_PARAM_CURRENT_INPUT, true);
    rr_param_cache_setup_entry(motor, APP_PARAM_TEMPERATURE_ACTUATOR, true);

    // Tambien existe la dirección APP_PARAM_POSITION_ROTOR..., que es la posición del rotor
}

void RozumMotor::update_cache(){
    if (motor == nullptr) return;

    // Actualiza la caché del ordenador con los datos del motor. Si falla, no se sobreescribe los valores 
    if (rr_param_cache_update(motor) != RET_OK) {
        return; 
    }
}

// ==========================================
// LECTURA DE VARIABLES (TELEMETRÍA)
// ==========================================

void RozumMotor::read_all_parameters(){
    // Antes de leer los parámetros, usar update_cache() para asegurarse de que los datos estén actualizados

    float temp_current = 0.0f;
    float temp_temperature = 0.0f;

    // Leer los valores cacheados y almacenarlo en la estructura de telemetría 
    rr_read_cached_parameter(motor, APP_PARAM_POSITION, &telemetry_motor.position);
    rr_read_cached_parameter(motor, APP_PARAM_VELOCITY, &telemetry_motor.velocity);
    rr_read_cached_parameter(motor, APP_PARAM_CURRENT_INPUT, &temp_current);
    rr_read_cached_parameter(motor, APP_PARAM_TEMPERATURE_ACTUATOR, &temp_temperature);
    // Puede ser que no entregue los datos correctamente
    telemetry_motor.current = static_cast<int16_t>(temp_current);
    telemetry_motor.temperature = static_cast<int8_t>(temp_temperature);
}

void RozumMotor::read_position() {
    // Antes de leer los parámetros, usar update_cache() para asegurarse de que los datos estén actualizados
    rr_read_cached_parameter(motor, APP_PARAM_POSITION, &telemetry_motor.position);
}

void RozumMotor::read_velocity() {
    // Antes de leer los parámetros, usar update_cache() para asegurarse de que los datos estén actualizados
    rr_read_cached_parameter(motor, APP_PARAM_VELOCITY, &telemetry_motor.velocity);
}

void RozumMotor::read_current() {
    // Antes de leer los parámetros, usar update_cache() para asegurarse de que los datos estén actualizados
    float temp_current = 0.0f;
    rr_read_cached_parameter(motor, APP_PARAM_CURRENT_INPUT, &temp_current);
    telemetry_motor.current = static_cast<int16_t>(temp_current);
}

void RozumMotor::read_temperature() {
    // Antes de leer los parámetros, usar update_cache() para asegurarse de que los datos estén actualizados
    float temp_temperature = 0.0f;
    rr_read_cached_parameter(motor, APP_PARAM_TEMPERATURE_ACTUATOR, &temp_temperature);
    telemetry_motor.temperature = static_cast<int8_t>(temp_temperature);
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

    // Enviar el comando de velocidad y comprobar si falla 
    rr_ret_status_t res_motor = rr_set_velocity(motor, actuation_motor.velocity);
    if (res_motor != RET_OK){
        throw std::runtime_error("Error al establecer la velocidad del motor Rozum con ID: " + std::to_string(id));
    }

}

void RozumMotor::set_position(){
    if (motor == nullptr) return;

    // Tener en cuenta los límites de posición
    actuation_motor.position = std::max(std::min(actuation_motor.position, (float)MAX_POSITION), (float)MIN_POSITION);
    
    // Enviar el comando de posición y comprobar si falla 
    rr_ret_status_t res_motor = rr_set_position(motor, actuation_motor.position);
    if (res_motor != RET_OK){
        throw std::runtime_error("Error al establecer la posición del motor Rozum con ID: " + std::to_string(id));
    }
}


// ==========================================
// GESTIÓN DE HARDWARE
// ==========================================

void RozumMotor::set_velocity_limits(){
    if (motor == nullptr) return;

    rr_ret_status_t res_arm = rr_set_max_velocity(motor, MAX_VELOCITY);
    if (res_arm != RET_OK){
        throw std::runtime_error("Error al establecer los límites de velocidad del motor Rozum con ID: " + std::to_string(id));
    }

}

int RozumMotor::get_id(){
    /*
    Por programar
    */
    return 0;
}

void RozumMotor::set_id(int new_id){

    /*
    Por programar
    */
    new_id = new_id; // Poner valor para que no salte el warning de variable no usada
    return;
}

