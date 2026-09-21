#include "API_robot/motor_rozum.hpp"

// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================

RozumMotor::RozumMotor(rr_can_interface_t* interface, int hardware_id, float max_pos_input, float min_pos_input){
    // Inicializa la interfaz CAN y el motor con el ID proporcionado
    iface = interface;                              
    motor = rr_init_servo(iface, hardware_id);
    id = hardware_id;
    MAX_POSITION = max_pos_input / CONV_DEG2RAD;
    MIN_POSITION = min_pos_input / CONV_DEG2RAD;

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
    if (motor == nullptr) {
        throw std::runtime_error("Error: Puntero del motor nulo al intentar actualizar caché.");
    }

    // Actualiza la caché del ordenador con los datos del motor. Si falla, no se sobreescribe los valores 
    if (rr_param_cache_update(motor) != RET_OK) {
        throw std::runtime_error("Error de comunicación: Fallo al actualizar la caché del motor Rozum.");
    }
}

// ==========================================
// LECTURA DE VARIABLES (TELEMETRÍA)
// ==========================================

void RozumMotor::read_all_parameters(){
    // Antes de leer los parámetros, usar update_cache() para asegurarse de que los datos estén actualizados

    float temp_position = 0.0f;
    float temp_velocity = 0.0f;
    float temp_current = 0.0f;
    float temp_temperature = 0.0f;

    // Leer los valores cacheados y almacenarlo en la estructura de telemetría y verificar que funciona
    if (rr_read_cached_parameter(motor, APP_PARAM_POSITION, &temp_position) != RET_OK ||
        rr_read_cached_parameter(motor, APP_PARAM_VELOCITY, &temp_velocity) != RET_OK ||
        rr_read_cached_parameter(motor, APP_PARAM_CURRENT_INPUT, &temp_current) != RET_OK ||
        rr_read_cached_parameter(motor, APP_PARAM_TEMPERATURE_ACTUATOR, &temp_temperature) != RET_OK) {
        
        throw std::runtime_error("Error: No se pudieron extraer los parámetros de la caché del motor Rozum.");
    }
    // Puede ser que no entregue los datos correctamente
    telemetry_motor.position = temp_position * CONV_DEG2RAD;
    telemetry_motor.velocity = temp_velocity * CONV_DEG2RAD;
    telemetry_motor.current = static_cast<int16_t>(temp_current);
    telemetry_motor.temperature = static_cast<int8_t>(temp_temperature);
}

void RozumMotor::read_position() {
    // Antes de leer los parámetros, usar update_cache() para asegurarse de que los datos estén actualizados
    float temp_position = 0.0f;
    if (rr_read_cached_parameter(motor, APP_PARAM_POSITION, &temp_position) != RET_OK) {
        throw std::runtime_error("Error: Fallo al leer la posición de la caché.");
    }
    telemetry_motor.position = temp_position * CONV_DEG2RAD;
}

void RozumMotor::read_velocity() {
    // Antes de leer los parámetros, usar update_cache() para asegurarse de que los datos estén actualizados
    float temp_velocity = 0.0f;
    if (rr_read_cached_parameter(motor, APP_PARAM_VELOCITY, &temp_velocity) != RET_OK) {
        throw std::runtime_error("Error: Fallo al leer la velocidad de la caché.");
    }
    telemetry_motor.velocity = temp_velocity * CONV_DEG2RAD;
}

void RozumMotor::read_current() {
    // Antes de leer los parámetros, usar update_cache() para asegurarse de que los datos estén actualizados
    float temp_current = 0.0f;
    if (rr_read_cached_parameter(motor, APP_PARAM_CURRENT_INPUT, &temp_current) != RET_OK) {
        throw std::runtime_error("Error: Fallo al leer la corriente de la caché.");
    }
    telemetry_motor.current = static_cast<int16_t>(temp_current);
}

void RozumMotor::read_temperature() {
    // Antes de leer los parámetros, usar update_cache() para asegurarse de que los datos estén actualizados
    float temp_temperature = 0.0f;
    if (rr_read_cached_parameter(motor, APP_PARAM_TEMPERATURE_ACTUATOR, &temp_temperature) != RET_OK) {
        throw std::runtime_error("Error: Fallo al leer la temperatura de la caché.");
    }
    telemetry_motor.temperature = static_cast<int8_t>(temp_temperature);
}

// ==========================================
// CONTROL DE MOVIMIENTO
// ==========================================

void RozumMotor::set_velocity(){
    if (motor == nullptr) {
        throw std::runtime_error("Error: Puntero nulo al intentar enviar velocidad al motor Rozum con ID: " + std::to_string(id));
    }
    
    // Enviar el comando de velocidad y comprobar si falla 
    float target_vel_deg = actuation_motor.velocity / CONV_DEG2RAD;
    float current_pos_deg = telemetry_motor.position / CONV_DEG2RAD;

    if ((current_pos_deg >= (float)MAX_POSITION && target_vel_deg > 0.0f) || (current_pos_deg <= (float)MIN_POSITION && target_vel_deg < 0.0f)){
        target_vel_deg = 0.0f;

        if (!limit_warning_shown) {
            std::cerr << "[SEGURIDAD] Motor " << std::to_string(id) << " en límite MÁXIMO. Vel bloqueada." << std::endl;
            limit_warning_shown = true;
            
        }
    } else {
        limit_warning_shown = false; // Se resetea cuando sale del límite
    }

    rr_ret_status_t res_motor = rr_set_velocity(motor, target_vel_deg);
    if (res_motor != RET_OK){
        throw std::runtime_error("Error al establecer la velocidad del motor Rozum con ID: " + std::to_string(id));
    }

}

void RozumMotor::set_position(){
    if (motor == nullptr) {
        throw std::runtime_error("Error: Puntero nulo al intentar enviar posición al motor Rozum con ID: " + std::to_string(id));
    }
    
    // Enviar el comando de posición y comprobar si falla 
    float target_pos_deg = actuation_motor.position / CONV_DEG2RAD;
    if (target_pos_deg > (float)MAX_POSITION) {
        std::cerr << "[AVISO] Motor " << std::to_string(id) << " superó el límite MÁXIMO (" << MAX_POSITION << "°). Recortando trayectoria..." << std::endl;
    } else if (target_pos_deg < (float)MIN_POSITION) {
        std::cerr << "[AVISO] Motor " << std::to_string(id) << " superó el límite MÍNIMO (" << MIN_POSITION << "°). Recortando trayectoria..." << std::endl;
    }
    target_pos_deg = std::max(std::min(target_pos_deg, (float)MAX_POSITION), (float)MIN_POSITION);
    rr_ret_status_t res_motor = rr_set_position(motor, target_pos_deg);
    if (res_motor != RET_OK){
        throw std::runtime_error("Error al establecer la posición del motor Rozum con ID: " + std::to_string(id));
    }
}


// ==========================================
// GESTIÓN DE HARDWARE
// ==========================================

void RozumMotor::set_velocity_limits(){
    if (motor == nullptr) {
        throw std::runtime_error("Error: Puntero nulo al intentar enviar límites de velocidad al motor Rozum con ID: " + std::to_string(id));
    }

    rr_ret_status_t res_arm = rr_set_max_velocity(motor, MAX_VELOCITY/CONV_DEG2RAD);
    if (res_arm != RET_OK){
        throw std::runtime_error("Error al establecer los límites de velocidad del motor Rozum con ID: " + std::to_string(id));
    }
}

// No hay función de set_max_position

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

