#include "API_robot/motor_dinamixel.hpp"

// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================

DinamixelMotor::DinamixelMotor(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, uint8_t motor_id)
    : portHandler(port), packetHandler(packet), id(motor_id) // se guardan las variables dentro de las privadas
    {

    }

DinamixelMotor::~DinamixelMotor() {}


// ==========================================
// INICIALIZACIÓN Y ESTADO
// ==========================================

void DinamixelMotor::set_torque_state(bool state) {
    
    // Activar el torque y comprobar si funciona
    if (packetHandler->write1ByteTxRx(portHandler, id, READ_TORQUE_ADDRESS, state) != COMM_SUCCESS) {
        throw std::runtime_error("Error: No se pudo activar el torque del motor " + std::to_string(id));
    }
}

void DinamixelMotor::set_mode(uint8_t mode) {

    // Cambiar el modo de operación y comprobar si funciona
    if (packetHandler->write1ByteTxRx(portHandler, id, OPERATING_MODE, mode) != COMM_SUCCESS) {
        throw std::runtime_error("Error: No se pudo cambiar el modo del motor " + std::to_string(id));
    }
}

// ==========================================
// LECTURA DE VARIABLES (TELEMETRÍA)
// ==========================================

void DinamixelMotor::update_telemetry() {
    /*
    TODO: No se sabe si se debe hacer o no
    */
}


void DinamixelMotor::read_torque_state() { 
    // Crear variable almacenar resultado de lectura
    uint8_t torque_state_int;
    int res_torque = packetHandler->read1ByteTxRx(portHandler, id, READ_TORQUE_ADDRESS, &torque_state_int);
    
    // Validar la lectura y convertir a bool 
    if (res_torque != COMM_SUCCESS) {
        throw std::runtime_error("Error: No se pudo obtener el estado de torque del motor " + std::to_string(id));
    }
    telemetry_motor.torque_state = static_cast<bool>(torque_state_int);

}

void DinamixelMotor::add_ID_to_sync_read(dynamixel::GroupSyncRead* groupSyncRead) {
    // Verificamos por seguridad que el puntero no sea nulo
    if (groupSyncRead == nullptr) {
        throw std::invalid_argument("Error: El objeto GroupSyncRead no está inicializado.");
    }

    // Intentamos añadir el ID de este motor específico a la lista del paquete y comprobamos si la operación fue exitosa
    bool success = groupSyncRead->addParam(id);
    if (!success) {
        // Reemplazamos el fprintf y el exit(1) por una excepción estándar de C++
        throw std::runtime_error("Fail to read data (No se pudo añadir al grupo el motor ID: " + std::to_string(this->id) + ")");
    }
}


void DinamixelMotor::read_position(dynamixel::GroupSyncRead* groupSyncRead) { 
    // Según si está usando SyncRead o no, se lee con el método agrupado o individual
    if (groupSyncRead != nullptr) {
        
        // Verificamos si los datos de este motor en concreto llegaron en el paquete sincronizado
        if (groupSyncRead->isAvailable(id, READ_POSITION_ADDRESS, 4)) {
            telemetry_motor.position = groupSyncRead->getData(id, READ_POSITION_ADDRESS, 4);
        } else {
            throw std::runtime_error("Error: Datos de posición no disponibles en el paquete para el motor " + std::to_string(this->id));
        }
        
    } else {
        
        // Lectura individual directa usando la dirección 132 (READ_POSITION_ADDRESS)
        uint32_t raw_position = 0;
        int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, this->id, READ_POSITION_ADDRESS, &raw_position);
        
        if (dxl_comm_result != COMM_SUCCESS) {
            throw std::runtime_error("Error: No se pudo leer la posición del motor " + std::to_string(this->id));
        }
        telemetry_motor.position = static_cast<float>(raw_position);
    }
 }

void DinamixelMotor::read_velocity(dynamixel::GroupSyncRead* groupSyncRead) { 
    if (groupSyncRead != nullptr) {
        
        // Verificamos si los datos de este motor llegaron en el paquete sincronizado
        if (groupSyncRead->isAvailable(id, READ_VELOCITY_ADDRESS, 4)) {
            telemetry_motor.velocity = groupSyncRead->getData(id, READ_VELOCITY_ADDRESS, 4);
        } else {
            throw std::runtime_error("Error: Datos de velocidad no disponibles en el paquete para el motor " + std::to_string(this->id));
        }
        
    } else {
        
        // Lectura individual directa usando la dirección 128 (READ_VELOCITY_ADDRESS)
        uint32_t raw_velocity = 0;
        int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, this->id, READ_VELOCITY_ADDRESS, &raw_velocity);
        
        if (dxl_comm_result != COMM_SUCCESS) {
            throw std::runtime_error("Error: No se pudo leer la velocidad del motor " + std::to_string(this->id));
            telemetry_motor.velocity = static_cast<float>(raw_velocity);
        }
    }
 }

void DinamixelMotor::read_current(dynamixel::GroupSyncRead* groupSyncRead) {  
    if (groupSyncRead != nullptr) {
        
        if (groupSyncRead->isAvailable(id, READ_CURRENT_ADDRESS, 2)) {
            telemetry_motor.current = groupSyncRead->getData(id, READ_CURRENT_ADDRESS, 2);
        } else {
            throw std::runtime_error("Error: Datos de corriente no disponibles en el paquete para el motor " + std::to_string(this->id));
        }
        
    } else {
        
        // Lectura individual directa usando la dirección 126 (READ_CURRENT_ADDRESS)
        uint16_t raw_current = 0;
        int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, this->id, READ_CURRENT_ADDRESS, &raw_current);
        
        if (dxl_comm_result != COMM_SUCCESS) {
            throw std::runtime_error("Error: No se pudo leer la corriente del motor " + std::to_string(this->id));
        }
        telemetry_motor.current = static_cast<int16_t>(raw_current);
    }
}

void DinamixelMotor::read_temperature(dynamixel::GroupSyncRead* groupSyncRead) {  
    if (groupSyncRead != nullptr) {
        
        if (groupSyncRead->isAvailable(id, READ_TEMPERATURE_ADDRESS, 1)) {
            telemetry_motor.temperature = groupSyncRead->getData(id, READ_TEMPERATURE_ADDRESS, 1);
        } else {
            throw std::runtime_error("Error: Datos de temperatura no disponibles en el paquete para el motor " + std::to_string(this->id));
        }
        
    } else {
        
        // Lectura individual directa usando la dirección 146 (READ_TEMPERATURE_ADDRESS)
        uint8_t raw_temperature = 0;
        int dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, this->id, READ_TEMPERATURE_ADDRESS, &raw_temperature);
        
        if (dxl_comm_result != COMM_SUCCESS) {
            throw std::runtime_error("Error: No se pudo leer la temperatura del motor " + std::to_string(this->id));
        }
        telemetry_motor.temperature = static_cast<int8_t>(raw_temperature);
    }
}


// ==========================================
// CONTROL DE MOVIMIENTO
// ==========================================


void DinamixelMotor::set_velocity(dynamixel::GroupSyncWrite* groupSyncWrite) {
    // Crear array para dividir variable en pasos más pequeños
    uint8_t velocity_bytes[4];
    velocity_bytes[0] = DXL_LOBYTE(DXL_LOWORD(actuation_motor.velocity));
    velocity_bytes[1] = DXL_HIBYTE(DXL_LOWORD(actuation_motor.velocity));
    velocity_bytes[2] = DXL_LOBYTE(DXL_HIWORD(actuation_motor.velocity));
    velocity_bytes[3] = DXL_HIBYTE(DXL_HIWORD(actuation_motor.velocity));

    // Según si está usando BultWrite o no, se envía con el método BultWrite o con el método individual
    if (groupSyncWrite != nullptr) {
        bool result = groupSyncWrite->addParam(id, velocity_bytes);
        
        if (!result) {
            throw std::runtime_error("Error: No se pudo empaquetar la velocidad del motor " + std::to_string(id));
        }
    } else {
        int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, WRITE_VELOCITY_ADDRESS, actuation_motor.velocity);
        
        if (dxl_comm_result != COMM_SUCCESS) {
            throw std::runtime_error("Error: No se pudo enviar la velocidad al motor " + std::to_string(id));
        }
    }
  
}

void DinamixelMotor::set_position(dynamixel::GroupSyncWrite* groupSyncWrite) {
    // Crear array para dividir variable en pasos más pequeños
    uint8_t position_bytes[4];
    position_bytes[0] = DXL_LOBYTE(DXL_LOWORD(actuation_motor.position));
    position_bytes[1] = DXL_HIBYTE(DXL_LOWORD(actuation_motor.position));
    position_bytes[2] = DXL_LOBYTE(DXL_HIWORD(actuation_motor.position));
    position_bytes[3] = DXL_HIBYTE(DXL_HIWORD(actuation_motor.position));

    // Según si está usando BultWrite o no, se envía con el método BultWrite o con el método individual
    if (groupSyncWrite != nullptr) {
        bool result = groupSyncWrite->addParam(id, position_bytes);
        
        if (!result) {
            throw std::runtime_error("Error: No se pudo empaquetar la posición del motor " + std::to_string(id));
        }
    } else {
        int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, WRITE_POSITION_ADDRESS, actuation_motor.position);
        
        if (dxl_comm_result != COMM_SUCCESS) {
            throw std::runtime_error("Error: No se pudo enviar la posición al motor " + std::to_string(id));
        }
    }
}

// ==========================================
// GESTIÓN DE HARDWARE
// ==========================================

void DinamixelMotor::set_position_limits() { 

    // Utilizamos MAX_POSITION (está en int igual hay adaptarlo a 32 bits escalarlo a los ticks )
    int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, POSITION_LIMIT, MAX_POSITION);

    // Validar escritura
    if (dxl_comm_result != COMM_SUCCESS) { 
        throw std::runtime_error("Error: No se pudo establecer el límite de posición del motor " + std::to_string(this->id));
    }
    }

void DinamixelMotor::set_velocity_limits() { 
    int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, VELOCITY_LIMIT, MAX_VELOCITY);

    // Validar escritura
    if (dxl_comm_result != COMM_SUCCESS) {
        throw std::runtime_error("Error: No se pudo establecer el límite de velocidad del motor " + std::to_string(this->id));
    }
    }

void DinamixelMotor::get_id() { 
    /*
    TODO: 
    */
    return;
    }

void DinamixelMotor::set_id(uint8_t new_id) {
    /*
    TODO: 
    */
    new_id = new_id; // Poner valor para que salte el warning de variable no usada
    return;
}