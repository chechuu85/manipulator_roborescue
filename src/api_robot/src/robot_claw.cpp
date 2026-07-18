#include "API_robot/robot_claw.hpp"

// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================

DynamixelClaw::DynamixelClaw(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, 
                             uint8_t id1, uint8_t id2, uint8_t id3, uint8_t id5, uint8_t id12)
    : portHandler(port), packetHandler(packet),
      motor1(port, packet, id1),
      motor2(port, packet, id2),
      motor3(port, packet, id3),
      motor5(port, packet, id5),
      motor12(port, packet, id12)
{
    if (portHandler == nullptr || packetHandler == nullptr) {
        throw std::runtime_error("Error: Handlers de Dynamixel nulos en la inicialización de DynamixelClaw.");
    }
}

DynamixelClaw::~DynamixelClaw() {
    // Apagamos el torque por seguridad al destruir el objeto
    try {
        set_torque_all(false); 
    } catch (...) {
        // Suprimir excepciones en el destructor
    }
}

// ==========================================
// INICIALIZACIÓN Y ESTADO
// ==========================================

void DynamixelClaw::set_torque_all(bool state) {
    motor1.set_torque_state(state);
    motor2.set_torque_state(state);
    motor3.set_torque_state(state);
    motor5.set_torque_state(state);
    motor12.set_torque_state(state);
}

void DynamixelClaw::set_mode_all(uint8_t mode) {
    motor1.set_mode(mode);
    motor2.set_mode(mode);
    motor3.set_mode(mode);
    motor5.set_mode(mode);
    motor12.set_mode(mode);
}

// ==========================================
// LECTURA DE VARIABLES SÍNCRONA
// ==========================================

void DynamixelClaw::read_positions() {
    // 132 es READ_POSITION_ADDRESS y requiere 4 bytes[cite: 6]
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, READ_POSITION_ADDRESS, 4);
    
    // Añadimos los motores al paquete de lectura
    motor1.add_ID_to_sync_read(&groupSyncRead);
    motor2.add_ID_to_sync_read(&groupSyncRead);
    motor3.add_ID_to_sync_read(&groupSyncRead);
    motor5.add_ID_to_sync_read(&groupSyncRead);
    motor12.add_ID_to_sync_read(&groupSyncRead);

    if (groupSyncRead.txRxPacket() != COMM_SUCCESS) {
        throw std::runtime_error("Error en comunicación txRxPacket al leer posiciones.");
    }

    // Extraemos la información de la respuesta
    motor1.read_position(&groupSyncRead);
    motor2.read_position(&groupSyncRead);
    motor3.read_position(&groupSyncRead);
    motor5.read_position(&groupSyncRead);
    motor12.read_position(&groupSyncRead);
}

void DynamixelClaw::read_velocities() {
    // 128 es READ_VELOCITY_ADDRESS y requiere 4 bytes[cite: 6]
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, READ_VELOCITY_ADDRESS, 4);
    
    motor1.add_ID_to_sync_read(&groupSyncRead);
    motor2.add_ID_to_sync_read(&groupSyncRead);
    motor3.add_ID_to_sync_read(&groupSyncRead);
    motor5.add_ID_to_sync_read(&groupSyncRead);
    motor12.add_ID_to_sync_read(&groupSyncRead);

    if (groupSyncRead.txRxPacket() != COMM_SUCCESS) {
        throw std::runtime_error("Error en comunicación txRxPacket al leer velocidades.");
    }

    motor1.read_velocity(&groupSyncRead);
    motor2.read_velocity(&groupSyncRead);
    motor3.read_velocity(&groupSyncRead);
    motor5.read_velocity(&groupSyncRead);
    motor12.read_velocity(&groupSyncRead);
}

void DynamixelClaw::read_currents() {
    // 126 es READ_CURRENT_ADDRESS y requiere 2 bytes[cite: 6]
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, READ_CURRENT_ADDRESS, 2);
    
    motor1.add_ID_to_sync_read(&groupSyncRead);
    motor2.add_ID_to_sync_read(&groupSyncRead);
    motor3.add_ID_to_sync_read(&groupSyncRead);
    motor5.add_ID_to_sync_read(&groupSyncRead);
    motor12.add_ID_to_sync_read(&groupSyncRead);

    if (groupSyncRead.txRxPacket() != COMM_SUCCESS) {
        throw std::runtime_error("Error en comunicación txRxPacket al leer corrientes.");
    }

    motor1.read_current(&groupSyncRead);
    motor2.read_current(&groupSyncRead);
    motor3.read_current(&groupSyncRead);
    motor5.read_current(&groupSyncRead);
    motor12.read_current(&groupSyncRead);
}

void DynamixelClaw::read_temperatures() {
    // 146 es READ_TEMPERATURE_ADDRESS y requiere 1 byte[cite: 6]
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, READ_TEMPERATURE_ADDRESS, 1);
    
    motor1.add_ID_to_sync_read(&groupSyncRead);
    motor2.add_ID_to_sync_read(&groupSyncRead);
    motor3.add_ID_to_sync_read(&groupSyncRead);
    motor5.add_ID_to_sync_read(&groupSyncRead);
    motor12.add_ID_to_sync_read(&groupSyncRead);

    if (groupSyncRead.txRxPacket() != COMM_SUCCESS) {
        throw std::runtime_error("Error en comunicación txRxPacket al leer temperaturas.");
    }

    motor1.read_temperature(&groupSyncRead);
    motor2.read_temperature(&groupSyncRead);
    motor3.read_temperature(&groupSyncRead);
    motor5.read_temperature(&groupSyncRead);
    motor12.read_temperature(&groupSyncRead);
}

// ==========================================
// CONTROL DE MOVIMIENTO SÍNCRONO
// ==========================================

void DynamixelClaw::set_velocities(const std::array<float, 5>& target_velocities) {
    // Asignamos la velocidad interna requerida antes del empaquetado
    motor1.actuation_motor.velocity = target_velocities[0];
    motor2.actuation_motor.velocity = target_velocities[1];
    motor3.actuation_motor.velocity = target_velocities[2];
    motor5.actuation_motor.velocity = target_velocities[3];
    motor12.actuation_motor.velocity = target_velocities[4];

    // 104 es WRITE_VELOCITY_ADDRESS (4 bytes)[cite: 6]
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, WRITE_VELOCITY_ADDRESS, 4);

    // Empaquetamos los datos
    motor1.set_velocity(&groupSyncWrite);
    motor2.set_velocity(&groupSyncWrite);
    motor3.set_velocity(&groupSyncWrite);
    motor5.set_velocity(&groupSyncWrite);
    motor12.set_velocity(&groupSyncWrite);

    // Transmitimos a todos los motores en un solo paquete
    if (groupSyncWrite.txPacket() != COMM_SUCCESS) {
        throw std::runtime_error("Error al enviar el paquete GroupSyncWrite de velocidades.");
    }
}

void DynamixelClaw::set_positions(const std::array<float, 5>& target_positions) {
    // Asignamos la posición interna requerida antes del empaquetado
    motor1.actuation_motor.position = target_positions[0];
    motor2.actuation_motor.position = target_positions[1];
    motor3.actuation_motor.position = target_positions[2];
    motor5.actuation_motor.position = target_positions[3];
    motor12.actuation_motor.position = target_positions[4];

    // 116 es WRITE_POSITION_ADDRESS (4 bytes)[cite: 6]
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, WRITE_POSITION_ADDRESS, 4);

    // Empaquetamos los datos
    motor1.set_position(&groupSyncWrite);
    motor2.set_position(&groupSyncWrite);
    motor3.set_position(&groupSyncWrite);
    motor5.set_position(&groupSyncWrite);
    motor12.set_position(&groupSyncWrite);

    // Transmitimos a todos los motores simultáneamente
    if (groupSyncWrite.txPacket() != COMM_SUCCESS) {
        throw std::runtime_error("Error al enviar el paquete GroupSyncWrite de posiciones.");
    }
}

// ==========================================
// GESTIÓN DE HARDWARE
// ==========================================

void DynamixelClaw::set_limits_all() {
    // Límites de Posición
    motor1.set_position_limits();
    motor2.set_position_limits();
    motor3.set_position_limits();
    motor5.set_position_limits();
    motor12.set_position_limits();

    // Límites de Velocidad
    motor1.set_velocity_limits();
    motor2.set_velocity_limits();
    motor3.set_velocity_limits();
    motor5.set_velocity_limits();
    motor12.set_velocity_limits();
}