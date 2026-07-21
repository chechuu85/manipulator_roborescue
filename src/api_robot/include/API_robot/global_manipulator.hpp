#ifndef GLOBAL_MANIPULATOR_HPP
#define GLOBAL_MANIPULATOR_HPP

#include "rclcpp/rclcpp.hpp" 
#include "manipulator_msgs/msg/manipulator_motor_stage.hpp"
#include "API_robot/robot_arm.hpp"
#include "API_robot/robot_claw.hpp"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <array>
#include <atomic>
#include <chrono>

// Enum basado en la estructura de tu documento
enum class tarea_dynamixel {
    SLEEPING,
    SEND_VELOCITY,
    SEND_POSITION,
    READ_TEMPERATURE,
    READ_VELOCITY,
    READ_POSITION,
    READ_CURRENT,
    POWER_OFF
};

class GlobalManipulator : public rclcpp::Node {
private:
    // --- Componentes del Manipulador ---
    RozumArm* arm;
    DynamixelClaw* claw;

    // --- Sincronización y Hilos ---
    std::thread hilo_dynamixel;
    std::mutex mtx_sincronizacion;
    std::condition_variable cv_iniciar_tarea_;
    std::condition_variable cv_tarea_terminada_;
    
    tarea_dynamixel comando_actual_ = tarea_dynamixel::SLEEPING;
    bool tarea_completada_ = false;

    rclcpp::Publisher<manipulator_msgs::msg::ManipulatorMotorStage>::SharedPtr telemetry_pub_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;

    // Bucle infinito del hilo secundario
    void gestor_tareas();

    // Callback para leer y publicar el estado de los motores
    void publish_telemetry_callback();

public:
    // --- Constructor y Destructor ---
    // Recibe los manejadores de los puertos ya inicializados para inyectarlos en las subclases
    GlobalManipulator(rr_can_interface_t* rozum_iface, dynamixel::PortHandler* port, dynamixel::PacketHandler* packet);
    ~GlobalManipulator();

    // --- Inicialización ---
    void init();
    void deinit();

    // --- Lectura de Variables Síncrona ---
    // Ejecutan la lectura del brazo en el hilo principal y la garra en el secundario
    void read_positions();
    void read_velocities();
    void read_currents();
    void read_temperatures();

    // --- Control de Movimiento ---
    // Reciben los arrays de objetivos para brazo (3) y garra (5)
    void set_velocities(const std::array<float, 3>& arm_vels, const std::array<float, 5>& claw_vels);
    void set_positions(const std::array<float, 3>& arm_pos, const std::array<float, 5>& claw_pos);
};

#endif // GLOBAL_MANIPULATOR_HPP