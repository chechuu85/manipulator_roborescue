#include "manual_user_interface/adapterToSimulation.hpp"

// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================
AdapterToSimulationNode::AdapterToSimulationNode() : Node("adapter_to_simulation_node") {

    // Obtener el valor del parámetro por el launch
    this->declare_parameter<int>("timer_period_ms", 20);
    this->get_parameter("timer_period_ms", timer_period_ms);

    // Suscriptor al teclado o estado de motores
    sub_articular_ = this->create_subscription<manipulator_msgs::msg::HiperJointState>(
        "/kdl_articular", 10, std::bind(&AdapterToSimulationNode::callback, this, std::placeholders::_1));

    // Publicador estándar que exige ROS2 / Foxglove para animar el URDF
    pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms), 
             std::bind(&AdapterToSimulationNode::timer_callback, this));
    
    // Define los nombres exactos de los joints declarados en tu manipulador.urdf.xacro. 
    // No debería estar hardcodeado. ***CAMBIAR*** 
    joint_state_msg_.header.frame_id = "base_link";
    joint_state_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};

    joint_state_msg_.velocity.resize(8);
    joint_state_msg_.position.resize(8);

   
    // FIX: mover robot de posición inicial para evitar singularidades 
    joint_state_msg_.position[1] = posiciones_actuales_[1] = 0.2;
    joint_state_msg_.position[2] = posiciones_actuales_[2] = 0.2;
    joint_state_msg_.position[4] = posiciones_actuales_[4] = 0.2;

    ultimo_tiempo_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Nodo de adaptación a simulación inicializado. ");
}

AdapterToSimulationNode::~AdapterToSimulationNode(){
    RCLCPP_INFO(this->get_logger(), "Nodo de adaptación a simulación finalizado. ");
}

// ==========================================
// INTERRUPCIONES
// ==========================================
// Recolecta los datos 
void AdapterToSimulationNode::callback(const manipulator_msgs::msg::HiperJointState::SharedPtr msg) {
    // Obtenemos el tamaño real del mensaje
    size_t pos_size = msg->joint_state_command.position.size();
    size_t vel_size = msg->joint_state_command.velocity.size();

    // En modo trayectoria no se mueven los motores de las garras
    if (pos_size < 6) {
        RCLCPP_WARN(this->get_logger(), "Datos insuficientes: %zu. Se requieren al menos 6.", pos_size);
        return;
    }

    if (msg->command_info == "FIRST" || msg->command_info == "RECORDING") {
        
        if (msg->command_info == "FIRST") {
            RCLCPP_INFO(this->get_logger(), "[START] Trayectoria iniciada. Comandos manuales bloqueados.");
        }
        
        // Bloqueamos modo manual y actualizamos la pose objetivo al instante
        current_mode_ = RobotMode::EXECUTING;
        
        for (size_t i = 0; i < 8; i++) {
            // Si el índice existe en el mensaje, lo copiamos. Si no, forzamos 0.0
            posiciones_actuales_[i] = (i < pos_size) ? msg->joint_state_command.position[i] : 0.0;
            velocidades_actuales_[i] = (i < vel_size) ? msg->joint_state_command.velocity[i] : 0.0;
        }

    } else if (msg->command_info == "LAST") {
        
        RCLCPP_INFO(this->get_logger(), "[END] Último punto recibido. Control manual DESBLOQUEADO.");
        current_mode_ = RobotMode::MANUAL;
        
        for (size_t i = 0; i < 8; i++) {
            posiciones_actuales_[i] = (i < pos_size) ? msg->joint_state_command.position[i] : 0.0;
            velocidades_actuales_[i] = (i < vel_size) ? msg->joint_state_command.velocity[i] : 0.0;
        }

    } else if (msg->command_info == "MANUAL" && current_mode_ == RobotMode::MANUAL) {
        // En modo manual se controlan todos los motores
        if (msg->joint_state_command.velocity.size() >= 8) {
            for (size_t i = 0; i < 8; i++) {
                velocidades_actuales_[i] = msg->joint_state_command.velocity[i];
            }
        }
    }
}

// Envia los datos 
void AdapterToSimulationNode::timer_callback() {
    auto ahora = this->now();
    joint_state_msg_.header.stamp = ahora;

    double dt = (ahora - ultimo_tiempo_).seconds();
    ultimo_tiempo_ = ahora; 

    if (current_mode_ == RobotMode::EXECUTING) {
        // Modo Streaming: Pasamos directamente la última posición recibida de la trayectoria
        for (size_t i = 0; i < 8; i++) {
            joint_state_msg_.position[i] = posiciones_actuales_[i];
            joint_state_msg_.velocity[i] = velocidades_actuales_[i];
        }
    } 
    else if (current_mode_ == RobotMode::MANUAL) {
        // Modo Integración: Calculamos posición en base a la velocidad del mando
        for (size_t i = 0; i < 8; i++) {
            joint_state_msg_.velocity[i] = velocidades_actuales_[i];
            posiciones_actuales_[i] += velocidades_actuales_[i] * dt;
            joint_state_msg_.position[i] = posiciones_actuales_[i];
        }
    }

    pub_joint_states_->publish(joint_state_msg_);
}


// ==========================================
// FUNCIONES AUXILIARES
// ==========================================
trajectory_msgs::msg::JointTrajectoryPoint AdapterToSimulationNode::create_trajectory_point(
    const manipulator_msgs::msg::HiperJointState::SharedPtr msg) 
{
    trajectory_msgs::msg::JointTrajectoryPoint point;
    
    // Redimensionamos los arrays basándonos en los 8 joints del URDF
    point.positions.resize(8);
    point.velocities.resize(8);
    
    for (size_t i = 0; i < 8; i++) {
        point.positions[i] = msg->joint_state_command.position[i]; 
        point.velocities[i] = msg->joint_state_command.velocity[i];
    }
    
    return point;
}

// ==========================================
// MAIN
// ==========================================
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AdapterToSimulationNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}