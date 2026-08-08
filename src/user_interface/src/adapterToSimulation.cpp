#include "user_interface/adapterToSimulation.hpp"

// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================
AdapterToSimulationNode::AdapterToSimulationNode() : Node("adapter_to_simulation_node") {
        // Suscriptor al teclado o estado de motores
        sub_articular_ = this->create_subscription<manipulator_msgs::msg::ManipulatorMotorStage>(
            "keyboard_articular", 10, std::bind(&AdapterToSimulationNode::callback, this, std::placeholders::_1));
            // se cambiará posteriormente 

        // Publicador estándar que exige ROS2 / Foxglove para animar el URDF
        pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
        
        ultimo_tiempo_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Nodo de adaptación a simulación inicializado. ");
    }

AdapterToSimulationNode::~AdapterToSimulationNode(){
    RCLCPP_INFO(this->get_logger(), "Nodo de adaptación a simulación finalizado. ");
}

// ==========================================
// INTERRUPCIÓN
// ==========================================
void AdapterToSimulationNode::callback(const manipulator_msgs::msg::ManipulatorMotorStage::SharedPtr msg) {
    auto joint_state = sensor_msgs::msg::JointState();
        
    joint_state.header.frame_id = "base_link";
    auto ahora = this->now();
    joint_state.header.stamp = ahora;

    // Calcular el diferencial de tiempo real
    double dt = (ahora - ultimo_tiempo_).seconds();
    ultimo_tiempo_ = ahora; // Actualizar para el siguiente ciclo
    
    // Define los nombres exactos de los joints declarados en tu manipulador.urdf.xacro
    joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};
    
    // Mapea tus arrays personalizados de Rozum y Dinamixel al array plano estándar
    joint_state.velocity.resize(8);
    joint_state.position.resize(8);

    joint_state.velocity[0] = msg->rozum_motors[0].velocity;
    joint_state.velocity[1] = msg->rozum_motors[1].velocity;
    joint_state.velocity[2] = msg->rozum_motors[2].velocity;
    
    joint_state.velocity[3] = msg->dinamixel_motors[0].velocity;
    joint_state.velocity[4] = msg->dinamixel_motors[1].velocity;
    joint_state.velocity[5] = msg->dinamixel_motors[2].velocity;
    joint_state.velocity[6] = msg->dinamixel_motors[3].velocity;
    joint_state.velocity[7] = msg->dinamixel_motors[4].velocity;

    // Calcular la posición 
    for (size_t i = 0; i < 8; i++) {
        // Posición actual = Posición anterior + (Velocidad * Variación de tiempo)
        posiciones_actuales_[i] += joint_state.velocity[i] * dt;
        
        // Ahora sí, pasamos el dato calculado al mensaje ROS2
        joint_state.position[i] = posiciones_actuales_[i];
    }

    // Publicamos el estado estándar para que el robot_state_publisher lo procese
    pub_joint_states_->publish(joint_state);
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