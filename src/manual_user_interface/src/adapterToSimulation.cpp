#include "manual_user_interface/adapterToSimulation.hpp"

// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================
AdapterToSimulationNode::AdapterToSimulationNode() : Node("adapter_to_simulation_node") {

    // Obtener el valor del parámetro por el launch
    this->declare_parameter<int>("timer_period_ms", 20);
    this->get_parameter("timer_period_ms", timer_period_ms);

    // Suscriptor al teclado o estado de motores
    sub_articular_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/kdl_articular", 10, std::bind(&AdapterToSimulationNode::callback, this, std::placeholders::_1));

    // Publicador estándar que exige ROS2 / Foxglove para animar el URDF
    pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms), 
             std::bind(&AdapterToSimulationNode::timer_callback, this));
    
    // Define los nombres exactos de los joints declarados en tu manipulador.urdf.xacro
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
void AdapterToSimulationNode::callback(const sensor_msgs::msg::JointState::SharedPtr msg) {

    velocidades_actuales_[0] = msg->velocity[0];
    velocidades_actuales_[1] = msg->velocity[1];
    velocidades_actuales_[2] = msg->velocity[2];
    
    velocidades_actuales_[3] = msg->velocity[3];
    velocidades_actuales_[4] = msg->velocity[4];
    velocidades_actuales_[5] = msg->velocity[5];
    velocidades_actuales_[6] = msg->velocity[6];
    velocidades_actuales_[7] = msg->velocity[7];
}

// Envia los datos 
void AdapterToSimulationNode::timer_callback() {
    
    auto ahora = this->now();
    joint_state_msg_.header.stamp = ahora;

    // Calcular el diferencial de tiempo real
    double dt = (ahora - ultimo_tiempo_).seconds();
    ultimo_tiempo_ = ahora; 

    // Calcular la posición 
    for (size_t i = 0; i < 8; i++) {
        joint_state_msg_.velocity[i] = velocidades_actuales_[i];

        // Posición actual = Posición anterior + (Velocidad * Variación de tiempo)
        posiciones_actuales_[i] += joint_state_msg_.velocity[i] * dt;
        joint_state_msg_.position[i] = posiciones_actuales_[i];
    }

    // Publicamos el estado estándar para que el robot_state_publisher lo procese
    pub_joint_states_->publish(joint_state_msg_);
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