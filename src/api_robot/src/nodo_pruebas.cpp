#include <rclcpp/rclcpp.hpp>
#include "manipulator_msgs/msg/hiper_joint_state.hpp"
#include "manipulator_msgs/msg/manipulator_motor_stage.hpp"
#include "API_robot/motor_dinamixel.hpp"
#include "API_robot/motor_rozum.hpp"





class RobotTestNode : public rclcpp::Node {
public:
    RobotTestNode() : Node("robot_test_node") {
        std::cout << "--- Inicializando Nodo de Control Dynamixel ---" << std::endl;
        
        // DINAMIXEL SETUP
        // Inicialización de los gestores de comunicación. Instanciamos el puerto en "/dev/u2d2_dyn" y el protocolo 2.0
        dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler("/dev/u2d2_dyn");
        dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

        // Abrir el puerto y configurar el bus RS-485
        if (!portHandler->openPort()) {
            std::cerr << "Error crítico: No se pudo abrir el puerto serial." << std::endl;
        }
        if (!portHandler->setBaudRate(57600)) {
            std::cerr << "Error crítico: No se pudo configurar el baudrate a 57600." << std::endl;
        }

        std::cout << "Puerto abierto correctamente a 57600 bps." << std::endl;

        dyn_motor1 = std::make_shared<DinamixelMotor>(portHandler, packetHandler, 1);
        dyn_motor1->set_mode('p'); // Modo posición
        dyn_motor1->set_torque_state(true);


        // ROZUM SETUP
        // Inicializar la interfaz CAN para los motores Rozum
        rr_can_interface_t* can_iface = rr_init_interface("/dev/ttyACM0");
        if (!can_iface) {
            std::cerr << "Error al abrir la interfaz CAN de Rozum." << std::endl;
        }
        rozum_motor1 = std::make_shared<RozumMotor>(can_iface, 1);
        rozum_motor1->activate();
        rozum_motor1->setup_telemetry_cache();

        // Recoger datos por terminal
        this->declare_parameter<int>("timer_period_ms", 20);
        this->get_parameter("timer_period_ms", timer_period_ms);


        // Suscriptor a comandos
        sub_ = this->create_subscription<manipulator_msgs::msg::HiperJointState>(
            "kdl_articular", 10, std::bind(&RobotTestNode::cmd_callback, this, std::placeholders::_1));

        // Publicador de estados (Usamos el mensaje que engloba a todos los motores)
        pub_ = this->create_publisher<manipulator_msgs::msg::ManipulatorMotorStage>("real_robot_data", 10);

        // Timer para publicar el estado de todos los motores (ej. a 10 Hz)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms),
            std::bind(&RobotTestNode::publish_state, this));
    }

private:
    void cmd_callback(const manipulator_msgs::msg::HiperJointState::SharedPtr msg) {
        // Enviar posición al Dynamixel
        dyn_motor1->actuation_motor.position = msg->joint_state_command.position[3];
        dyn_motor1->set_position(nullptr); 

        // Enviar posición al Rozum
        rozum_motor1->actuation_motor.position = msg->joint_state_command.position[0];
        rozum_motor1->set_position();
    }

    void publish_state() {
        manipulator_msgs::msg::ManipulatorMotorStage state_msg;

        // --- INICIALIZAR TODOS LOS MOTORES A 0 ---
        // Llenamos con 0 los 5 motores Dynamixel
        for (int i = 0; i < 5; ++i) {
            state_msg.dinamixel_motors[i].position = 0.0f;       // 4 bytes: Posición[cite: 1]
            state_msg.dinamixel_motors[i].velocity = 0.0f;       // 4 bytes: Velocidad[cite: 1]
            state_msg.dinamixel_motors[i].current = 0;           // 2 bytes: Corriente[cite: 1]
            state_msg.dinamixel_motors[i].temperature = 0;       // 1 byte: Temperatura en ºC[cite: 1]
            state_msg.dinamixel_motors[i].torque_state = false;  // 1 byte: Estado del torque[cite: 1]
        }
        
        // Llenamos con 0 los 3 motores Rozum
        for (int i = 0; i < 3; ++i) {
            state_msg.rozum_motors[i].position = 0.0f;           // 4 bytes: Posición[cite: 3]
            state_msg.rozum_motors[i].velocity = 0.0f;           // 4 bytes: Velocidad[cite: 3]
            state_msg.rozum_motors[i].current = 0;               // 2 bytes: Corriente[cite: 3]
            state_msg.rozum_motors[i].temperature = 0;           // 1 byte: Temperatura en ºC[cite: 3]
        }

        // --- ACTUALIZAR LOS MOTORES QUE SÍ ESTÁN ACTIVOS ---
        // En tu código tienes 1 motor de cada tipo. Suponiendo que ocupan la posición 0 de los arrays:
        
        // Actualizar la telemetría y leer valores del dinamixel
        dyn_motor1->read_all_parameters(nullptr);
        state_msg.dinamixel_motors[0].position = dyn_motor1->telemetry_motor.position;
        state_msg.dinamixel_motors[0].velocity = dyn_motor1->telemetry_motor.velocity;
        state_msg.dinamixel_motors[0].current = dyn_motor1->telemetry_motor.current;
        state_msg.dinamixel_motors[0].temperature = dyn_motor1->telemetry_motor.temperature;   
        state_msg.dinamixel_motors[0].torque_state = true;
        

        // Actualizar la telemetría y leer valores del rozum
        rozum_motor1->update_cache();
        rozum_motor1->read_all_parameters();
        state_msg.rozum_motors[0].position = rozum_motor1->actuation_motor.position;
        state_msg.rozum_motors[0].velocity = rozum_motor1->telemetry_motor.velocity;
        state_msg.rozum_motors[0].current = rozum_motor1->telemetry_motor.current;
        state_msg.rozum_motors[0].temperature = rozum_motor1->telemetry_motor.temperature;
        // Igual que arriba, puedes añadir lecturas de sensores si la API lo permite.

        // Publicar el mensaje
        pub_->publish(state_msg);
    }

    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    std::shared_ptr<DinamixelMotor> dyn_motor1;
    std::shared_ptr<RozumMotor> rozum_motor1;

    rclcpp::Subscription<manipulator_msgs::msg::HiperJointState>::SharedPtr sub_;
    rclcpp::Publisher<manipulator_msgs::msg::ManipulatorMotorStage>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int timer_period_ms;
};



// ==========================================
//             FUNCIÓN MAIN
// ==========================================
int main(int argc, char **argv)
{
    // Inicializar el sistema de ROS 2
    rclcpp::init(argc, argv);

    try {
        // Crear el nodo pasándole la interfaz CAN
        auto node = std::make_shared<RobotTestNode>();

        std::cout << "Nodo RobotTestNode ejecutándose correctamente." << std::endl;

        // Mantener el nodo vivo procesando callbacks (temporizador y suscriptores)
        rclcpp::spin(node);
    } 
    catch (const std::exception& e) {
        // Capturar la excepción si falla la apertura del puerto serial Dynamixel
        std::cerr << "Excepción fatal durante la ejecución: " << e.what() << std::endl;
    }

    // Limpieza al cerrar el nodo 
    std::cout << "Apagando el nodo y cerrando ROS 2..." << std::endl;
    
    // (Opcional) Si tu API de Rozum requiere cerrar la interfaz CAN, hazlo aquí:
    // rr_deinit_interface(&can_iface);

    rclcpp::shutdown();
    return 0;
}