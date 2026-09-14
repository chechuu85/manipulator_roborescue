#include <rclcpp/rclcpp.hpp>
#include "manipulator_msgs/msg/hiper_joint_state.hpp"
#include "manipulator_msgs/msg/manipulator_motor_stage.hpp"
#include "API_robot/motor_dinamixel.hpp"
#include "API_robot/motor_rozum.hpp"
#include "API_robot/robot_arm.hpp"
#include "API_robot/robot_claw.hpp"



class RobotTestNode : public rclcpp::Node {
public:
    RobotTestNode() : Node("robot_test_node") {
        std::cout << "--- Inicializando Nodo de Control Dynamixel ---" << std::endl;
        
        // DINAMIXEL SETUP
        // Inicialización de los gestores de comunicación. Instanciamos el puerto en "/dev/u2d2_dyn" y el protocolo 2.0
        dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
        dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

        // Abrir el puerto y configurar el bus RS-485
        if (!portHandler->openPort()) {
            std::cerr << "Error crítico: No se pudo abrir el puerto serial." << std::endl;
        }
        if (!portHandler->setBaudRate(57600)) {
            std::cerr << "Error crítico: No se pudo configurar el baudrate a 57600." << std::endl;
        }

        // dyn_motor1 = std::make_shared<DinamixelMotor>(portHandler, packetHandler, 1);
        // dyn_motor1->set_mode('v'); // Modo posición/velocidad
        // dyn_motor1->set_torque_state(true);
        // dyn_motor2 = std::make_shared<DinamixelMotor>(portHandler, packetHandler, 3);
        // dyn_motor2->set_mode('v'); // Modo posición/velocidad
        // dyn_motor2->set_torque_state(true);
        // dyn_motor3 = std::make_shared<DinamixelMotor>(portHandler, packetHandler, 5);
        // dyn_motor3->set_mode('v'); // Modo posición/velocidad
        // dyn_motor3->set_torque_state(true);
        // dyn_motor4 = std::make_shared<DinamixelMotor>(portHandler, packetHandler, 2);
        // dyn_motor4->set_mode('v'); // Modo posición/velocidad
        // dyn_motor4->set_torque_state(true);
        // dyn_motor5 = std::make_shared<DinamixelMotor>(portHandler, packetHandler, 12);
        // dyn_motor5->set_mode('v'); // Modo posición/velocidad
        // dyn_motor5->set_torque_state(true);
        dyn_claw = std::make_shared<DynamixelClaw>(portHandler, packetHandler, 1, 3, 5, 2, 12); // IDs estándar de Dynamixel en tu código
        dyn_claw->set_torque_all(true);
        dyn_claw->set_mode_all('v');


        // ROZUM SETUP
        // Inicializar la interfaz CAN para los motores Rozum
        // rr_can_interface_t* can_iface = rr_init_interface("/dev/ttyACM0");
        // if (!can_iface) {
        //     std::cerr << "Error al abrir la interfaz CAN de Rozum." << std::endl;
        // }
        rr_can_interface_t* can_iface = rr_init_interface("udp://192.168.0.123:2000");

        if (!can_iface) {
            std::cerr << "Error al abrir la interfaz CAN Ethernet de Rozum."
                    << std::endl;
            return 1;
        }


        // rozum_motor1 = std::make_shared<RozumMotor>(can_iface, 123);
        // rozum_motor1->activate();
        // rozum_motor1->setup_telemetry_cache();
        // rozum_motor2 = std::make_shared<RozumMotor>(can_iface, 124);
        // rozum_motor2->activate();
        // rozum_motor2->setup_telemetry_cache();
        // rozum_motor3 = std::make_shared<RozumMotor>(can_iface, 125);
        // rozum_motor3->activate();
        // rozum_motor3->setup_telemetry_cache();
        rozum_arm = std::make_shared<RozumArm>(can_iface, 123, 124, 125);
        rozum_arm->activate_all();
        rozum_arm->setup_telemetry_cache_all();

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
        // dyn_motor1->actuation_motor.position = 2000; //msg->joint_state_command.position[3];
        // dyn_motor1->set_position(nullptr); 

        // dyn_motor1->actuation_motor.velocity = msg->joint_state_command.velocity[3];
        // dyn_motor1->set_velocity(nullptr);
        // dyn_motor2->actuation_motor.velocity = msg->joint_state_command.velocity[4];
        // dyn_motor2->set_velocity(nullptr);
        // dyn_motor3->actuation_motor.velocity = msg->joint_state_command.velocity[5];
        // dyn_motor3->set_velocity(nullptr);
        // dyn_motor4->actuation_motor.velocity = msg->joint_state_command.velocity[6];
        // dyn_motor4->set_velocity(nullptr);
        // dyn_motor5->actuation_motor.velocity = msg->joint_state_command.velocity[7];
        // dyn_motor5->set_velocity(nullptr);
        dyn_claw->set_velocities({  msg->joint_state_command.velocity[3],
                                    msg->joint_state_command.velocity[4],
                                    msg->joint_state_command.velocity[5],
                                    msg->joint_state_command.velocity[6],
                                    msg->joint_state_command.velocity[7]});


        // Enviar posición al Rozum
        // rozum_motor1->actuation_motor.position = msg->joint_state_command.position[0];
        // rozum_motor1->set_position();
        // rozum_motor1->actuation_motor.velocity = msg->joint_state_command.velocity[0];
        // rozum_motor1->set_velocity();
        rozum_arm->set_velocities({  msg->joint_state_command.velocity[0],
                                     msg->joint_state_command.velocity[1],
                                     msg->joint_state_command.velocity[2]});

        // rozum_motor2->actuation_motor.velocity = msg->joint_state_command.velocity[1];
        // rozum_motor2->set_velocity();

        // rozum_motor3->actuation_motor.velocity = msg->joint_state_command.velocity[2];
        // rozum_motor3->set_velocity();
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
        dyn_claw->read_all_parameters();
        //dyn_motor1->read_all_parameters(nullptr);
        state_msg.dinamixel_motors[0].position = dyn_claw->motor1.telemetry_motor.position;
        state_msg.dinamixel_motors[0].velocity = dyn_claw->motor1.telemetry_motor.velocity;
        state_msg.dinamixel_motors[0].current = dyn_claw->motor1.telemetry_motor.current;
        state_msg.dinamixel_motors[0].temperature = dyn_claw->motor1.telemetry_motor.temperature;   
        state_msg.dinamixel_motors[0].torque_state = true;

        //dyn_motor2->read_all_parameters(nullptr);
        state_msg.dinamixel_motors[1].position = dyn_claw->motor2.telemetry_motor.position;
        state_msg.dinamixel_motors[1].velocity = dyn_claw->motor2.telemetry_motor.velocity;
        state_msg.dinamixel_motors[1].current = dyn_claw->motor2.telemetry_motor.current;
        state_msg.dinamixel_motors[1].temperature = dyn_claw->motor2.telemetry_motor.temperature;   
        state_msg.dinamixel_motors[1].torque_state = true;

        //dyn_motor3->read_all_parameters(nullptr);
        state_msg.dinamixel_motors[2].position = dyn_claw->motor3.telemetry_motor.position;
        state_msg.dinamixel_motors[2].velocity = dyn_claw->motor3.telemetry_motor.velocity;
        state_msg.dinamixel_motors[2].current = dyn_claw->motor3.telemetry_motor.current;
        state_msg.dinamixel_motors[2].temperature = dyn_claw->motor3.telemetry_motor.temperature;   
        state_msg.dinamixel_motors[2].torque_state = true;

        //dyn_motor4->read_all_parameters(nullptr);
        state_msg.dinamixel_motors[3].position = dyn_claw->motor5.telemetry_motor.position;
        state_msg.dinamixel_motors[3].velocity = dyn_claw->motor5.telemetry_motor.velocity;
        state_msg.dinamixel_motors[3].current = dyn_claw->motor5.telemetry_motor.current;
        state_msg.dinamixel_motors[3].temperature = dyn_claw->motor5.telemetry_motor.temperature;   
        state_msg.dinamixel_motors[3].torque_state = true;

        //dyn_motor5->read_all_parameters(nullptr);
        state_msg.dinamixel_motors[4].position = dyn_claw->motor12.telemetry_motor.position;
        state_msg.dinamixel_motors[4].velocity = dyn_claw->motor12.telemetry_motor.velocity;
        state_msg.dinamixel_motors[4].current = dyn_claw->motor12.telemetry_motor.current;
        state_msg.dinamixel_motors[4].temperature = dyn_claw->motor12.telemetry_motor.temperature;   
        state_msg.dinamixel_motors[4].torque_state = true;
        

        // Actualizar la telemetría y leer valores del rozum
        // rozum_arm->update_cache_all(). No se hace porque ya se hace dentro de la función de la clase rozum_arm
        rozum_arm->read_all_parameters();
        //rozum_motor1->update_cache();
        //rozum_motor1->read_all_parameters();
        state_msg.rozum_motors[0].position = rozum_arm->motor1.telemetry_motor.position;
        state_msg.rozum_motors[0].velocity = rozum_arm->motor1.telemetry_motor.velocity;
        state_msg.rozum_motors[0].current = rozum_arm->motor1.telemetry_motor.current;
        state_msg.rozum_motors[0].temperature = rozum_arm->motor1.telemetry_motor.temperature;

        //rozum_motor2->update_cache();
        //rozum_motor2->read_all_parameters();
        state_msg.rozum_motors[1].position = rozum_arm->motor2.telemetry_motor.position;
        state_msg.rozum_motors[1].velocity = rozum_arm->motor2.telemetry_motor.velocity;
        state_msg.rozum_motors[1].current = rozum_arm->motor2.telemetry_motor.current;
        state_msg.rozum_motors[1].temperature = rozum_arm->motor2.telemetry_motor.temperature;

        //rozum_motor3->update_cache();
        //rozum_motor3->read_all_parameters();
        state_msg.rozum_motors[2].position = rozum_arm->motor3.telemetry_motor.position;
        state_msg.rozum_motors[2].velocity = rozum_arm->motor3.telemetry_motor.velocity;
        state_msg.rozum_motors[2].current = rozum_arm->motor3.telemetry_motor.current;
        state_msg.rozum_motors[2].temperature = rozum_arm->motor3.telemetry_motor.temperature;
        // Igual que arriba, puedes añadir lecturas de sensores si la API lo permite.

        // Publicar el mensaje
        pub_->publish(state_msg);
    }

    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    // std::shared_ptr<DinamixelMotor> dyn_motor1;
    // std::shared_ptr<DinamixelMotor> dyn_motor2;
    // std::shared_ptr<DinamixelMotor> dyn_motor3;
    // std::shared_ptr<DinamixelMotor> dyn_motor4;
    // std::shared_ptr<DinamixelMotor> dyn_motor5;
    std::shared_ptr<DynamixelClaw> dyn_claw;
    // std::shared_ptr<RozumMotor> rozum_motor1;
    // std::shared_ptr<RozumMotor> rozum_motor2;
    // std::shared_ptr<RozumMotor> rozum_motor3;
    std::shared_ptr<RozumArm> rozum_arm;

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