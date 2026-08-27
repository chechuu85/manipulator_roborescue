#include "manual_user_interface/kdlCartesianToJoint.hpp"

// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================
KdlCartesianToJoint::KdlCartesianToJoint() : Node("kdl_ik_node")
{
    joint_velocity_limit_ = 1.0;

    // Inicializar KDL y verificar que se ha inicializado correctamente
    if (!this->initKDL()) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo inicializar KDL. Abortando.");
        rclcpp::shutdown();
    }

    // Crear suscripciones y publicadores
    sub_js_robot = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&KdlCartesianToJoint::robotJointStateCallback, this, std::placeholders::_1));
    sub_twist_input_ = this->create_subscription<manipulator_msgs::msg::HiperTwist>(
        "/input_cartesian", 10, std::bind(&KdlCartesianToJoint::twistCallback, this, std::placeholders::_1));
    sub_js_input_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/input_articular", 10, std::bind(&KdlCartesianToJoint::jStateCallback, this, std::placeholders::_1));
    sub_planing_pose_input_ = this->create_subscription<manipulator_msgs::msg::HiperPose>(
        "/planning_pose", 10, std::bind(&KdlCartesianToJoint::poseCallback, this, std::placeholders::_1));

    pub_cmd_ = this->create_publisher<manipulator_msgs::msg::HiperJointState>("/kdl_articular", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo KDL (Cartesiano -> Articular) inicializado correctamente.");
}

KdlCartesianToJoint::~KdlCartesianToJoint() {
    RCLCPP_INFO(this->get_logger(), "Nodo KDL finalizado. ");
}

bool KdlCartesianToJoint::initKDL()
{
    // Leer robot_description (cadena cinemática del robot en formato URDF)
    this->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
    std::string robot_description;
    
    if (!this->get_parameter("robot_description", robot_description)) {
        RCLCPP_ERROR(this->get_logger(), "No se encontró el parámetro 'robot_description'. Asegúrate de publicarlo o mapearlo.");
    }

    // Construir el árbol y la cadena KDL
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_description, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Fallo al parsear el URDF a KDL.");
        return false;
    }

    // Extraemos la cadena desde la base hasta la muñeca/TCP (link7)
    if (!tree.getChain("base_link", "link7", chain_)) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo obtener la cadena cinemática base_link -> link7.");
        return false;
    }

    
    // Extrae las articulaciones móviles que hay en la cadena
    q_current_.resize(chain_.getNrOfJoints());
    KDL::SetToZero(q_current_);

    // Inicializamos el solver IK de velocidades (Damped Least Squares) evitando singularidades 
    ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_wdls>(chain_, 1e-6);

    // Inicializas el solver de FK pasándole tu cadena cinemática
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);

    // Asignamos pesos para dar un poco de prioridad a la traslación sobre la rotación
    Eigen::MatrixXd Wts = Eigen::MatrixXd::Identity(6, 6);
    Wts(3, 3) = 0.1;  // rot x
    Wts(4, 4) = 0.1;  // rot y
    Wts(5, 5) = 0.1;  // rot z
    ik_vel_solver_->setWeightTS(Wts);

    // Crear el solver de posición IK (Newton-Raphson) inyectando los dos anteriores
    ik_pos_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR>(
        chain_, *fk_solver_, *ik_vel_solver_, 100, 1e-6);

    // Variables para crear trayectoria 
    joint_planning_positions_.resize(chain_.getNrOfJoints());
    KDL::SetToZero(joint_planning_positions_);
    next_joint_planning_positions_.resize(chain_.getNrOfJoints());
    KDL::SetToZero(next_joint_planning_positions_);

    // Guardamos nombres del archivo .xacro
    joint_names_.clear();
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
        const KDL::Joint& joint = chain_.getSegment(i).getJoint();
        
        // Filtramos para guardar solo las articulaciones móviles (ignorando las fijas)
        if (joint.getType() != KDL::Joint::None) {
        joint_names_.push_back(joint.getName());
        }
    }

    return true;
}

// ==========================================
// INTERRUPCIONES
// ==========================================
void KdlCartesianToJoint::poseCallback(const manipulator_msgs::msg::HiperPose::SharedPtr msg)
{
    
    // Obtener posición anterior según mensaje obtenido 
    joint_planning_positions_ = (msg->command_info == "FIRST") ? q_current_ : next_joint_planning_positions_;

    // Obtener posición deseada del mensaje 
    const KDL::Frame desired_ee_pose(
        KDL::Rotation::Quaternion(
            msg->pose_command.orientation.x, 
            msg->pose_command.orientation.y, 
            msg->pose_command.orientation.z, 
            msg->pose_command.orientation.w
        ),
        KDL::Vector(
            msg->pose_command.position.x, 
            msg->pose_command.position.y, 
            msg->pose_command.position.z
        )
    );

    // Calcular espacio articular
    if (ik_pos_solver_->CartToJnt(joint_planning_positions_, desired_ee_pose, next_joint_planning_positions_) < 0) { 
            RCLCPP_WARN(this->get_logger(), "Error: IK falló al calcular es espacio de estados articular."); 
            return; 
        }

    
    // Se crean datos necesarios y se rellenan huecos para ser publicados
    manipulator_msgs::msg::HiperJointState joint_publish;
    joint_publish.joint_state_command.position.resize(chain_.getNrOfJoints());
    joint_publish.joint_state_command.velocity.resize(chain_.getNrOfJoints());

    // Rellenan las posiciones
    std::memcpy(
        joint_publish.joint_state_command.position.data(), 
        next_joint_planning_positions_.data.data(),
        joint_publish.joint_state_command.position.size() * sizeof(double)
    );
    // Se rellena velocidades con 0 para que no tengan basura
    std::fill(joint_publish.joint_state_command.velocity.begin(), joint_publish.joint_state_command.velocity.end(), 0.0);

    // Escribe mensaje necesario 
    joint_publish.command_info = (msg->command_info == "FIRST") ? "FIRST" : ((msg->command_info == "LAST") ? "LAST" : "RECORDING");
    pub_cmd_-> publish(joint_publish);
}

void KdlCartesianToJoint::robotJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{   
    // Actualizar las posiciones articulares leyendo por nombre para evitar problemas de índices
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        for (size_t j = 0; j < msg->name.size(); ++j) {
            if (msg->name[j] == joint_names_[i]) {
                q_current_(i) = msg->position[j];
                break;
            }
        }
    }
}

void KdlCartesianToJoint::jStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
    // Cambiar el topic por donde se publican los datos 
    manipulator_msgs::msg::HiperJointState cmd_msg;
    
    // Avisa que es un comando manual
    cmd_msg.command_info = "MANUAL";
    cmd_msg.joint_state_command = *msg;

    // Se asegura que el array es de 8 elementos para no causar daños en el siguiente nodo
    cmd_msg.joint_state_command.position.resize(8, 0.0);
    cmd_msg.joint_state_command.velocity.resize(8, 0.0);

    pub_cmd_->publish(cmd_msg);
 }

void KdlCartesianToJoint::twistCallback(const manipulator_msgs::msg::HiperTwist::SharedPtr msg)
{
    //Crear variables
    KDL::Twist twist_target;
    twist_target.vel.x(msg->twist_command.linear.x);
    twist_target.vel.y(msg->twist_command.linear.y);
    twist_target.vel.z(msg->twist_command.linear.z);
    twist_target.rot.x(msg->twist_command.angular.x);
    twist_target.rot.y(msg->twist_command.angular.y);
    twist_target.rot.z(msg->twist_command.angular.z);

    KDL::JntArray q_dot(chain_.getNrOfJoints());  // Dimensiona contenedor para velocidades articulares (q_dot) de cinemática inversa

    // Inicializamos el contenido del mensaje a publicar
    manipulator_msgs::msg::HiperJointState cmd_msg;

    cmd_msg.joint_state_command.position.resize(8, 0.0);
    cmd_msg.joint_state_command.velocity.resize(8, 0.0);


    if (msg->command_info == "TCP") {
        // Frame para guardar la pose actual de link7 respecto a base_link
        KDL::Frame tcp_frame; 
        
        // Calculamos la cinemática directa (Forward Kinematics) para saber dónde está el TCP ahora mismo
        if (fk_solver_->JntToCart(q_current_, tcp_frame) < 0) { 
            RCLCPP_WARN(this->get_logger(), "Error: FK falló al calcular la pose del TCP."); 
            return; 
        }
        
        // Se pasan las velocidades del punto de referencia del TCP a la base 
        // tcp_frame.M ( matriz de rotación del TCP actual )
        twist_target = tcp_frame.M * twist_target; 
    }


    // Calculamos la cinemática inversa de velocidades
    if (ik_vel_solver_->CartToJnt(q_current_, twist_target, q_dot) < 0) {
        RCLCPP_WARN(this->get_logger(), "IK falló al intentar resolver las velocidades.");
        return;
    }

    // Generar saturación
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {        
        cmd_msg.joint_state_command.velocity[i] = std::clamp(q_dot(i), -joint_velocity_limit_, joint_velocity_limit_);
    }
    
    // Control de velocidad de la garra 
    cmd_msg.joint_state_command.velocity[6] = msg->gripper;
    cmd_msg.joint_state_command.velocity[7] = -msg->gripper;

    cmd_msg.command_info = "MANUAL";

    pub_cmd_->publish(cmd_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KdlCartesianToJoint>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}