#include "advanced_user_manual/trajectoryPlanning.hpp"
#include "advanced_user_manual/trajectory_math.hpp" 


// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================
TrajectoryPlanningNode::TrajectoryPlanningNode() : Node("trayectory_planing"){

    sub_instr_trayectory = this->create_subscription<std_msgs::msg::String>(
        "input_instr_trayectory", 10, std::bind(&TrajectoryPlanningNode::keyboardCallback, this, std::placeholders::_1));
    
    // Inicializar el cliente para el servicio stop_service
    client_pose = this->create_client<manipulator_msgs::srv::GetCurrentPose>("/service_odometry_pose");


    RCLCPP_INFO(this->get_logger(), "Nodo Trayectory Planning inicializado correctamente");

}

TrajectoryPlanningNode::~TrajectoryPlanningNode(){
    RCLCPP_INFO(this->get_logger(), "Nodo Trayectory Planning finalizado correctamente");
}


// ==========================================
// INTERRUPCIÓN
// ==========================================
void TrajectoryPlanningNode::keyboardCallback(const std_msgs::msg::String::SharedPtr msg){

    if(msg->data=="SAVE"){
        // Creo petición
        auto request_pose = std::make_shared<manipulator_msgs::srv::GetCurrentPose::Request>();

        // Mando petición y creo variable para guardar respueta cuando llegue
        auto result_pose = client_pose->async_send_request(request_pose);

        // Se crea función que se ejecutará cuando llegue el mensaje
        // [this] permite usar métodos de nuestra clase 
        auto response_callback = [this](rclcpp::Client<manipulator_msgs::srv::GetCurrentPose>::SharedFuture future_msg) {
            // Comprobar si la respuesta es válida y guardar respuesta en variable
            try {
                // Obtener la información
                auto content_response = future_msg.get();
                auto pose = content_response->current_pose;
                
                RCLCPP_INFO_STREAM(this->get_logger(), "Posición guardada -> X: " << pose.position.x 
                                                                        << ", Y: " << pose.position.y
                                                                        << ", Z: " << pose.position.z);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "El servicio falló: %s", e.what());
            }
        };

        // Enviar la petición asíncrona pasando el callback
        client_pose->async_send_request(request_pose, response_callback);
        
    }
}


// ==========================================
// MAIN
// ==========================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPlanningNode>();
    
    // Usar ejecutor multihilo para procesar las respuestas en paralelo
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}


// using namespace std::chrono_literals;

// TrajectoryPlanningNode::TrajectoryPlanningNode() : Node("trayectory_planing")
// {
//     // 1. Crear el publicador[cite: 5]
//     trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//         "/r6bot_controller/joint_trajectory", 10);

//     // 2. Declarar parámetros[cite: 5]
//     this->declare_parameter<std::string>("robot_description", "");
//     this->declare_parameter<std::string>("poses_yaml", "");

//     // 3. Inicializar cinemática y cargar poses
//     initialize_kinematics();
//     load_poses_from_yaml();

//     // 4. Usar un timer para ejecutar la planificación una vez que el nodo esté "girando" (spinning)
//     timer_ = this->create_wall_timer(
//         500ms, std::bind(&TrajectoryPlanningNode::generate_and_publish_trajectory, this));
// }

// void TrajectoryPlanningNode::initialize_kinematics()
// {
//     std::string robot_description;
//     this->get_parameter("robot_description", robot_description);

//     KDL::Tree robot_tree;
//     if (!kdl_parser::treeFromString(robot_description, robot_tree)) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree from robot_description");
//         return;
//     }

//     // Extraer cadena cinemática (Asegúrate de que 'base_link' y 'tool0' existan en tu URDF)[cite: 5]
//     robot_tree.getChain("base_link", "tool0", chain_);

//     // Inicializar solvers[cite: 5]
//     fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
//     ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_, 0.0000001);
//     ik_pos_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR>(
//         chain_, *fk_solver_, *ik_vel_solver_, 100, 1e-6);
        
//     RCLCPP_INFO(this->get_logger(), "Kinematics initialized successfully.");
// }

// void TrajectoryPlanningNode::load_poses_from_yaml()
// {
//     std::string poses_yaml_path;
//     this->get_parameter("poses_yaml", poses_yaml_path);

//     if (poses_yaml_path.empty()) {
//         poses_yaml_path = ament_index_cpp::get_package_share_directory("cartesian_trajectory_planning") +
//                           "/config/poses.yaml";
//     }

//     try {
//         const YAML::Node poses_root = YAML::LoadFile(poses_yaml_path);
//         // Usamos la función importada de trajectory_math.hpp[cite: 5]
//         pose0_ = ParsePoseMatrix(poses_root, "pose0");
//         pose1_ = ParsePoseMatrix(poses_root, "pose1");
//         pose2_ = ParsePoseMatrix(poses_root, "pose2");
//         RCLCPP_INFO(this->get_logger(), "Poses loaded from YAML.");
//     } catch (const std::exception &e) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to load poses YAML '%s': %s", poses_yaml_path.c_str(), e.what());
//     }
// }

// void TrajectoryPlanningNode::generate_and_publish_trajectory()
// {
//     // Cancelar el timer para que esto se ejecute solo una vez
//     timer_->cancel();

//     trajectory_msgs::msg::JointTrajectory trajectory_msg;
//     trajectory_msg.header.stamp = this->now();
    
//     for (size_t i = 0; i < chain_.getNrOfSegments(); i++) {
//         auto joint = chain_.getSegment(i).getJoint();
//         if (joint.getType() != KDL::Joint::Fixed) {
//             trajectory_msg.joint_names.push_back(joint.getName());
//         }
//     }

//     trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
//     trajectory_point_msg.positions.resize(chain_.getNrOfJoints());
//     trajectory_point_msg.velocities.resize(chain_.getNrOfJoints());

//     int tau = 1; 
//     int T = 10;  
//     const double sample_time = 0.1;
//     int point_index = 0;

//     auto joint_positions = KDL::JntArray(chain_.getNrOfJoints());

//     // Loop de cálculo cartesiano e IK[cite: 5]
//     for (double t = -T; t <= T + 1e-9; t += sample_time) {
//         // Usamos ComputeNextCartesianPose de trajectory_math.hpp[cite: 5]
//         const auto [p_interp, q_interp] = ComputeNextCartesianPose(pose0_, pose1_, pose2_, tau, T, t);

//         const KDL::Frame desired_ee_pose(
//             KDL::Rotation::Quaternion(q_interp.x(), q_interp.y(), q_interp.z(), q_interp.w()),
//             KDL::Vector(p_interp.x(), p_interp.y(), p_interp.z()));

//         KDL::JntArray next_joint_positions(chain_.getNrOfJoints());
//         const int ik_status = ik_pos_solver_->CartToJnt(joint_positions, desired_ee_pose, next_joint_positions);
        
//         if (ik_status < 0) {
//             RCLCPP_WARN(this->get_logger(), "IK failed at t=%.3f with error code %d. Skipping point.", t, ik_status);
//             continue;
//         }

//         std::memcpy(trajectory_point_msg.positions.data(), next_joint_positions.data.data(),
//                     trajectory_point_msg.positions.size() * sizeof(double));
//         std::fill(trajectory_point_msg.velocities.begin(), trajectory_point_msg.velocities.end(), 0.0);

//         const double elapsed = static_cast<double>(point_index + 1) * sample_time;
//         trajectory_point_msg.time_from_start.sec = static_cast<int32_t>(elapsed);
//         trajectory_point_msg.time_from_start.nanosec = static_cast<uint32_t>(
//             (elapsed - static_cast<double>(trajectory_point_msg.time_from_start.sec)) * 1e9);

//         trajectory_msg.points.push_back(trajectory_point_msg);
//         joint_positions = next_joint_positions;
//         point_index++;
//     }

//     if (trajectory_msg.points.empty()) {
//         RCLCPP_ERROR(this->get_logger(), "No valid trajectory points were generated.");
//         return;
//     }

//     // Esperar a que el controlador esté escuchando[cite: 5]
//     while (trajectory_pub_->get_subscription_count() == 0 && rclcpp::ok()) {
//         RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for joint_trajectory subscriber...");
//         rclcpp::sleep_for(200ms);
//     }

//     trajectory_msg.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.2);
//     RCLCPP_INFO(this->get_logger(), "Publishing %zu trajectory points.", trajectory_msg.points.size());
//     trajectory_pub_->publish(trajectory_msg);
// }
