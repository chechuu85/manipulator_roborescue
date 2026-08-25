#include "advanced_user_manual/trajectoryPlanning.hpp"
#include "advanced_user_manual/trajectory_math.hpp" 


// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================
TrajectoryPlanningNode::TrajectoryPlanningNode() : Node("trayectory_planing"){

    // Construir la ruta al archivo 
    poses_yaml_path = ament_index_cpp::get_package_share_directory("bringup");
    size_t install_pos = poses_yaml_path.find("/install/");
    poses_yaml_path = poses_yaml_path.substr(0, install_pos) + "/src/bringup/description/poses.yaml";

    RCLCPP_INFO(this->get_logger(), "Ruta cargada: %s", poses_yaml_path.c_str());

    const YAML::Node poses_root = YAML::LoadFile(poses_yaml_path); 
    num_pose = poses_root.size();

    // Obtener el valor del parámetro por el launch
    int timer_period_ms;
    this->declare_parameter<int>("timer_period_ms", 20);
    this->get_parameter("timer_period_ms", timer_period_ms);
    sample_time_ = timer_period_ms/1000.0;

    // Inicializar el cliente y servicio
    sub_instr_trayectory = this->create_subscription<std_msgs::msg::String>(
        "input_instr_trayectory", 10, std::bind(&TrajectoryPlanningNode::keyboardCallback, this, std::placeholders::_1));
    client_pose = this->create_client<manipulator_msgs::srv::GetCurrentPose>("/service_odometry_pose");

    pub_planning_pose_ = this->create_publisher<manipulator_msgs::msg::HiperPose>("/planning_pose", 10);

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

                Eigen::Matrix4d pose_yaml = PoseToMatrix(pose);
                std::string key = "pose" + std::to_string(num_pose);
                AddPoseMatrix(key, pose_yaml, poses_yaml_path);
                num_pose++;
                
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "El servicio falló: %s", e.what());
            }
        };

        // Enviar la petición asíncrona pasando el callback
        client_pose->async_send_request(request_pose, response_callback);
        
    } else if (msg->data=="PLAY") {

        RCLCPP_INFO(this->get_logger(), "Comando PLAY recibido. Obteniendo pose actual para inicio suave...");

        auto request_pose = std::make_shared<manipulator_msgs::srv::GetCurrentPose::Request>();

        auto response_callback = [this](rclcpp::Client<manipulator_msgs::srv::GetCurrentPose>::SharedFuture future_msg) {
            try {
                // Obtener la pose actual del robot real
                auto content_response = future_msg.get();
                Eigen::Matrix4d real_start_pose = PoseToMatrix(content_response->current_pose);

                // Inicializar el vector con la pose actual como punto 0
                trajectory_poses.clear();
                trajectory_poses.push_back(real_start_pose);

                // Cargar el resto de poses desde el YAML dinámicamente
                YAML::Node poses_root = YAML::LoadFile(poses_yaml_path); 
                while (true) {
                    num_pose--;
                    std::string key = "pose" + std::to_string(num_pose);
                    if (!poses_root[key]) break;

                    trajectory_poses.push_back(ParsePoseMatrix(key, poses_yaml_path)); 
                    
                    // Eliminar la pose de la estructura en memoria
                    poses_root.remove(key);
                    std::ofstream fout(poses_yaml_path);
                    fout << poses_root;
                    fout.close();
                }

                // Revisa que haya más de 3 poses 
                if (trajectory_poses.size() < 3) {
                    RCLCPP_ERROR(this->get_logger(), "Faltan poses. Se encontraron %zu, se requieren al menos 3.", trajectory_poses.size());
                    return; // Abortar ejecución para no enviar basura a KDL
                }

                // Si hay un timer ejecutándose, lo detenemos por seguridad
                if (timer_play_) {
                    timer_play_->cancel();
                }


                // Creamos e iniciamos el timer
                timer_play_ = this->create_wall_timer(
                    std::chrono::milliseconds(static_cast<int>(sample_time_ * 1000)),
                    std::bind(&TrajectoryPlanningNode::timer_play_callback, this)
                );

            } catch (const std::exception &e) { 
                RCLCPP_ERROR(this->get_logger(), "Fallo al generar trayectoria en PLAY: %s", e.what());
            }
        };
        client_pose->async_send_request(request_pose, response_callback);
    }
}


void TrajectoryPlanningNode::timer_play_callback()
{
    // Condición de finalización del bucle (t <= T + 1e-9)
    if (t_current_ <= t_max_ + 1e-9) {
        
        const auto [p_interp, q_interp] = ComputeNextCartesianPose(
            trajectory_poses[0], trajectory_poses[1], trajectory_poses[2], tau_, t_max_, t_current_);

        manipulator_msgs::msg::HiperPose hiperpose_msg;

        hiperpose_msg.command_info = (t_current_ <= -t_max_ + 1e-6) ? "FIRST" : ((t_current_ >= t_max_ - 1e-6) ? "LAST" : "");

        hiperpose_msg.pose_command.position.x = p_interp.x();
        hiperpose_msg.pose_command.position.y = p_interp.y();
        hiperpose_msg.pose_command.position.z = p_interp.z();

        hiperpose_msg.pose_command.orientation.x = q_interp.x();
        hiperpose_msg.pose_command.orientation.y = q_interp.y();
        hiperpose_msg.pose_command.orientation.z = q_interp.z();
        hiperpose_msg.pose_command.orientation.w = q_interp.w();

        pub_planning_pose_->publish(hiperpose_msg);

        // Avanzamos el paso (equivalente al t += sample_time del for)
        t_current_ += sample_time_;

    } else {
        // La trayectoria terminó
        RCLCPP_INFO(this->get_logger(), "Ejecución de trayectoria finalizada.");
        
        // Detenemos el timer para que no siga publicando
        timer_play_->cancel();
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