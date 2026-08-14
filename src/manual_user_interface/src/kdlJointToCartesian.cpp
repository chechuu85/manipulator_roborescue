#include "manual_user_interface/kdlJointToCartesian.hpp"

// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================
KdlJointToCartesian::KdlJointToCartesian() : Node("kdl_joint_to_cartesian")
{
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

  if (!this->initKdl()) {
    RCLCPP_ERROR(this->get_logger(), "No se pudo inicializar KDL para el servidor de pose actual.");
    throw std::runtime_error("KDL init failed");
  }

  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&KdlJointToCartesian::jointStateCallback, this, std::placeholders::_1));

  current_pose_service_ = this->create_service<manipulator_msgs::srv::GetCurrentPose>(
    "/service_odometry_pose", std::bind(&KdlJointToCartesian::handleCurrentPose, this, 
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)  );

  RCLCPP_INFO(this->get_logger(), "Servidor de pose actual listo. Servicio: /get_current_pose");
}

KdlJointToCartesian::~KdlJointToCartesian() {
    RCLCPP_INFO(this->get_logger(), "Servicio KDL finalizado. ");
}


bool KdlJointToCartesian::initKdl()
{
  // Leer robot_description (cadena cinemática del robot en formato URDF)
  this->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  std::string robot_description;
  if (!this->get_parameter("robot_description", robot_description) || robot_description.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No se encontró el parámetro 'robot_description'.");
    return false;
  }

  // Construir el árbol y la cadena KDL
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(robot_description, tree)) {
    RCLCPP_ERROR(this->get_logger(), "Fallo al parsear URDF en KDL.");
    return false;
  }

  // Extraemos la cadena desde la base hasta la muñeca/TCP (link7)
  if (!tree.getChain("base_link", "link7", chain_)) {
    RCLCPP_ERROR(this->get_logger(), "No se pudo obtener la cadena cinemática base_link -> link7.");
    return false;
  }

  q_current_.resize(chain_.getNrOfJoints());
  KDL::SetToZero(q_current_);
  fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
  return true;
}

// ==========================================
// INTERRUPCIONES
// ==========================================
void KdlJointToCartesian::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.empty() || msg->position.empty()) {
    return;
  }

  for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
    for (size_t j = 0; j < msg->name.size(); ++j) {
      if (msg->name[j] == joint_names_[i]) {
        q_current_(i) = msg->position[j];
        break;
      }
    }
  }
}

void KdlJointToCartesian::handleCurrentPose(
    const std::shared_ptr<rmw_request_id_t>request_header,
    const std::shared_ptr<manipulator_msgs::srv::GetCurrentPose::Request>request,
    std::shared_ptr<manipulator_msgs::srv::GetCurrentPose::Response>response){

  // Calcula cinemática directa y verifica si lo hace correctamente
  KDL::Frame tcp_frame;
  if (fk_solver_->JntToCart(q_current_, tcp_frame) < 0) {
    RCLCPP_WARN(this->get_logger(), "FK falló al calcular la pose actual del TCP.");
    response->current_pose = geometry_msgs::msg::Pose();
  }

  // Obtener posición
  response->current_pose.position.x = tcp_frame.p.x();
  response->current_pose.position.y = tcp_frame.p.y();
  response->current_pose.position.z = tcp_frame.p.z();

  // Obtener orientación en quaternios
  double qx, qy, qz, qw;
  tcp_frame.M.GetQuaternion(qx, qy, qz, qw);
  response->current_pose.orientation.x = qx;
  response->current_pose.orientation.y = qy;
  response->current_pose.orientation.z = qz;
  response->current_pose.orientation.w = qw;
}





int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KdlJointToCartesian>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
