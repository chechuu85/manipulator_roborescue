#ifndef KDL_JOINT_TO_CARTESIAN_HPP_
#define KDL_JOINT_TO_CARTESIAN_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "geometry_msgs/msg/pose.hpp"
#include "manipulator_msgs/srv/get_current_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

class KdlJointToCartesian : public rclcpp::Node
{
public:
  KdlJointToCartesian();
  ~KdlJointToCartesian();

private:
  bool initKdl();
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void handleCurrentPose(
            const std::shared_ptr<rmw_request_id_t>request_header,
            const std::shared_ptr<manipulator_msgs::srv::GetCurrentPose::Request>request,
            std::shared_ptr<manipulator_msgs::srv::GetCurrentPose::Response>response);
            // Opcion para que no salga el warning:
            // (void)request_header;
            // (void)request;

  // Variables calcular cadena 
  std::vector<std::string> joint_names_;
  KDL::Chain chain_;
  KDL::JntArray q_current_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  // Subscriptor y servicio
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Service<manipulator_msgs::srv::GetCurrentPose>::SharedPtr current_pose_service_;
};

#endif  // KDL_JOINT_TO_CARTESIAN_HPP_
