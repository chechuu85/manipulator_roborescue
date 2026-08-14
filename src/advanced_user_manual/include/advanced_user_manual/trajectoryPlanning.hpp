#pragma once

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "manipulator_msgs/srv/get_current_pose.hpp"

// #include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <trajectory_msgs/msg/joint_trajectory_point.hpp>
// #include <fstream>
// #include <iomanip>
// #include <chrono>
// #include <ctime>
// #include <sstream>
// #include <cerrno>
// #include <cstring>
// #include <sys/stat.h>
// #include <sys/types.h>

// // KDL
// #include <kdl/tree.hpp>
// #include <kdl_parser/kdl_parser.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>
// #include <kdl/chainiksolverpos_nr.hpp>
// #include <kdl/jntarray.hpp>
// #include <ament_index_cpp/get_package_share_directory.hpp>
// #include <yaml-cpp/yaml.h>

// // Eigen
// #include <Eigen/Dense>

// #include <memory>
// #include <string>

class TrajectoryPlanningNode : public rclcpp::Node
{
public:
    TrajectoryPlanningNode();
    ~TrajectoryPlanningNode();

private:

    void keyboardCallback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_instr_trayectory;
    rclcpp::Client<manipulator_msgs::srv::GetCurrentPose>::SharedPtr client_pose;




    

    // // Métodos principales
    // void initialize_kinematics();
    // void load_poses_from_yaml();
    // void generate_and_publish_trajectory();

    // // Atributos de ROS 2

    // rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    // rclcpp::TimerBase::SharedPtr timer_;

    // // Variables de Cinemática (KDL)
    // KDL::Chain chain_;
    // std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    // std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
    // std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;

    // // Poses cartesianas extraídas del YAML
    // Eigen::Matrix4d pose0_;
    // Eigen::Matrix4d pose1_;
    // Eigen::Matrix4d pose2_;
};