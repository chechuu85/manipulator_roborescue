#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "manipulator_msgs/srv/get_current_pose.hpp"
#include "manipulator_msgs/msg/hiper_pose.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class TrajectoryPlanningNode : public rclcpp::Node
{
public:
    TrajectoryPlanningNode();
    ~TrajectoryPlanningNode();

private:

    void keyboardCallback(const std_msgs::msg::String::SharedPtr msg);
    void timer_play_callback();

    rclcpp::TimerBase::SharedPtr timer_play_;
    std::vector<Eigen::Matrix4d> trajectory_poses;

    // Variables bucle trayectoria
    double t_max_ = 10.0; // cuanto tiempo se quiere que dure la trayectoria
    double t_current_ = -t_max_; // cuando comienza bucle 
    double sample_time_; // en saltos de cuanto se realiza la acción
    int tau_ = 1; // cuando se quiere que empiece a cambiar de punto

    YAML::Node poses_root;
    std::string poses_yaml_path;
    std::string key;
    uint8_t num_pose;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_instr_trayectory;
    rclcpp::Client<manipulator_msgs::srv::GetCurrentPose>::SharedPtr client_pose;
    rclcpp::Publisher<manipulator_msgs::msg::HiperPose>::SharedPtr pub_planning_pose_;
    
};