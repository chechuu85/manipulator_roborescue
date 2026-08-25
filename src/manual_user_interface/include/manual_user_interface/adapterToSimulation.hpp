#ifndef ADAPTER_TO_SIMULATION_HPP
#define ADAPTER_TO_SIMULATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "manipulator_msgs/msg/hiper_joint_state.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// Enumeración para la máquina de estados
enum class RobotMode {
    MANUAL,
    EXECUTING
};

class AdapterToSimulationNode : public rclcpp::Node {
public:
    // Constructor y Destructor
    AdapterToSimulationNode();
    ~AdapterToSimulationNode();

    // Temporización para el timer_
    int timer_period_ms = 0; // (ms)
private:
    void callback(const manipulator_msgs::msg::HiperJointState::SharedPtr msg);
    void timer_callback();

    trajectory_msgs::msg::JointTrajectoryPoint create_trajectory_point(
        const manipulator_msgs::msg::HiperJointState::SharedPtr msg);

    rclcpp::Subscription<manipulator_msgs::msg::HiperJointState>::SharedPtr sub_articular_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables para calcular la posición
    std::vector<double> posiciones_actuales_ = std::vector<double>(8, 0.0);
    std::vector<double> velocidades_actuales_ = std::vector<double>(8, 0.0);
    sensor_msgs::msg::JointState joint_state_msg_{};
    rclcpp::Time ultimo_tiempo_;

    // Máquina de estados y variables de trayectoria
    RobotMode current_mode_ = RobotMode::MANUAL;
    trajectory_msgs::msg::JointTrajectory trajectory_msg_;
    size_t trajectory_publish_index_ = 0;

};


#endif