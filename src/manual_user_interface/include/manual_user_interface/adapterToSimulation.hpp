#ifndef ADAPTER_TO_SIMULATION_HPP
#define ADAPTER_TO_SIMULATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "manipulator_msgs/msg/hiper_joint_state.hpp"
#include "manipulator_msgs/msg/manipulator_motor_stage.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// Enumeración para la máquina de estados
enum class RobotMode {
    MANUAL,
    TRAJECTORY,
    ROBOT_RECEAVING
};

class AdapterToSimulationNode : public rclcpp::Node {
public:
    // Constructor y Destructor
    AdapterToSimulationNode();
    ~AdapterToSimulationNode();

    // Temporización para el timer_
    int timer_period_ms = 0; // (ms)
    // Dice el bloque si comunica con simulación o con robot
    bool sim_mode = true; 
private:
    void callback_simul(const manipulator_msgs::msg::HiperJointState::SharedPtr msg);
    void callback_real(const manipulator_msgs::msg::ManipulatorMotorStage::SharedPtr msg);
    void timer_callback();

    rclcpp::Subscription<manipulator_msgs::msg::HiperJointState>::SharedPtr sub_articular_;
    rclcpp::Subscription<manipulator_msgs::msg::ManipulatorMotorStage>::SharedPtr sub_physic_robot_;
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