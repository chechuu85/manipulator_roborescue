#ifndef ADAPTER_TO_SIMULATION_HPP
#define ADAPTER_TO_SIMULATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class AdapterToSimulationNode : public rclcpp::Node {
public:
    AdapterToSimulationNode();
    ~AdapterToSimulationNode();

    // Temporización para el timer_
    int timer_period_ms = 0; // (ms)
private:
    void callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void timer_callback();

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_articular_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables para calcular la posición
    std::vector<double> posiciones_actuales_ = std::vector<double>(8, 0.0);
    std::vector<double> velocidades_actuales_ = std::vector<double>(8, 0.0);
    sensor_msgs::msg::JointState joint_state_msg_{};
    rclcpp::Time ultimo_tiempo_;
};


#endif