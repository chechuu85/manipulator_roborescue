#ifndef ADAPTER_TO_SIMULATION_HPP
#define ADAPTER_TO_SIMULATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "manipulator_msgs/msg/manipulator_motor_stage.hpp"

class AdapterToSimulationNode : public rclcpp::Node {
public:
    AdapterToSimulationNode();
    ~AdapterToSimulationNode();

private:
    void callback(const manipulator_msgs::msg::ManipulatorMotorStage::SharedPtr msg);

    rclcpp::Subscription<manipulator_msgs::msg::ManipulatorMotorStage>::SharedPtr sub_articular_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;

    // Variables para calcular la posición
    std::vector<double> posiciones_actuales_ = std::vector<double>(8, 0.0);
    rclcpp::Time ultimo_tiempo_;
};


#endif