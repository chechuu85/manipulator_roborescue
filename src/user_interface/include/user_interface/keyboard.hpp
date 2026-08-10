#ifndef KEYBOARD_NODE_HPP
#define KEYBOARD_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "manipulator_msgs/msg/hiper_twist.hpp"
#include <SDL2/SDL.h>

class KeyboardNode : public rclcpp::Node {
public:
    KeyboardNode();
    ~KeyboardNode();

    // Temporización para el timer_
    int timer_period_ms = 0; // (ms)

private:
    void timer_callback();
    void cartesian_mode(const Uint8 *state);
    void articular_mode(const Uint8 *state);

    // Publicador y temporizador 
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_articular_;
    rclcpp::Publisher<manipulator_msgs::msg::HiperTwist>::SharedPtr publisher_cartesian_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Ventana SDL para capturar teclado 
    SDL_Window* window_;
    
    // Variables de control
    float ref_vel_rozum = 0.0f;
    float ref_vel_dinamixel = 0.0f;
    float ref_vel_cartesian = 0.5f;
    bool flag_q, flag_a, flag_t, flag_g, flag_m, flag_b; // Flags para evitar múltiples cambios por pulsación de tecla 
    bool cartesian_mode_ = false;  // false = articular, true = cartesian
    bool referencia_base_ = true; // true = base, false = TCP

    // Variables para publicar mensajes
    sensor_msgs::msg::JointState msg_articular_{};
    manipulator_msgs::msg::HiperTwist msg_cartesian_{};
};

#endif