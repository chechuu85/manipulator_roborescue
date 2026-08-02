#ifndef KEYBOARD_NODE_HPP
#define KEYBOARD_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "manipulator_msgs/msg/manipulator_motor_stage.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <SDL2/SDL.h>

class KeyboardNode : public rclcpp::Node {
public:
    KeyboardNode();
    ~KeyboardNode();

    // Temporización para el timer_
    uint8_t timer_period_ms = 20; // (ms)

private:
    void timer_callback();
    void cartesian_mode(const Uint8 *state);
    void articular_mode(const Uint8 *state);

    // Publicador y temporizador 
    rclcpp::Publisher<manipulator_msgs::msg::ManipulatorMotorStage>::SharedPtr publisher_articular_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cartesian_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Ventana SDL para capturar teclado 
    SDL_Window* window_;
    
    // Variables de control
    float ref_vel_rozum = 0.0f;
    float ref_vel_dinamixel = 0.0f;
    float ref_vel_cartesian = 0.5f;
    bool flag_q, flag_a, flag_t, flag_g, flag_m; // Flags para evitar múltiples cambios por pulsación de tecla 
    bool cartesian_mode_ = false; // false = articular, true = cartesian

    // Variables para publicar mensajes
    manipulator_msgs::msg::ManipulatorMotorStage msg_articular_{};
    geometry_msgs::msg::Twist msg_cartesian_{};
};

#endif