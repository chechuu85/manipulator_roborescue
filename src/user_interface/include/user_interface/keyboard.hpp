#ifndef KEYBOARD_NODE_HPP
#define KEYBOARD_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "manipulator_msgs/msg/manipulator_motor_stage.hpp"
#include <SDL2/SDL.h>

class KeyboardNode : public rclcpp::Node {
public:
    KeyboardNode();
    ~KeyboardNode();

    // Temporización para el timer_
    uint8_t timer_period_ms = 20; // (ms)

private:
    void timer_callback();

    // Publicador y temporizador 
    rclcpp::Publisher<manipulator_msgs::msg::ManipulatorMotorStage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Ventana SDL para capturar teclado 
    SDL_Window* window_;
    
    // Variables de control
    float ref_vel_rozum = 0.0f;
    float ref_vel_dinamixel = 0.0f;
    bool flag_q, flag_a, flag_t, flag_g;
};

#endif