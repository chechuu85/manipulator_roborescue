#include "user_interface/keyboard.hpp"


// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================
KeyboardNode::KeyboardNode() : Node("keyboard_node") {
    // Inicializar subsistema de eventos de SDL
    if (SDL_Init(SDL_INIT_EVENTS | SDL_INIT_VIDEO) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error SDL: %s", SDL_GetError());
    }

    // Crear una ventana para capturar eventos de teclado y comprobar que que se crea
    window_ = SDL_CreateWindow("KeyboardCapture", 
                                        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 
                                        100, 100, SDL_WINDOW_SHOWN);
    if (window_ == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo crear la ventana SDL: %s", SDL_GetError());
    }

    // Configurar el publicador y el temporizador 
    publisher_articular_ = this->create_publisher<manipulator_msgs::msg::ManipulatorMotorStage>("keyboard_articular", 10);
    publisher_cartesian_ = this->create_publisher<geometry_msgs::msg::Twist>("keyboard_cartesian", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms), 
             std::bind(&KeyboardNode::timer_callback, this));
    
    flag_q = flag_a = flag_t = flag_g = flag_m= false;

    RCLCPP_INFO(this->get_logger(), "Nodo de teclado inicializado. ");

}

KeyboardNode::~KeyboardNode() { 

    // Liberamos la memoria de la ventana de forma segura y apagamos la librería SDL
    if (window_ != nullptr) {
        SDL_DestroyWindow(window_); 
    }
    SDL_Quit();

    RCLCPP_INFO(this->get_logger(), "Nodo de teclado finalizado. ");
}


// ==========================================
// INTERRUPCIÓN
// ==========================================
void KeyboardNode::timer_callback() {
    // Procesa todos los eventos del hardware del PC (teclado, ratón, etc.) y lo guarda en state
    SDL_PumpEvents(); 
    const Uint8 *state = SDL_GetKeyboardState(NULL);

    // Lógica para alternar el modo con la tecla 'M'
    if (state[SDL_SCANCODE_M]) {
        if (!flag_m) {
            cartesian_mode_ = !cartesian_mode_;
            flag_m = true;
            RCLCPP_INFO(this->get_logger(), "Modo cambiado a: %s", 
                        cartesian_mode_ ? "CARTESIANO" : "ARTICULAR");
        }
    } else {
        flag_m = false;
    }

    // Seleccionar el modo de operación
    if (cartesian_mode_){
        cartesian_mode(state);
        publisher_cartesian_->publish(msg_cartesian_);
    }else{
        articular_mode(state);
        publisher_articular_->publish(msg_articular_);
    }
}


// ==========================================
// FUNCIONES LECTURA TECLADO 
// ==========================================
void KeyboardNode::articular_mode(const Uint8 *state){
    // Lógica de incremento/decremento (Q, A para Rozum; T, G para Dinamixel)
    if (state[SDL_SCANCODE_Q]) { 
        if(!flag_q) { ref_vel_rozum += 0.5f; flag_q = true;} 
    } else flag_q = false;
    if (state[SDL_SCANCODE_A]) { 
        if(!flag_a) { ref_vel_rozum -= 0.5f; flag_a = true; } 
    } else flag_a = false;
    
    if (state[SDL_SCANCODE_T]) { 
        if(!flag_t) { ref_vel_dinamixel += 2.0f; flag_t = true; } 
    } else flag_t = false;
    if (state[SDL_SCANCODE_G]) { 
        printf("G pressed\n");
        if(!flag_g) { ref_vel_dinamixel -= 2.0f; flag_g = true; } 
    } else flag_g = false;

    // Mapeo de columnas para motores. Rozum (3 columnas W/S, E/D, R/F)
    msg_articular_.rozum_motors[0].velocity = state[SDL_SCANCODE_W] ? ref_vel_rozum : (state[SDL_SCANCODE_S] ? -ref_vel_rozum : 0);
    msg_articular_.rozum_motors[1].velocity = state[SDL_SCANCODE_E] ? ref_vel_rozum : (state[SDL_SCANCODE_D] ? -ref_vel_rozum : 0);
    msg_articular_.rozum_motors[2].velocity = state[SDL_SCANCODE_R] ? ref_vel_rozum : (state[SDL_SCANCODE_F] ? -ref_vel_rozum : 0);

    // Asignación directa a tus 5 motores Dinamixel(teclas I/K, Y/H, U/J, O,L , P/Ñ))
    msg_articular_.dinamixel_motors[0].velocity = state[SDL_SCANCODE_Y] ? ref_vel_dinamixel : (state[SDL_SCANCODE_H] ? -ref_vel_dinamixel : 0);
    msg_articular_.dinamixel_motors[1].velocity = state[SDL_SCANCODE_U] ? ref_vel_dinamixel : (state[SDL_SCANCODE_J] ? -ref_vel_dinamixel : 0);
    msg_articular_.dinamixel_motors[2].velocity = state[SDL_SCANCODE_I] ? ref_vel_dinamixel : (state[SDL_SCANCODE_K] ? -ref_vel_dinamixel : 0);
    msg_articular_.dinamixel_motors[3].velocity = state[SDL_SCANCODE_O] ? ref_vel_dinamixel : (state[SDL_SCANCODE_L] ? -ref_vel_dinamixel : 0);
    msg_articular_.dinamixel_motors[4].velocity = state[SDL_SCANCODE_P] ? ref_vel_dinamixel : (state[SDL_SCANCODE_SEMICOLON] ? -ref_vel_dinamixel : 0);
                                                                            // Se usa ; en lugar de la Ñ por compatibilidad de teclado
}

void KeyboardNode::cartesian_mode(const Uint8 *state){
    // Control de velocidad cartesiana global (usamos T / G para aumentar/disminuir)
    if (state[SDL_SCANCODE_T]) { 
        if(!flag_t) { ref_vel_cartesian += 0.1f; flag_t = true;} 
    } else flag_t = false;
    if (state[SDL_SCANCODE_G]) { 
        if(!flag_g) { ref_vel_cartesian -= 0.1f; flag_g = true; } 
    } else flag_g = false;

    // Eje X (Avanzar/Retroceder): Teclas W / S -- Eje Y (Izquierda/Derecha): Teclas A / D -- Eje Z (Arriba/Abajo): Teclas Z / X
    msg_cartesian_.linear.x = state[SDL_SCANCODE_W] ? ref_vel_cartesian : (state[SDL_SCANCODE_S] ? -ref_vel_cartesian : 0.0);
    msg_cartesian_.linear.y = state[SDL_SCANCODE_A] ? ref_vel_cartesian : (state[SDL_SCANCODE_D] ? -ref_vel_cartesian : 0.0);
    msg_cartesian_.linear.z = state[SDL_SCANCODE_Z] ? ref_vel_cartesian : (state[SDL_SCANCODE_X] ? -ref_vel_cartesian : 0.0);

    // Ejes angulares (Orientación del end-effector)
    // Yaw (Rotar Z): Teclas U / J -- // Pitch (Rotar Y): Teclas I / K -- Roll (Rotar X): Teclas O / L
    msg_cartesian_.angular.z = state[SDL_SCANCODE_U] ? ref_vel_cartesian : (state[SDL_SCANCODE_J] ? -ref_vel_cartesian : 0.0);
    msg_cartesian_.angular.y = state[SDL_SCANCODE_I] ? ref_vel_cartesian : (state[SDL_SCANCODE_K] ? -ref_vel_cartesian : 0.0);
    msg_cartesian_.angular.x = state[SDL_SCANCODE_O] ? ref_vel_cartesian : (state[SDL_SCANCODE_L] ? -ref_vel_cartesian : 0.0);

    // Publicar mensaje cartesiano
    publisher_cartesian_->publish(msg_cartesian_);
}


// ==========================================
// MAIN
// ==========================================
int main(int argc, char * argv[]) {
    // Inicializar ROS 2 y crear el nodo
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardNode>();

    // Ejecutar el nodo hasta que se reciba una interrupción
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}