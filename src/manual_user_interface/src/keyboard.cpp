#include "manual_user_interface/keyboard.hpp"


// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================
KeyboardNode::KeyboardNode() : Node("keyboard_node") {
    // Inicializar subsistema de eventos de SDL y SDL_ttf
    if (SDL_Init(SDL_INIT_EVENTS | SDL_INIT_VIDEO) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error SDL: %s", SDL_GetError());
    }
    if (TTF_Init() == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error SDL_ttf: %s", TTF_GetError());
    }

    // Obtener el valor del parámetro por el launch
    this->declare_parameter<int>("timer_period_ms", 20);
    this->get_parameter("timer_period_ms", timer_period_ms);

    // Crear una ventana para capturar eventos de teclado y comprobar que que se crea y renderizar para mostrar mensajes
    window_ = SDL_CreateWindow("Panel de Control - Brazo Manipulador",
                                SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                700, 600, SDL_WINDOW_SHOWN);
    if (window_ == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo crear la ventana SDL: %s", SDL_GetError());
    }
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);

    // Cargar una fuente del sistema (Rutas estándar en Ubuntu)
    font_ = TTF_OpenFont("/usr/share/fonts/truetype/ubuntu/Ubuntu-B.ttf", 16);
    if (!font_) {
        // Fallback a DejaVu si no existe Ubuntu Font
        font_ = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16);
    }
    if (!font_) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo encontrar fuente de texto");
    }

    // Configurar el publicador y el temporizador 
    publisher_articular_ = this->create_publisher<sensor_msgs::msg::JointState>("input_articular", 10);
    publisher_cartesian_ = this->create_publisher<manipulator_msgs::msg::HiperTwist>("input_cartesian", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms), 
             std::bind(&KeyboardNode::timer_callback, this));
    
    flag_q = flag_a = flag_t = flag_g = flag_m = flag_b = false;

    // Ajustamos el tamaño a 8 elementos de velocidad
    msg_articular_.velocity.resize(8, 0.0); 

    RCLCPP_INFO(this->get_logger(), "Nodo de teclado inicializado. Comienza en espacio articular. Al cambiar al estado espacio cartesiano, el marco de referencia es la base del robot" );

}

KeyboardNode::~KeyboardNode() { 

    // Liberamos la memoria de la ventana de forma segura y apagamos las librerías SDL y SDL_ttf
    if (font_) TTF_CloseFont(font_);
    if (renderer_) SDL_DestroyRenderer(renderer_);
    if (window_ != nullptr) {
        SDL_DestroyWindow(window_); 
    }
    TTF_Quit();
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
            // RCLCPP_INFO(this->get_logger(), "Modo cambiado a: %s", cartesian_mode_ ? "CARTESIANO" : "ARTICULAR");
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

    // Dibujar la interfaz en cada iteracion del timer
    render_ui();
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
        if(!flag_g) { ref_vel_dinamixel -= 2.0f; flag_g = true; } 
    } else flag_g = false;

    // Mapeo de columnas para motores. Rozum (3 columnas W/S, E/D, R/F)
    msg_articular_.velocity[0] = state[SDL_SCANCODE_W] ? ref_vel_rozum : (state[SDL_SCANCODE_S] ? -ref_vel_rozum : 0);
    msg_articular_.velocity[1] = state[SDL_SCANCODE_E] ? ref_vel_rozum : (state[SDL_SCANCODE_D] ? -ref_vel_rozum : 0);
    msg_articular_.velocity[2] = state[SDL_SCANCODE_R] ? ref_vel_rozum : (state[SDL_SCANCODE_F] ? -ref_vel_rozum : 0);

    // Asignación directa a tus 5 motores Dinamixel(teclas I/K, Y/H, U/J, O,L , P/Ñ))
    msg_articular_.velocity[3] = state[SDL_SCANCODE_Y] ? ref_vel_dinamixel : (state[SDL_SCANCODE_H] ? -ref_vel_dinamixel : 0);
    msg_articular_.velocity[4] = state[SDL_SCANCODE_U] ? ref_vel_dinamixel : (state[SDL_SCANCODE_J] ? -ref_vel_dinamixel : 0);
    msg_articular_.velocity[5] = state[SDL_SCANCODE_I] ? ref_vel_dinamixel : (state[SDL_SCANCODE_K] ? -ref_vel_dinamixel : 0);
    msg_articular_.velocity[6] = state[SDL_SCANCODE_O] ? ref_vel_dinamixel : (state[SDL_SCANCODE_L] ? -ref_vel_dinamixel : 0);
    msg_articular_.velocity[7] = state[SDL_SCANCODE_P] ? ref_vel_dinamixel : (state[SDL_SCANCODE_SEMICOLON] ? -ref_vel_dinamixel : 0);
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
    msg_cartesian_.twist_command.linear.x = state[SDL_SCANCODE_W] ? ref_vel_cartesian : (state[SDL_SCANCODE_S] ? -ref_vel_cartesian : 0.0);
    msg_cartesian_.twist_command.linear.y = state[SDL_SCANCODE_A] ? ref_vel_cartesian : (state[SDL_SCANCODE_D] ? -ref_vel_cartesian : 0.0);
    msg_cartesian_.twist_command.linear.z = state[SDL_SCANCODE_Z] ? ref_vel_cartesian : (state[SDL_SCANCODE_X] ? -ref_vel_cartesian : 0.0);

    // Yaw (Rotar Z): Teclas U / J -- // Pitch (Rotar Y): Teclas I / K -- Roll (Rotar X): Teclas O / L
    msg_cartesian_.twist_command.angular.z = state[SDL_SCANCODE_U] ? ref_vel_cartesian : (state[SDL_SCANCODE_J] ? -ref_vel_cartesian : 0.0);
    msg_cartesian_.twist_command.angular.y = state[SDL_SCANCODE_I] ? ref_vel_cartesian : (state[SDL_SCANCODE_K] ? -ref_vel_cartesian : 0.0);
    msg_cartesian_.twist_command.angular.x = state[SDL_SCANCODE_O] ? ref_vel_cartesian : (state[SDL_SCANCODE_L] ? -ref_vel_cartesian : 0.0);

    // Garra (Abrir/Cerrar: Teclas C / V -- 
    msg_cartesian_.gripper = state[SDL_SCANCODE_C] ? ref_vel_cartesian : (state[SDL_SCANCODE_V] ? -ref_vel_cartesian : 0.0);

    if (state[SDL_SCANCODE_B]) {
        if (!flag_b) {
            referencia_base_ = !referencia_base_;
            flag_b = true;
            // RCLCPP_INFO(this->get_logger(), "Lugar referencia a: %s", referencia_base_ ? "BASE" : "TCP");
        }
    } else {
        flag_b = false;
    }

    // Seleccionar lugar referencia
    if (referencia_base_) {
        msg_cartesian_.command_info = "BASE";
    } else {
        msg_cartesian_.command_info = "TCP";
    }
}


// ==========================================
// RENDERIZADO DE INTERFAZ GRÁFICA
// ==========================================
void KeyboardNode::render_text(const std::string &text, int x, int y, SDL_Color color) {
    // Se pone color
    if (!font_) return;
    SDL_Surface* surface = TTF_RenderUTF8_Blended(font_, text.c_str(), color);

    // Se pone superficie
    if (!surface) return;
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);

    SDL_Rect dest = {x, y, surface->w, surface->h};
    SDL_RenderCopy(renderer_, texture, NULL, &dest);

    SDL_FreeSurface(surface);
    SDL_DestroyTexture(texture);
}

void KeyboardNode::render_ui() {
    if (!renderer_) return;

    // Fondo gris oscuro
    SDL_SetRenderDrawColor(renderer_, 30, 30, 35, 255);
    SDL_RenderClear(renderer_);

    SDL_Color c_white = {255, 255, 255, 255};
    SDL_Color c_green = {100, 255, 100, 255};
    SDL_Color c_cyan = {100, 200, 255, 255};
    SDL_Color c_yellow = {255, 220, 100, 255};
    SDL_Color c_gray = {170, 170, 170, 255};

    int y = 20;
    int col = 30;

    render_text("=== PANEL DE CONTROL DEL BRAZO ROBOT ===", col, y, c_cyan); y += 40;

    std::string modo_txt = cartesian_mode_ ? "MODO ACTUAL: CARTESIANO" : "MODO ACTUAL: ARTICULAR";
    render_text(modo_txt, col, y, c_green); y += 40;

    render_text("[M] Cambiar Modo | [B] Cambiar Ref. (Base / TCP)", col, y, c_white); y +=40;

    if (!cartesian_mode_) {
        render_text("--- CONTROLES MODO ARTICULAR ---", col, y, c_yellow); y += 30;
        render_text("Velocidades: [Q/A] Rozum | [T/G] Dinamixel", col, y, c_gray); y += 35;
        render_text("Articulaciones Rozum (Base a Codo):", col, y, c_white); y += 25;
        render_text(" [W/S] Joint 1 (Base)", col, y, c_white); y += 25;
        render_text(" [E/D] Joint 2", col, y, c_white); y += 25;
        render_text(" [R/F] Joint 3", col, y, c_white); y += 35;
        render_text("Articulaciones Dinamixel (Muñeca y Garra):", col, y, c_white); y += 25;
        render_text(" [Y/H] Joint 4 | [U/J] Joint 5 | [I/K] Joint 6", col, y, c_white); y += 25;
        render_text(" [O/L] Dedo Izquierdo | [P/ ;] Dedo Derecho", col, y, c_white); y += 25;
    } else {
        render_text("--- CONTROLES MODO CARTESIANO ---", col, y, c_yellow); y += 30;
        std::string ref_str = referencia_base_ ? "Referencia actual: BASE" : "Referencia actual: TCP";
        render_text(ref_str, col, y, c_gray); y += 35;
        render_text("Velocidad Cartesiana: [T/G] (+/-)", col, y, c_gray); y += 35;
        render_text("Traslacion (XYZ):", col, y, c_white); y += 25;
        render_text(" [W/S] X (Avance) | [A/D] Y (Lateral) | [Z/X] Z (Vertical)", col, y, c_white); y += 35;
        render_text("Rotacion (Roll, Pitch, Yaw):", col, y, c_white); y += 25;
        render_text(" [O/L] Roll (X) | [I/K] Pitch (Y) | [U/J] Yaw (Z)", col, y, c_white); y += 35;
        render_text("Herramienta:", col, y, c_white); y += 25;
        render_text(" [C/V] Abrir / Cerrar Garra", col, y, c_white); y += 25;
    }
    SDL_RenderPresent(renderer_);
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