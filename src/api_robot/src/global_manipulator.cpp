#include "API_robot/global_manipulator.hpp"
#include <iostream>

// ==========================================
// CONSTRUCTOR Y DESTRUCTOR
// ==========================================

GlobalManipulator::GlobalManipulator(rr_can_interface_t* rozum_iface, dynamixel::PortHandler* port, dynamixel::PacketHandler* packet) 
    : Node("manipulator_node"){
    // Inicializamos las instancias de los conjuntos de motores con sus IDs por defecto
    arm = new RozumArm(rozum_iface, 123, 124, 125); // IDs estándar de Rozum en tu código
    claw = new DynamixelClaw(port, packet, 1, 2, 3, 5, 12); // IDs estándar de Dynamixel en tu código

    // Se define un timer de 50ms (20Hz) y publicador
    telemetry_pub_ = this->create_publisher<manipulator_msgs::msg::ManipulatorMotorStage>("manipulator_telemetry", 10);
    telemetry_timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms)
        , std::bind(&GlobalManipulator::publish_telemetry_callback, this));
}

GlobalManipulator::~GlobalManipulator() {
    deinit();
    delete arm;
    delete claw;
}

// ==========================================
// INICIALIZACIÓN Y GESTIÓN DE HILOS
// ==========================================

void GlobalManipulator::init() {
    // Activar brazo Rozum
    arm->activate_all();
    arm->setup_telemetry_cache_all();

    // Activar garra Dynamixel
    claw->set_mode_all(VELOCITY_MODE); // O POSITION_MODE según necesidad
    claw->set_torque_all(true);
    claw->set_limits_all();

    // Iniciar el gestor de tareas en un hilo separado
    hilo_dynamixel = std::thread(&GlobalManipulator::gestor_tareas, this);
}

void GlobalManipulator::deinit() {
    // Avisar al hilo para que se apague
    {
        std::lock_guard<std::mutex> lock(mtx_sincronizacion);
        comando_actual_ = tarea_dynamixel::POWER_OFF;
    }
    cv_iniciar_tarea_.notify_one();
    
    // Esperar a que el hilo termine[cite: 2]
    if (hilo_dynamixel.joinable()) {
        hilo_dynamixel.join();
    }

    // Apagar torque de la garra por seguridad
    claw->set_torque_all(false);
}

void GlobalManipulator::gestor_tareas() {
    // Máquina de estados en bucle para el hilo de Dynamixel
    while (true) {
        std::unique_lock<std::mutex> lock(mtx_sincronizacion);
        
        // 1. DORMIR hasta que el comando sea distinto de 'SLEEPING'
        cv_iniciar_tarea_.wait(lock, [this]{ 
            return comando_actual_ != tarea_dynamixel::SLEEPING; 
        });
        
        // Si se apagan los motores, rompemos el bucle infinito
        if (comando_actual_ == tarea_dynamixel::POWER_OFF) break;

        // 2. DESPERTAR Y TRABAJAR: Soltamos el candado para no bloquear el hilo principal
        tarea_dynamixel tarea_a_ejecutar = comando_actual_;
        lock.unlock(); 
        
        // --- LA MÁQUINA DE ESTADOS (Solo afecta a la garra Dynamixel) ---
        try {
            switch (tarea_a_ejecutar) {
                case tarea_dynamixel::READ_POSITION:
                    claw->read_positions();
                    break;
                case tarea_dynamixel::READ_VELOCITY:
                    claw->read_velocities();
                    break;
                case tarea_dynamixel::READ_TEMPERATURE:
                    claw->read_temperatures();
                    break;
                case tarea_dynamixel::READ_CURRENT:
                    claw->read_currents();
                    break;
                case tarea_dynamixel::SEND_VELOCITY:
                    // La velocidad ya se guardó en claw->motorX.actuation_motor.velocity antes de llamar aquí
                    // Por lo que podemos disparar el sync write extrayéndolas directamente.
                    claw->set_velocities({claw->motor1.actuation_motor.velocity, claw->motor2.actuation_motor.velocity, 
                                          claw->motor3.actuation_motor.velocity, claw->motor5.actuation_motor.velocity, claw->motor12.actuation_motor.velocity});
                    break;
                case tarea_dynamixel::SEND_POSITION:
                    claw->set_positions({claw->motor1.actuation_motor.position, claw->motor2.actuation_motor.position, 
                                         claw->motor3.actuation_motor.position, claw->motor5.actuation_motor.position, claw->motor12.actuation_motor.position});
                    break;
                default:
                    break;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error en gestor_tareas (Dynamixel): " << e.what() << std::endl;
        }
        
        // 3. AVISAR: Volvemos a coger el candado e informamos que terminamos
        lock.lock();
        comando_actual_ = tarea_dynamixel::SLEEPING; // Volvemos al estado base
        tarea_completada_ = true;
        cv_tarea_terminada_.notify_one();
    }
}

// ==========================================
// FUNCIONES DE CONTROL PARALELO
// ==========================================

void GlobalManipulator::read_positions() {
    // 1. Notificar al hilo secundario (Garra)
    {
        std::lock_guard<std::mutex> lock(mtx_sincronizacion);
        comando_actual_ = tarea_dynamixel::READ_POSITION;
        tarea_completada_ = false;
    }
    cv_iniciar_tarea_.notify_one();

    // 2. Ejecutar la tarea del brazo en el hilo principal (Rozum)
    arm->read_positions();

    // 3. Esperar a que la garra termine para continuar sincronizados
    {
        std::unique_lock<std::mutex> lock(mtx_sincronizacion);
        cv_tarea_terminada_.wait(lock, [this]{ return tarea_completada_; });
    }
}

void GlobalManipulator::read_velocities() {
    {
        std::lock_guard<std::mutex> lock(mtx_sincronizacion);
        comando_actual_ = tarea_dynamixel::READ_VELOCITY;
        tarea_completada_ = false;
    }
    cv_iniciar_tarea_.notify_one();

    arm->read_velocities(); // Lectura síncrona Rozum

    {
        std::unique_lock<std::mutex> lock(mtx_sincronizacion);
        cv_tarea_terminada_.wait(lock, [this]{ return tarea_completada_; });
    }
}

void GlobalManipulator::read_currents() {
    {
        std::lock_guard<std::mutex> lock(mtx_sincronizacion);
        comando_actual_ = tarea_dynamixel::READ_CURRENT;
        tarea_completada_ = false;
    }
    cv_iniciar_tarea_.notify_one();

    arm->read_currents(); 

    {
        std::unique_lock<std::mutex> lock(mtx_sincronizacion);
        cv_tarea_terminada_.wait(lock, [this]{ return tarea_completada_; });
    }
}

void GlobalManipulator::read_temperatures() {
    {
        std::lock_guard<std::mutex> lock(mtx_sincronizacion);
        comando_actual_ = tarea_dynamixel::READ_TEMPERATURE;
        tarea_completada_ = false;
    }
    cv_iniciar_tarea_.notify_one();

    arm->read_temperatures(); 

    {
        std::unique_lock<std::mutex> lock(mtx_sincronizacion);
        cv_tarea_terminada_.wait(lock, [this]{ return tarea_completada_; });
    }
}

void GlobalManipulator::set_velocities(const std::array<float, 3>& arm_vels, const std::array<float, 5>& claw_vels) {
    // Precargamos los valores de actuación para la garra antes de disparar el hilo
    claw->motor1.actuation_motor.velocity = claw_vels[0];
    claw->motor2.actuation_motor.velocity = claw_vels[1];
    claw->motor3.actuation_motor.velocity = claw_vels[2];
    claw->motor5.actuation_motor.velocity = claw_vels[3];
    claw->motor12.actuation_motor.velocity = claw_vels[4];

    // Despertamos el hilo de transmisión
    {
        std::lock_guard<std::mutex> lock(mtx_sincronizacion);
        comando_actual_ = tarea_dynamixel::SEND_VELOCITY;
        tarea_completada_ = false;
    }
    cv_iniciar_tarea_.notify_one();

    // Transmitimos a Rozum
    arm->set_velocities(arm_vels);

    // Esperamos a Dynamixel
    {
        std::unique_lock<std::mutex> lock(mtx_sincronizacion);
        cv_tarea_terminada_.wait(lock, [this]{ return tarea_completada_; });
    }
}

void GlobalManipulator::set_positions(const std::array<float, 3>& arm_pos, const std::array<float, 5>& claw_pos) {
    // Precargamos los valores de actuación para la garra
    claw->motor1.actuation_motor.position = claw_pos[0];
    claw->motor2.actuation_motor.position = claw_pos[1];
    claw->motor3.actuation_motor.position = claw_pos[2];
    claw->motor5.actuation_motor.position = claw_pos[3];
    claw->motor12.actuation_motor.position = claw_pos[4];

    {
        std::lock_guard<std::mutex> lock(mtx_sincronizacion);
        comando_actual_ = tarea_dynamixel::SEND_POSITION;
        tarea_completada_ = false;
    }
    cv_iniciar_tarea_.notify_one();

    arm->set_positions(arm_pos);

    {
        std::unique_lock<std::mutex> lock(mtx_sincronizacion);
        cv_tarea_terminada_.wait(lock, [this]{ return tarea_completada_; });
    }
}


// ==========================================
// FUNCIONES ROS2
// ==========================================

void GlobalManipulator::publish_telemetry_callback() {
    // 1. Leer los datos más recientes de los motores mediante las funciones de sincronización
    read_positions();
    read_velocities();
    read_currents();
    read_temperatures();

    // 2. Crear el mensaje de ROS2
    auto msg = manipulator_msgs::msg::ManipulatorMotorStage();

    // 3. Extraer telemetría de Rozum (Brazo)
    // Motor 1 (ID 123)
    msg.rozum_motors[0].position = arm->motor1.telemetry_motor.position;
    msg.rozum_motors[0].velocity = arm->motor1.telemetry_motor.velocity;
    msg.rozum_motors[0].current = arm->motor1.telemetry_motor.current;
    msg.rozum_motors[0].temperature = arm->motor1.telemetry_motor.temperature;
    
    // Motor 2 (ID 124)
    msg.rozum_motors[1].position = arm->motor2.telemetry_motor.position;
    msg.rozum_motors[1].velocity = arm->motor2.telemetry_motor.velocity;
    msg.rozum_motors[1].current = arm->motor2.telemetry_motor.current;
    msg.rozum_motors[1].temperature = arm->motor2.telemetry_motor.temperature;

    // Motor 3 (ID 125)
    msg.rozum_motors[2].position = arm->motor3.telemetry_motor.position;
    msg.rozum_motors[2].velocity = arm->motor3.telemetry_motor.velocity;
    msg.rozum_motors[2].current = arm->motor3.telemetry_motor.current;
    msg.rozum_motors[2].temperature = arm->motor3.telemetry_motor.temperature;

    // 4. Extraer telemetría de Dynamixel (Garra)
    // Motor 1 (Dynamixel ID 1)
    msg.dinamixel_motors[0].position = claw->motor1.telemetry_motor.position;
    msg.dinamixel_motors[0].velocity = claw->motor1.telemetry_motor.velocity;
    msg.dinamixel_motors[0].current = claw->motor1.telemetry_motor.current;
    msg.dinamixel_motors[0].temperature = claw->motor1.telemetry_motor.temperature;

    // Motor 2 (Dynamixel ID 2)
    msg.dinamixel_motors[1].position = claw->motor2.telemetry_motor.position;
    msg.dinamixel_motors[1].velocity = claw->motor2.telemetry_motor.velocity;
    msg.dinamixel_motors[1].current = claw->motor2.telemetry_motor.current;
    msg.dinamixel_motors[1].temperature = claw->motor2.telemetry_motor.temperature;

    // Motor 3 (Dynamixel ID 3)
    msg.dinamixel_motors[2].position = claw->motor3.telemetry_motor.position;
    msg.dinamixel_motors[2].velocity = claw->motor3.telemetry_motor.velocity;
    msg.dinamixel_motors[2].current = claw->motor3.telemetry_motor.current;
    msg.dinamixel_motors[2].temperature = claw->motor3.telemetry_motor.temperature;

    // Motor 5 (Dynamixel ID 5)
    msg.dinamixel_motors[3].position = claw->motor5.telemetry_motor.position;
    msg.dinamixel_motors[3].velocity = claw->motor5.telemetry_motor.velocity;
    msg.dinamixel_motors[3].current = claw->motor5.telemetry_motor.current;
    msg.dinamixel_motors[3].temperature = claw->motor5.telemetry_motor.temperature;

    // Motor 12 (Dynamixel ID 12)
    msg.dinamixel_motors[4].position = claw->motor12.telemetry_motor.position;
    msg.dinamixel_motors[4].velocity = claw->motor12.telemetry_motor.velocity;
    msg.dinamixel_motors[4].current = claw->motor12.telemetry_motor.current;
    msg.dinamixel_motors[4].temperature = claw->motor12.telemetry_motor.temperature;

    // 5. Publicar el estado en el tópico
    telemetry_pub_->publish(msg);
}