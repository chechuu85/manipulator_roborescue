import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    #===========================================================
    # DECLARAR LOS ARGUMENTOS DE LANZAMIENTO
    # ==========================================================
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='Activa el modo simulación (true) o hardware real (false). Activa unos nodos u otros '
    )

    sampling_rate_arg = DeclareLaunchArgument(
        'sampling_rate',
        default_value='20',
        description='Tasa de muestreo en ms para los temporizadores'
    )

    input_type_arg = DeclareLaunchArgument(
        'input_type',
        default_value='keyboard', # Teclado por defecto
        description='Elige el controlador: "keyboard" o "joystick"'
    )

    # Crear variables de configuración que referencian los argumentos en tiempo de ejecución
    sim_mode = LaunchConfiguration('sim_mode')
    sampling_rate = LaunchConfiguration('sampling_rate')
    input_type = LaunchConfiguration('input_type')


    #===========================================================
    # URDF & ROBOT STATE PUBLISHER
    # ==========================================================
    # Obtener la ruta del archivo URDF
    urdf_path = os.path.join(
        get_package_share_directory("bringup"),
        "description",
        "manipulador_modu.urdf.xacro"
    )
    
    # Ejecutar Xacro para procesar el archivo y convertirlo en string
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # Nodo que publica el modelo 3D en el tópico /robot_description
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description,                  # Lee toda cadena cinemática del robot del archivo .xacro
                    #{"publish_frequency": 100.0}       Forzar la publicación TF a 100 Hz (en realidad publica a 50Hz)
                ],
    )

  
    #===========================================================
    # ADAPTADOR SIMULADOR FOXGLOVE
    # ==========================================================
    adapterToSimulation_node = Node(
        package="manual_user_interface",  
        executable="adapterToSimulation",     
        name="adapterToSimulation",      
        output="screen",
        condition=IfCondition(sim_mode), 
        parameters=[{'timer_period_ms': sampling_rate}] 
    )

    
    #===========================================================
    # KEYBOARD (C++ SDL2 Node)
    # ==========================================================
    keyboard_input_node = Node(
        package="manual_user_interface",  
        executable="keyboard",     
        name="keyboard",      
        output="screen",
        parameters=[{'timer_period_ms': sampling_rate}] 
    )

    #===========================================================
    # KDL CARTESIAN TO JOINT NODE
    # ==========================================================
    kdl_ik_node = Node(
        package="manual_user_interface",  
        executable="kdlCartesianToJoint",     
        name="kdl_ik_node",      
        output="screen",
        condition=IfCondition(PythonExpression(["'", input_type, "' == 'keyboard'"])),
        parameters=[robot_description]  
    )

    #===========================================================
    # KDL JOINT TO CARTESIAN NODE
    # ==========================================================
    kdl_fk_node = Node(
        package="manual_user_interface",  
        executable="kdlJointToCartesian",     
        name="kdl_fk_node",      
        output="screen",
        parameters=[robot_description]  
    )

    #===========================================================
    # TRAJECTORY PLANNING NODE
    # ==========================================================
    trajectory_planning_node = Node(
        package="advanced_user_manual",  
        executable="trajectoryPlanning",     
        name="trajectory_planning_node",      
        output="screen",
    )

    #===========================================================
    # FOXGLOVE BRIDGE
    # ==========================================================
    foxglove_bridge_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml"
            )
        )
    )

    #===========================================================
    # LAUNCH DESCRIPTION
    # ==========================================================
    return LaunchDescription([
        # Añadir las declaraciones aquí
        sim_mode_arg,         
        sampling_rate_arg,
        input_type_arg,

        #Lanza los nodos
        LogInfo(msg="--------------------------------------------------------------------------------"),
        LogInfo(msg="                      INICIANDO NODOS DE CONTROL                                "),
        LogInfo(msg="--------------------------------------------------------------------------------"),
        robot_state_publisher_node,
        keyboard_input_node,
        adapterToSimulation_node,
        kdl_ik_node,
        kdl_fk_node,
        trajectory_planning_node,

        # Acción con retardo para la comunicación
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="--------------------------------------------------------------------------------"),
                LogInfo(msg="                      INICIANDO COMUNICACIÓN CON REPRESENTADOR                  "),
                LogInfo(msg="--------------------------------------------------------------------------------"),
                foxglove_bridge_node 
            ]
        )
    ])