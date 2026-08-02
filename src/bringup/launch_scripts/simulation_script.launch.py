import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    #===========================================================
    # URDF & ROBOT STATE PUBLISHER
    # ==========================================================
    # Obtener la ruta del archivo URDF
    urdf_path = os.path.join(
        get_package_share_directory("bringup"),
        "description",
        "manipulador.urdf.xacro"
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
        parameters=[robot_description],             # Lee toda cadena cinemática del robot 
    )

    # 4. Nodo publicador de estados articulares ficticios (Opcional pero recomendado)
    # Esto asegura que el brazo no se vea colapsado en el punto 0,0,0 mientras no tengas 
    # tus motores Rozum y Dynamixel publicando datos reales en /joint_states.
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )


    #===========================================================
    # KEYBOARD (C++ SDL2 Node)
    # ==========================================================
    adapterToSimulation_node = Node(
        package="user_interface",  
        executable="adapterToSimulation",     
        name="adapterToSimulation",      
        output="screen",
    )

    
    #===========================================================
    # KEYBOARD (C++ SDL2 Node)
    # ==========================================================
    keyboard_input_node = Node(
        package="user_interface",  
        executable="keyboard",     
        name="keyboard",      
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
        keyboard_input_node,
        adapterToSimulation_node,
        robot_state_publisher_node,
        foxglove_bridge_node  
    ])