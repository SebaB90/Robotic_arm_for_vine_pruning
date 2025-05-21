import os  # Per la gestione dei percorsi dei file

from ament_index_python.packages import get_package_share_directory  # Per ottenere il percorso di condivisione dei pacchetti

# Importazioni necessarie per il launch file
from launch import LaunchDescription  # Utilizzato per definire una descrizione del lancio
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription # Permette di dichiarare argomenti da passare al launch file
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution  # Utilizzate per la sostituzione di variabili dinamiche nei percorsi
from launch_ros.actions import Node  # Per avviare i nodi ROS
from launch_ros.substitutions import FindPackageShare  # Per trovare il percorso di condivisione dei pacchetti ROS
from launch_ros.parameter_descriptions import ParameterValue  # Utilizzato per definire i parametri da passare ai nodi ROS
from launch.launch_description_sources import PythonLaunchDescriptionSource



# Funzione che genera la descrizione del lancio
def generate_launch_description():

    # Dichiarazione degli argomenti che saranno passati al launch file
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),  # Tipo di robot, valore di default è "ur5e"
        DeclareLaunchArgument("description_package", default_value="ur_description"),  # Nome del pacchetto che contiene la descrizione URDF
        DeclareLaunchArgument("description_file", default_value="ur.urdf.xacro"),  # Nome del file URDF da caricare, qui si usa un file xacro
        DeclareLaunchArgument("use_sim_time", default_value="true"),  # Indica se si usa il tempo di simulazione (true/false)
    ]

    # Per Gazebo Fortress (ros_gz_sim)
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    launch_dir = os.path.join(ros_gz_sim_dir, 'launch')
    
    # Inizializzazione delle configurazioni degli argomenti
    ur_type = LaunchConfiguration("ur_type")  # Configurazione per il tipo di robot (ur_type)
    description_package = LaunchConfiguration("description_package")  # Configurazione per il pacchetto della descrizione
    description_file = LaunchConfiguration("description_file")  # Configurazione per il nome del file URDF

    # Creazione del contenuto della descrizione del robot
    robot_description_content = Command([
        PathJoinSubstitution(["xacro"]),  # Esegui il comando 'xacro' per processare il file URDF
        " ",  # Spazio per separare i parametri
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),  # Trova il percorso del file URDF dentro il pacchetto
        " ",  # Spazio per separare i parametri
        "name:=ur5",  # Imposta il nome del robot
        " ",  # Spazio per separare i parametri
        "ur_type:=", ur_type,  # Passa l'argomento 'ur_type' che definisce il tipo di robot (es. ur5e)
    ])

    # Definizione del dizionario di parametri da passare ai nodi ROS
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)  # Definisce il parametro 'robot_description' che contiene il file URDF processato
    }

    # Definizione del file di configurazione per RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]  # Trova il file di configurazione RViz nel pacchetto
    )

    # Nodo per il publisher dello stato delle giunture (dati relativi alla posizione delle giunture del robot)
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",  # Pacchetto contenente il nodo
        executable="joint_state_publisher_gui",  # Eseguibile da lanciare
    )

    # Nodo per il publisher dello stato del robot (pubblica la descrizione del robot)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",  # Pacchetto che gestisce la pubblicazione dello stato del robot
        executable="robot_state_publisher",  # Eseguibile da lanciare
        output="both",  # Visualizza sia lo stdout che lo stderr nei log
        parameters=[robot_description],  # Passa il parametro 'robot_description' definito sopra
    )

    # Nodo per RViz, il visualizzatore 3D del robot
    rviz_node = Node(
        package="rviz2",  # Pacchetto RViz per ROS 2
        executable="rviz2",  # Eseguibile di RViz
        name="rviz2",  # Nome del nodo
        output="log",  # Scrive i log nei file di log
        arguments=["-d", rviz_config_file],  # Passa il file di configurazione RViz come argomento
    )

    # Lancio di ros_gz_sim
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    launch_dir = os.path.join(ros_gz_sim_dir, 'launch')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Spawn del robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",  # Pacchetto per la simulazione Gazebo
        executable="create",  # Eseguibile per spawnare il robot
        output="screen",  # Scrive l'output sullo schermo   
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur",
            "-allow_renaming",
            "true",
        ],
    )

    # Lista di nodi da avviare
    nodes_to_start = [
        joint_state_publisher_node,  # Nodo per il publisher dello stato delle giunture
        robot_state_publisher_node,  # Nodo per il publisher dello stato del robot
        rviz_node,  # Nodo per RViz
        gazebo_launch,  # Lancio di Gazebo
        spawn_entity,  # Spawn del robot in Gazebo
    ]

    # Restituisce la descrizione del lancio che contiene gli argomenti e i nodi da avviare
    return LaunchDescription(declared_arguments + nodes_to_start)
