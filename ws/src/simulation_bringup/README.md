# UR5 Potatura Workspace

## Modifiche principali

### 1. Creazione della cartella `ur_description`

Per semplificare la gestione delle dipendenze da parte di Gazebo, ho creato una nuova cartella chiamata `ur_description` contenente il materiale necessario copiato dalla cartella originale `Universal_Robots_ROS2_Description`.  
Questo rende più semplice il caricamento delle mesh e dei file relativi durante l'esecuzione delle simulazioni.

---

### 2. Modifica del launch file per integrare Gazebo Fortress e spawnare il robot

All'interno del launch file ho aggiunto il lancio di Gazebo Fortress (`ros_gz_sim`) e direttamente lo spawn del robot.  
Dopo alcune ricerche, ho trovato come lanciare correttamente Gazebo Fortress osservando l'organizzazione del pacchetto `ros_gz_sim`.

Codice inserito:

```python
# Per Gazebo Fortress (ros_gz_sim)
ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
launch_dir = os.path.join(ros_gz_sim_dir, 'launch')

# Lancio di ros_gz_sim
gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gz_sim.launch.py')),
    launch_arguments={'gz_args': '-r empty.sdf'}.items(),
)

# Spawn del robot in Gazebo
spawn_entity = Node(
    package="ros_gz_sim",  # Pacchetto per la simulazione Gazebo
    executable="create",   # Eseguibile per spawnare il robot
    output="screen",       # Scrive l'output sullo schermo
    arguments=[
        "-string",
        robot_description_content,
        "-name",
        "ur",
        "-allow_renaming",
        "true",
    ],
)
```


### Impostazione della variabile d'ambiente per Gazebo

Per permettere a Gazebo di trovare i file `mesh` necessari al corretto caricamento dei modelli, bisogna aggiornare la variabile di ambiente `IGN_GAZEBO_RESOURCE_PATH`.

**Comando temporaneo (valido solo per il terminale corrente):**

```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/ros2_workspaces/ur5_potatura_ws/src
```
