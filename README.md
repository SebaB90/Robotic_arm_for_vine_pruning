
# UR5 – Potatura della Vite  
📅 *24/04/2025*  
Applicazione di potatura automatizzata su una vite utilizzando un braccio robotico **UR5** con ROS 2 e MoveIt.

---

## ⚙️ Setup di MoveIt

1. **Segui le guide ufficiali per installare MoveIt** su ROS 2 Humble:  
   - [Getting Started with MoveIt](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)  
   - [MoveIt Setup Assistant Tutorial](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

2. **Crea un pacchetto ROS per il tuo URDF personalizzato**:

   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_cmake my_robot_description
   ```

3. **Aggiungi il tuo file `.urdf.xacro`**:
   
   ```bash
   cd ~/ros2_ws/src/my_robot_description
   mkdir -p urdf
   cp /path/to/your_robot.urdf.xacro urdf/
   ```

4. **Modifica `CMakeLists.txt` per installare il file URDF**:

   Aggiungi questa sezione **prima di `ament_package()`**:

   ```cmake
   install(
     DIRECTORY urdf
     DESTINATION share/${PROJECT_NAME}
   )
   ```

5. **Aggiungi la dipendenza `xacro` nel `package.xml`**:

   ```xml
   <exec_depend>xacro</exec_depend>
   ```

6. **Compila il workspace**:

   ```bash
   cd ~/ros2_ws
   colcon build
   ```

7. **Source del workspace**:

   ```bash
   source install/setup.bash
   ```

   Per farlo automaticamente ogni volta, aggiungi questa riga al tuo `.bashrc`:

   ```bash
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   ```

8. **Crea l'ambiente MoveIt utilizzando il nuovo URDF**:  
   Segui di nuovo i passi del [MoveIt Setup Assistant Tutorial](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html), ma questa volta **importa il tuo URDF personalizzato** per configurare il robot nel sistema MoveIt.

---
