
# UR5 – Potatura della Vite  
📅 *24/04/2025*  
Applicazione di potatura automatizzata su una vite utilizzando un braccio robotico **UR5** con ROS 2 e MoveIt.

---

## ⚙️ Setup di MoveIt

### 1. Installazione di MoveIt
Segui attentamente queste due guide ufficiali per installare MoveIt su ROS 2 Humble:
- [Getting Started with MoveIt](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)
- [MoveIt Setup Assistant Tutorial](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

---

## 🤖 Utilizzo di un URDF custom

### 2. Creazione del pacchetto `my_robot_description`

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_description
```

### 3. Inserisci il tuo file `.urdf.xacro`

```bash
cd ~/ros2_ws/src/my_robot_description
mkdir -p urdf
cp /path/to/your_robot.urdf.xacro urdf/
```

### 4. Modifica `CMakeLists.txt` per installare i file URDF

Aggiungi questa sezione **prima di `ament_package()`**:

```cmake
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
```

### 5. Aggiungi le dipendenze in `package.xml`

Se utilizzi file `.xacro`, aggiungi questa riga:

```xml
<exec_depend>xacro</exec_depend>
```

💡 *Commento utile:*  
> Serve per usare `xacro` durante la generazione del modello URDF al runtime.

---

## 🔧 Build e setup del workspace

### 6. Compila il workspace

```bash
cd ~/ros2_ws
colcon build
```

### 7. Source dell’ambiente

Ogni volta che apri un nuovo terminale, ricorda di eseguire:

```bash
source install/setup.bash
```

Oppure rendilo automatico aggiungendo questa riga al tuo `.bashrc`:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---
