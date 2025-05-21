# 🦾 Guida: Avvio Comunicazione con UR5 (Standard, Non *e*-Series)

## ✅ Prerequisiti

- UR5 acceso e collegato via Ethernet
- PC con Ubuntu + ROS 2 Humble installato
- Driver ROS 2 per UR5 installato:  
  👉 [Universal Robots ROS 2 Driver (GitHub)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)  
  👉 [Guida ufficiale UR ROS 2 (Universal Robots)](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/toc.html)

---



## 1. Configurazione della Rete tra PC e UR5

1. Collega l'UR5 al PC tramite cavo Ethernet.
2. Sul PC, vai su **Impostazioni → Rete → Wired (Cablata)**.
3. Clicca sull’ingranaggio ⚙️ della connessione attiva.
4. Vai alla scheda **IPv4**, imposta il metodo su **Manuale**.
5. Inserisci i seguenti valori:
   - **Indirizzo (Address):** `192.168.0.100` *(indirizzo del tuo PC)*
   - **Netmask:** `255.255.255.0`
   - **Gateway:** lascia vuoto o `192.168.0.1` (non è obbligatorio)

⚠️ L’indirizzo IP del braccio (UR5) deve essere sulla stessa sottorete, ad esempio: `192.168.0.104`.  
⚠️ Quando si vorra ricollegare il proprio pc ad una rete cablata per la connessione internet bisognerà reimpostare l'**IPv4** su automatico. 


### ℹ️ Breve Spiegazione: IP e Netmask

Un indirizzo IP è composto da 4 blocchi (es. `192.168.0.104`), ognuno dei quali va da 0 a 255.  
La **Netmask** determina quanti bit devono coincidere per considerare due dispositivi "sulla stessa rete".

- `255.255.255.0` significa che i primi 3 blocchi devono essere uguali.
  - Esempio: `192.168.0.100` e `192.168.0.104` **sono nella stessa rete**
- Cambiare Netmask a `255.255.0.0` estenderebbe la rete a `192.168.x.x`

---



## 2. Verifica Connessione tra PC e UR5

Usa `ping` per testare se il braccio è raggiungibile

```bash
ping 192.168.0.104  # <-- IP del tuo UR5
```

---



## 3. 🧩 Avvio del Driver di Controllo (ROS 2 Control)

Apri un terminale e lancia il nodo di controllo per il robot UR5:
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.0.104
```
Assicurati di:
- Sostituire robot_ip con l’indirizzo IP reale del tuo UR5.
- Specificare correttamente il tipo di robot (ur_type:=ur5).

👀 Se tutto è corretto, vedrai che i giunti del robot vengono letti. Puoi verificarlo con:

```bash
ros2 topic echo /joint_states
```
📝 Attenzione: l’ordine dei giunti nell’output potrebbe essere diverso da quello che ti aspetti. Nel mio caso l’ultimo mostrato era il primo giunto fisico del braccio ()!

---



## 4. 🧪 Verifica dello Stato dei Controller

Per controllare se i controller ROS sono attivi, apri un nuovo terminale:

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {} | grep active
```

💡 Se nessun controller risulta attivo, è normale: bisogna attivare **URCap** dal tablet del robot.

📌 (Da verificare) Per un output più leggibile senza grep, puoi usare:

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {} -o yaml
```

---



## 5. 🖥️ Attivazione URCap sul Tablet UR5
1. Sul tablet di controllo dell’UR5, vai su **Program → URCaps**. 
2. Seleziona la voce External Control.
3. Premi Avvia (Play) sul programma caricato.

A questo punto, i controller si attiveranno.

Verifica nuovamente dal PC:

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {} | grep active
```

✅ Ora dovresti vedere attivi i controller **scaled_joint_trajectory_controller** e il joint_state_broadcaster, fondamentali per l'integrazione con MoveIt.

---



## 6. 🤖 Avvio di MoveIt con RViz
In un nuovo terminale, avvia MoveIt con l'interfaccia grafica RViz:

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
```

📦 Questo comando:
- Lancia MoveIt configurato per UR5
- Avvia RViz per la visualizzazione 3D

✅ Verifica Finale
- Trascina gli slider in RViz per muovere il robot.
- Se tutto è stato configurato correttamente, il robot reale seguirà i movimenti visualizzati in RViz!
