# Taller: Simulación de Quadrotor con PX4 + ROS 2 + Rerun

> **Carrera:** Ingeniería en Robótica 
> **Modalidad:** Práctica guiada 
> **Stack:** Ubuntu 22.04 · ROS 2 Humble · PX4-Autopilot · uXRCE-DDS Agent · Rerun SDK · Textual
> **Repo:** https://github.com/Fairbrook/drone-demo

---

## 1. Objetivos del taller

Al finalizar, el estudiante será capaz de:

1. Lanzar una simulación SITL de PX4 (`gz_x500`) y conectarla a ROS 2 vía el **uXRCE-DDS bridge**.
2. Habilitar el modo **Offboard** y enviar comandos de **thrust** y **torque** desde un nodo C++ propio.
3. Entender el **controlador PD geométrico** que el repo implementa (posición + actitud vía quaternion log map).
4. Visualizar en tiempo real estados, errores y comandos en **Rerun**, y operar el dron desde un **TUI Textual**.

> **Importante:** este taller NO usa los controladores internos de PX4 (`mc_pos_control`, `mc_att_control`) durante GOTO/HOLD. Le hablamos a PX4 al nivel más bajo posible: thrust + torque normalizados en el cuerpo del dron. Las acciones de alto nivel (takeoff/land/disarm) sí se delegan a PX4 vía `VehicleCommand`.

> **Nota sobre los snippets:** los fragmentos de código aquí citados están copiados textualmente del repo `Fairbrook/drone-demo` para servir como referencia rápida durante el taller. Para ver el código completo abre los archivos directamente.

---

## 2. Repaso teórico 

### 2.1 Dinámica simplificada

Para un quadrotor con masa `m`, matriz de inercia `J`, y orientación `R ∈ SO(3)`:

```
m · v̇ = -m·g·e3 + R · (T · e3)        (traslación, marco inercial NED)
J · ω̇ + ω × J·ω = τ                    (rotación, marco cuerpo FRD)
```

donde:
- `T` ∈ ℝ es el **empuje colectivo** (escalar, sobre +z body, aplicado en −z body).
- `τ` ∈ ℝ³ es el **torque** aplicado al cuerpo (roll, pitch, yaw).
- `(T, τ)` es exactamente lo que vamos a comandar a PX4 en este taller.

### 2.2 Asignación de control (mixer) — la hace PX4 por nosotros

Internamente PX4 toma `(T, τ)` y  obtiene los PWM de los 4 motores. Eso es lo que nos permite saltarnos los lazos altos de PX4 y enviar directo `(T, τ)`.

### 2.3 Controlador PD geométrico (lo que implementa el repo)

```
┌──────────────┐   p_des, yaw_des   ┌──────────────┐  a_des   ┌──────────────────┐  e_R, e_ω   ┌──────────────┐  T, τ
│  TUI / GOTO  │ ─────────────────► │ PD posición  │ ───────► │ Construir q_des  │ ──────────► │ PD actitud   │ ─────►  PX4
└──────────────┘                    └──────────────┘          └──────────────────┘             └──────────────┘
```

- **Lazo de posición:** `a_des = -kp_pos · e_p − kd_pos · v − g_NED`. La magnitud de empuje sale de proyectar `a_des` sobre `−z_body` (porque PX4 FRD aplica thrust en −z).
- **Atitud deseada:** `q_des` se construye en dos pasos — primero `q_align` que rota `(0,0,1)` body hacia `−a_des/|a_des|` en world, luego se multiplica por un yaw rotation `q_yaw(yaw_des)`.
- **Error de atitud:** `q_e = q_des⁻¹ ⊗ q_actual`, y luego `e_R = log(q_e)` (mapa exponencial inverso ≈ ½ · ángulo-eje).
- **PD de atitud con saturación tanh:** `u_r = -kp_att · e_R − kd_att · ω`, luego `u_r ← krmax · tanh(|u_r|/krmax) · û_r`.
- **Torque body:** `τ = J · u_r + ω × (J · ω)` (incluye el término de Coriolis), y se normaliza por eje a `[-1, 1]` con `max_torque`.

> Convención: PX4 usa **NED** world + **FRD** body (x-front, y-right, z-down). El thrust sale por −z body, y el yaw NED es horario visto desde arriba.

---

## 3. Setup del entorno 

> Esto debería estar **ya hecho** antes del taller. Lo dejamos documentado para que cada estudiante pueda reproducirlo en casa.

### 3.1 Requisitos

- Ubuntu 22.04 LTS
- ROS 2 Humble (`/opt/ros/humble`)
- Python ≥ 3.10
- ~10 GB de disco libre

### 3.2 Instalar PX4-Autopilot

```bash
cd ~
git clone --recursive --b v1.17.0 https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh    # instala toolchain + Gazebo
make px4_sitl                   # primera compilación (lenta)
```

### 3.3 Instalar uXRCE-DDS Agent (bridge PX4 ↔ ROS 2)

```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/
```

### 3.4 Workspace ROS 2 con `px4_msgs` + repo del taller

```bash
git clone https://github.com/Fairbrook/drone-demo.git
cd drone-demo-ws
source /opt/ros/humble/setup.bash

# px4_msgs primero (es grande; primera build tarda varios minutos)
colcon build --symlink-install
source install/setup.bash
```

### 3.5 Dependencias Python

```bash
pip install --user rerun-sdk textual
```

---

## 4. Arquitectura del taller

```
┌────────────────────┐   UDP    ┌─────────────────────┐   DDS   ┌────────────────────────────────────────┐
│   PX4 SITL         │ ───────► │  uXRCE-DDS Agent    │ ──────► │  ROS 2 (Humble)  ·  px4_msgs           │
│   + gz_x500        │ ◄─────── │  (udp4 -p 8888)     │ ◄────── │                                        │
└────────────────────┘          └─────────────────────┘         └────────────────────────────────────────┘
                                                                          │ /fmu/out/*           ▲ /fmu/in/*
                                                                          ▼                      │
                                                          ┌────────────────────────────────────────┐
                                                          │ px4_control_node (C++)                 │
                                                          │  - máquina de estados                  │
                                                          │  - PD geométrico (Eigen)               │
                                                          │  - publica thrust + torque             │
                                                          └────────────────────────────────────────┘
                                                          /tui/state ▲      ▼ /tui/cmd       ▼ /px4_control/pd_diagnostics
                                                 ┌─────────────────┐ │      │            ┌────────────────────────┐
                                                 │  px4_tui        │ │      │            │  px4_rerun_bridge      │
                                                 │  (Textual TUI)  │─┘      └──────────► │  (spawn Rerun viewer)  │
                                                 └─────────────────┘                     └────────────────────────┘
```

**Topics PX4 ↔ ROS 2 (los que usa el `px4_control_node`):**

| Dirección | Topic | Mensaje | QoS |
|-----------|-------|---------|-----|
| OUT | `/fmu/out/vehicle_odometry`        | `px4_msgs/VehicleOdometry`      | best_effort, volatile |
| OUT | `/fmu/out/vehicle_status_v1`       | `px4_msgs/VehicleStatus`        | best_effort, volatile |
| OUT | `/fmu/out/battery_status`          | `px4_msgs/BatteryStatus`        | best_effort, volatile |
| OUT | `/fmu/out/vehicle_attitude`        | `px4_msgs/VehicleAttitude`      | best_effort, volatile |
| IN  | `/fmu/in/offboard_control_mode`    | `px4_msgs/OffboardControlMode`  | best_effort, volatile |
| IN  | `/fmu/in/vehicle_thrust_setpoint`  | `px4_msgs/VehicleThrustSetpoint`| best_effort, volatile |
| IN  | `/fmu/in/vehicle_torque_setpoint`  | `px4_msgs/VehicleTorqueSetpoint`| best_effort, volatile |
| IN  | `/fmu/in/vehicle_command`          | `px4_msgs/VehicleCommand`       | best_effort, volatile |

**Topics internos (custom msgs del paquete `px4_control`):**

| Topic | Mensaje | Rate | QoS |
|-------|---------|------|-----|
| `/tui/state`                  | `px4_control/DroneState`     | 10 Hz                 | reliable |
| `/tui/cmd`                    | `px4_control/DroneCommand`   | event-driven          | reliable |
| `/px4_control/pd_diagnostics` | `px4_control/PDDiagnostics`  | 200 Hz en GOTO/HOLD   | best_effort |

---

## 5. Primer arranque

Abre **5 terminales**.

### Terminal A — PX4 SITL + Gazebo

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Espera a ver el prompt `pxh>` y el dron en Gazebo.

### Terminal B — Agente uXRCE-DDS

```bash
MicroXRCEAgent udp4 -p 8888
```

Deberías ver mensajes "create_participant" cuando PX4 se conecte.

### Terminal C — Verificar topics ROS 2

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/projects/drone-demo-ws/install/setup.bash
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_odometry --once
```

✅ **Checkpoint:** si ves la odometría con `position` válida, el puente funciona.

> Si tu PX4 expone `/fmu/out/vehicle_status` sin `_v1`, edita las rutas en `src/px4-control/src/px4_control_node.cpp`.

---

## 6. Anatomía del nodo `px4_control_node`

> El código vive en `src/px4-control/`. Lo explicamos por bloques.

### 6.1 Máquina de estados

```cpp
enum class FlightState {
  IDLE,
  TAKING_OFF,
  HOLD,
  GOTO,
  LANDING,
  EMERGENCY,
};
```

- `IDLE`: nada se publica. Esperando un `DroneCommand`.
- `TAKING_OFF`: se delega a PX4 (`VEHICLE_CMD_NAV_TAKEOFF`). Cuando la altitud está dentro de `kTakeoffAltitudeTolM` del objetivo → `HOLD`.
- `HOLD` / `GOTO`: el PD geométrico genera `(T, τ)` y publica `vehicle_thrust_setpoint` + `vehicle_torque_setpoint` a 200 Hz.
- `LANDING`: se delega a PX4 (`VEHICLE_CMD_NAV_LAND`).
- `EMERGENCY`: disarm inmediato (`VEHICLE_CMD_COMPONENT_ARM_DISARM` con 0).

### 6.2 Habilitar Offboard

Antes de poder mandar thrust/torque hay que hacer 2 cosas, **en este orden**:

1. **Publicar `OffboardControlMode` a 200 Hz durante ~1 segundo** (`kOffboardWarmupTicks = 200`) con `thrust_and_torque = True` y los demás flags en False. PX4 exige ver el stream antes de aceptar el modo.
2. Enviar `VehicleCommand` con `VEHICLE_CMD_DO_SET_MODE` (Offboard, custom_mode 6).

### 6.3 Loop de control a 200 Hz (`controlTick`)

```cpp
control_timer_ = create_wall_timer(
    5ms, std::bind(&Px4ControlNode::controlTick, this));  // 200 Hz
state_timer_ = create_wall_timer(
    100ms, std::bind(&Px4ControlNode::statePublishTick, this));  // 10 Hz
```

En `controlTick` el nodo:
1. Verifica que la odometría no esté stale (`kOdomStaleSec = 0.2 s`).
2. Publica `OffboardControlMode` (siempre, mientras esté activo).
3. Según el `FlightState`, decide si llamar al PD o publicar setpoints neutros.

### 6.4 Publicar `(T, τ)` a PX4

```cpp
void publishThrustTorque(const Eigen::Vector3d &thrust,
                         const Eigen::Vector3d &torque);
```

Internamente:
```cpp
ts.timestamp = nowUs();
ts.xyz = { thrust.x(), thrust.y(), thrust.z() };   // ya viene como (0, 0, -T_norm) del controller
thrust_pub_->publish(ts);

tq.timestamp = nowUs();
tq.xyz = { torque.x(), torque.y(), torque.z() };
torque_pub_->publish(tq);
```

### 6.5 QoS PX4

uXRCE-DDS publica con `BEST_EFFORT`, `VOLATILE`. Nuestros subs y pubs hacia `/fmu/*` usan el mismo perfil; lo expone el nodo así:

```cpp
rclcpp::QoS px4SubQos() const {
  return rclcpp::QoS(rclcpp::KeepLast(10))
      .best_effort().durability_volatile();
}
rclcpp::QoS px4PubQos() const {
  return rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort().durability_volatile();
}
```

---

## 7. Controlador PD geométrico — paso a paso

Código en `src/px4-control/src/pd_controller.cpp`. Seis pasos:

### Paso 1 — Position PD → aceleración deseada (NED)

```cpp
const Eigen::Vector3d e_p = in.p - in.p_des;
const Eigen::Vector3d e_v = in.v;            // target velocity = 0
Eigen::Vector3d a_des = -gains_.kp_pos.cwiseProduct(e_p)
                       - gains_.kd_pos.cwiseProduct(e_v);
a_des.z() += -kGravity;                       // compensación de gravedad en NED
```

### Paso 2 — Saturación horizontal

```cpp
Eigen::Vector2d a_xy(a_des.x(), a_des.y());
if (a_xy.norm() > gains_.max_accel_xy) {
  a_xy *= gains_.max_accel_xy / a_xy.norm();
  a_des.head<2>() = a_xy;
}
```

### Paso 3 — Magnitud de empuje (proyección sobre −z body)

```cpp
const Eigen::Matrix3d R = in.q.toRotationMatrix();
const double T_scalar = -a_des.dot(R.col(2));   // R.col(2) = z_body en world
const double T_norm   = clamp(T_scalar * (hover_thrust / g), 0, 1);
Eigen::Vector3d thrust_body(0.0, 0.0, -T_norm); // PX4 FRD: thrust por -z
```

### Paso 4 — Construcción de `q_des`

```cpp
const Eigen::Vector3d h = -a_des / a_des.norm();   // dirección deseada de -z_body
const Eigen::Quaterniond q_align = quatFromZ(h);   // rota (0,0,1) → h
const Eigen::Quaterniond q_yaw(cos(yaw_des/2), 0, 0, sin(yaw_des/2));
Eigen::Quaterniond q_target = q_align * q_yaw;
```

### Paso 5 — Error de atitud vía quaternion log

```cpp
Eigen::Quaterniond q_e = q_target.conjugate() * in.q;
if (q_e.w() < 0) q_e.coeffs() *= -1.0;      // ruta corta
const Eigen::Vector3d e_R = quatLog(q_e);   // ≈ (θ/2)·n̂
```

> `quatLog` está definido en el mismo archivo: `r = (vec(q)/|vec(q)|) · acos(w)`.

### Paso 6 — PD de atitud con saturación tanh + torque body

```cpp
Eigen::Vector3d u_r = -gains_.kp_att.cwiseProduct(e_R)
                     - gains_.kd_att.cwiseProduct(in.omega);
// Saturación suave en la magnitud
if (u_r.norm() > 1e-9 && gains_.krmax > 0) {
  u_r = gains_.krmax * std::tanh(u_r.norm() / gains_.krmax) * (u_r / u_r.norm());
}
// τ = J·u + ω × (J·ω)
const Eigen::Vector3d Jw = gains_.inertia.cwiseProduct(in.omega);
const Eigen::Vector3d tau = gains_.inertia.cwiseProduct(u_r) + in.omega.cross(Jw);
// Normalizar por eje a [-1, 1]
Eigen::Vector3d torque_body{
  clamp(tau.x()/max_torque.x(), -1, 1),
  clamp(tau.y()/max_torque.y(), -1, 1),
  clamp(tau.z()/max_torque.z(), -1, 1),
};
```

### Ganancias por defecto (en `pd_controller.hpp` y expuestas como parámetros ROS)

| Parámetro | Default | Rol |
|-----------|---------|-----|
| `kp_pos`        | `[1.5, 1.5, 3.0]`   | proporcional posición xyz |
| `kd_pos`        | `[1.2, 1.2, 2.5]`   | derivativo velocidad xyz |
| `kp_att`        | `[12.0, 12.0, 4.0]` | proporcional actitud (sobre `log(q_e)`) |
| `kd_att`        | `[1.6, 1.6, 0.8]`   | derivativo rates body |
| `inertia`       | `[0.029, 0.029, 0.055]` kg·m² | inercia diagonal del cuerpo |
| `krmax`         | `50.0` rad/s²       | saturación tanh sobre `u_r` |
| `max_torque`    | `[0.5, 0.5, 0.2]`   | normalización por eje a PX4 |
| `max_accel_xy`  | `10.0` m/s²         | clamp lateral de `a_des` |
| `hover_thrust`  | `1.0` (norm)        | escala T_scalar → thrust normalizado |

---

## 8. Visualización con Rerun

Código en `src/px4-rerun-bridge/px4_rerun_bridge/bridge.py`.

### 8.1 Qué consume

- `/px4_control/pd_diagnostics` (`PDDiagnostics`, ~200 Hz durante GOTO/HOLD)
- `/tui/state` (`DroneState`, 10 Hz)

### 8.2 Qué loguea

El bridge ya viene con un **blueprint** preconfigurado. No tienes que diseñar la vista:

- Vista 3D `/world` con dron (punto verde), target (punto rojo), línea entre ambos, y flecha de `desired_accel`.
- Plots por componente xyz / rpy / pqr para:
  - `errors/position`, `errors/velocity`
  - `errors/attitude` (e_R), `errors/rate` (e_ω)
  - `cmd/thrust`, `cmd/torque`
  - `ref/desired_accel`
- Plots de posición medida vs target por eje (xyz).
- Plot de yaw medido vs target (en grados).
- Plot de estado: `armed`, `failsafe`, `offboard_active`, `nav_state`, voltaje, batería %.

### 8.3 Modos de ejecución

```bash
# Lanzar el viewer y verlo en vivo
ros2 run px4_rerun_bridge rerun_bridge

# Grabar a archivo .rrd para reproducir luego (sin viewer abierto)
ros2 run px4_rerun_bridge rerun_bridge --ros-args -p save_path:=/tmp/flight.rrd
```

✅ **Checkpoint:** al hacer un GOTO desde el TUI, en Rerun verás el dron moviéndose, la línea hacia el target, y los plots de errores aproximándose a 0.

---

## 9. TUI con Textual

Código en `src/px4-tui/px4_tui/`.

### 9.1 Qué hace

- Suscribe a `/tui/state` (RELIABLE QoS) y publica `/tui/cmd`.
- Renderiza panel de estado (batería, pose NED, arming, nav_state, offboard, failsafe, goto activo).
- Modal para ingresar `x y z yaw_deg` (GOTO) o altitud (TAKEOFF).

### 9.2 Atajos de teclado

| Tecla | Acción |
|-------|--------|
| `t` | TAKEOFF (modal pide altitud AGL en metros, default 3.0) |
| `l` | LAND |
| `e` | EMERGENCY (disarm inmediato) |
| `g` | GOTO (modal pide `x y z yaw_deg` en NED) |
| `h` | HOLD en posición actual |
| `q` | Quit |

> Cuando hagas `GOTO`, el TUI manda `yaw` en grados; el TUI lo convierte a radianes con `math.radians` antes de publicar.

---

## 10. Ejecución de la demo completa

Abre **5 terminales**:

| # | Comando |
|---|---------|
| A | `cd ~/PX4-Autopilot && make px4_sitl gz_x500` |
| B | `MicroXRCEAgent udp4 -p 8888` |
| C | `source ~/Documents/projects/drone-demo-ws/install/setup.bash && ros2 launch px4_control px4_control.launch.py` |
| D | `source ~/Documents/projects/drone-demo-ws/install/setup.bash && ros2 run px4_tui px4_tui` |
| E | `source ~/Documents/projects/drone-demo-ws/install/setup.bash && ros2 run px4_rerun_bridge rerun_bridge` |

Flujo esperado:
1. En el TUI presiona `t` → TAKEOFF (modal: 3.0 m).
2. PX4 entra a `AUTO_TAKEOFF`. Cuando llega cerca de 3 m, el nodo transiciona a `HOLD` → toma control vía thrust+torque.
3. Presiona `g` → GOTO con `2.0 1.0 -3.0 0` (NED: 2 m norte, 1 m este, 3 m de altura, yaw 0).
4. En Rerun ves el dron acercarse al target, error → 0, comandos saturados al inicio y suavizándose.
5. Presiona `l` → LAND, o `e` para parada de emergencia.

---

## 11. Ejercicios en vivo

### Ejercicio 1 — Tunear `kp_pos`
Lanza el launch con la mitad del default: `ros2 launch px4_control px4_control.launch.py kp_pos:='[0.75, 0.75, 1.5]'`. Manda un GOTO y observa rise time / overshoot en Rerun.

### Ejercicio 2 — Sin componente derivativa de actitud
`kd_att:='[0.0, 0.0, 0.0]'`. ¿Por qué oscila? Conecta con el rol del D en sistemas de 2.º orden.

### Ejercicio 3 — Saturación más estricta
`max_torque:='[0.2, 0.2, 0.1]'` y haz un GOTO agresivo (5 m, yaw 90°). ¿Cuándo deja de seguir? Discute la pérdida de autoridad.

### Ejercicio 4 — `krmax` bajo
`krmax:=5.0`. El tanh ahora satura mucho antes. ¿Qué pasa con la respuesta?

### Ejercicio 5 — Cambiar `hover_thrust`
Sube a `1.5` o baja a `0.7`. Observa el efecto directo en la altitud durante HOLD.

> Todos los parámetros son ROS params declarados en el launch (`src/px4-control/launch/px4_control.launch.py`), así que se cambian sin recompilar.

---

## 13. Recursos

- **Repo del taller:** https://github.com/Fairbrook/drone-demo
- **PX4 Offboard Mode:** https://docs.px4.io/main/en/flight_modes/offboard.html
- **uXRCE-DDS bridge:** https://docs.px4.io/main/en/middleware/uxrce_dds.html
- **px4_msgs (release/1.14):** https://github.com/PX4/px4_msgs/tree/release/1.14
- **Rerun docs:** https://www.rerun.io/docs
- **Textual:** https://textual.textualize.io/
- **Lee, Leok, McClamroch (2010), *Geometric tracking control of a quadrotor UAV on SE(3)*.**
- **Mellinger & Kumar (2011), *Minimum snap trajectory generation and control*.**

---

## Apéndice A — Estructura del repo

```
drone-demo/
├── .gitignore
└── src/
    ├── px4-control/                  # C++ controller node + custom msgs
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── README.md
    │   ├── include/px4_control/
    │   │   ├── pd_controller.hpp
    │   │   └── px4_control_node.hpp
    │   ├── src/
    │   │   ├── main.cpp
    │   │   ├── pd_controller.cpp
    │   │   └── px4_control_node.cpp
    │   ├── msg/
    │   │   ├── DroneCommand.msg
    │   │   ├── DroneState.msg
    │   │   └── PDDiagnostics.msg
    │   └── launch/
    │       └── px4_control.launch.py
    ├── px4-rerun-bridge/             # Python — Rerun visualizer
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   └── px4_rerun_bridge/
    │       ├── __init__.py
    │       └── bridge.py
    └── px4-tui/                      # Python — Textual TUI
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/px4_tui
        └── px4_tui/
            ├── __init__.py
            ├── main.py
            └── app.py
```

## Apéndice B — Mensajes custom

**`DroneCommand.msg`** (lo que manda el TUI):

```
uint8 KIND_TAKEOFF   = 1
uint8 KIND_LAND      = 2
uint8 KIND_EMERGENCY = 3
uint8 KIND_GOTO      = 4
uint8 KIND_HOLD      = 5
uint8 kind
float64 x, y, z, yaw           # NED, radianes (sólo GOTO)
float32 takeoff_altitude        # AGL en metros (sólo TAKEOFF)
```

**`DroneState.msg`** (lo que publica el controller a 10 Hz):

```
# pose NED (x,y,z), roll/pitch/yaw, body angular velocity wx/wy/wz,
# battery (voltage, remaining, warning), arming_state, nav_state,
# failsafe, offboard_active, goto activo + target.
```

**`PDDiagnostics.msg`** (lo que consume el bridge a 200 Hz):

```
# position, velocity, position_target, yaw, yaw_target
# position_error, velocity_error, attitude_error (e_R), rate_error (e_ω)
# desired_accel, thrust_body (norm), torque_body (norm)
```

## Apéndice C — Troubleshooting rápido

| Síntoma | Causa probable | Solución |
|---------|----------------|----------|
| `ros2 topic list` no muestra `/fmu/*` | Agente no corre o PX4 no se conectó | Verificar terminal B; reiniciar PX4 |
| Dron no entra en Offboard | No publicaste `OffboardControlMode` antes de cambiar de modo | El nodo lo hace solo; espera `kOffboardWarmupTicks` (~1 s) |
| `vehicle_status` no existe | Tu PX4 expone `vehicle_status` sin `_v1` | Editar nombres en `px4_control_node.cpp` |
| Rerun no muestra plots | El controller está en IDLE / TAKING_OFF | Pasa a HOLD o GOTO; las diagnósticas sólo se publican ahí |
| El dron sube y choca | `hover_thrust` mal calibrado para este airframe | Bájalo en el launch (e.g. `hover_thrust:=0.7`) |
| Build de `px4_rerun_bridge` falla | Falta `rerun-sdk` o `px4_msgs` no buildeado primero | `pip install --user rerun-sdk` y rebuilds en orden |
