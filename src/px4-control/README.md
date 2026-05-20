# px4_control + px4_tui

Companion-computer controller and TUI for PX4 SITL via the uXRCE-DDS bridge.

## What it does

- `px4_control_node` (C++) subscribes to PX4 odometry/status/battery/attitude and
  drives the quad by publishing `VehicleThrustSetpoint` + `VehicleTorqueSetpoint`
  (PX4's control allocator/mixer turns those into motor commands).
- A simple **geometric-PD controller** (Lee/Mellinger-style, simplified) runs on
  the companion side for `goto` targets.
- `takeoff`, `land`, and `emergency` (disarm) are forwarded to PX4 as
  `VehicleCommand` (MAVLink) — PX4 handles them internally.
- `px4_tui` (Python / Textual) is an interactive terminal UI for monitoring
  state and issuing commands.

## Topics

PX4 → companion (BEST_EFFORT, VOLATILE):

| topic                              | message                       |
|------------------------------------|-------------------------------|
| `/fmu/out/vehicle_odometry`        | `px4_msgs/VehicleOdometry`    |
| `/fmu/out/vehicle_status_v1`       | `px4_msgs/VehicleStatus`      |
| `/fmu/out/battery_status`          | `px4_msgs/BatteryStatus`      |
| `/fmu/out/vehicle_attitude`        | `px4_msgs/VehicleAttitude`    |

Companion → PX4:

| topic                              | message                            |
|------------------------------------|------------------------------------|
| `/fmu/in/offboard_control_mode`    | `px4_msgs/OffboardControlMode`     |
| `/fmu/in/vehicle_thrust_setpoint`  | `px4_msgs/VehicleThrustSetpoint`   |
| `/fmu/in/vehicle_torque_setpoint`  | `px4_msgs/VehicleTorqueSetpoint`   |
| `/fmu/in/vehicle_command`          | `px4_msgs/VehicleCommand`          |

Companion ↔ TUI:

| topic         | message                       |
|---------------|-------------------------------|
| `/tui/state`  | `px4_control/DroneState`      |
| `/tui/cmd`    | `px4_control/DroneCommand`    |

Controller diagnostics (intended for a Rerun bridge node, added separately):

| topic                            | message                       | rate                |
|----------------------------------|-------------------------------|---------------------|
| `/px4_control/pd_diagnostics`    | `px4_control/PDDiagnostics`   | 200 Hz during GOTO/HOLD |

`PDDiagnostics` carries `position_error`, `velocity_error`, `attitude_error`
(e_R in body frame), `rate_error` (e_ω in body frame), `desired_accel`, plus
the normalized `thrust_body` / `torque_body` commands, so a downstream node
can log them straight into Rerun as scalars/vectors. The
`px4_rerun_bridge` package in this workspace does exactly that.

If your PX4 version exposes a topic under a different name (e.g.
`/fmu/out/vehicle_status` without `_v1`), edit the topic strings in
`src/px4-control/src/px4_control_node.cpp`. Run `ros2 topic list` after the
uXRCE-DDS agent connects to confirm what your PX4 actually publishes.

## Build

```bash
cd ~/Documents/projects/drone-demo-ws
# px4_msgs is large; first build can take a few minutes.
colcon build --packages-select px4_msgs
source install/setup.bash
colcon build --packages-select px4_control px4_tui px4_rerun_bridge
source install/setup.bash
```

Python deps (Textual is not in apt as `python3-textual` on all distros — install
via pip if needed):

```bash
pip install --user textual
```

## Run (four terminals)

```bash
# Terminal A — PX4 SITL + Gazebo
cd ~/PX4-Autopilot
make px4_sitl gz_x500           # or gazebo-classic iris, depending on your setup

# Terminal B — uXRCE-DDS agent (already in this workspace)
MicroXRCEAgent udp4 -p 8888

# Terminal C — companion controller
source ~/Documents/projects/drone-demo-ws/install/setup.bash
ros2 launch px4_control px4_control.launch.py

# Terminal D — TUI
source ~/Documents/projects/drone-demo-ws/install/setup.bash
ros2 run px4_tui px4_tui

# Terminal E (optional) — Rerun bridge
source ~/Documents/projects/drone-demo-ws/install/setup.bash
ros2 run px4_rerun_bridge rerun_bridge        # spawns the viewer
# or record to a file for later replay:
ros2 run px4_rerun_bridge rerun_bridge --ros-args -p save_path:=/tmp/flight.rrd
```

Install rerun once:

```bash
pip install --user rerun-sdk
```

In the TUI:

| key | action                                                              |
|-----|---------------------------------------------------------------------|
| `t` | takeoff (prompts for AGL altitude)                                  |
| `l` | land                                                                |
| `e` | emergency — disarm immediately (motors stop, quad falls)            |
| `g` | goto — prompts for `x y z yaw_deg` (NED meters, yaw in degrees)     |
| `h` | hold — latch current position, PD controller keeps you there        |
| `q` | quit                                                                |

## Smoke test without the TUI

```bash
source install/setup.bash

# arm + takeoff
ros2 topic pub --once /tui/cmd px4_control/msg/DroneCommand \
  "{kind: 1, takeoff_altitude: 3.0}"

# goto (NED, z is "down" so -3 = 3 m up)
ros2 topic pub --once /tui/cmd px4_control/msg/DroneCommand \
  "{kind: 4, x: 2.0, y: 1.0, z: -3.0, yaw: 0.0}"

# land
ros2 topic pub --once /tui/cmd px4_control/msg/DroneCommand "{kind: 2}"

# emergency (only do this near the ground in SITL)
ros2 topic pub --once /tui/cmd px4_control/msg/DroneCommand "{kind: 3}"
```

Confirm the data is flowing:

```bash
ros2 topic hz /fmu/out/vehicle_odometry        # expect ~50 Hz
ros2 topic echo /tui/state -n 1                # one sample of the TUI state
ros2 topic hz /fmu/in/vehicle_thrust_setpoint  # expect ~200 Hz during goto/hold
```

## Tuning

PD gains and limits are exposed as ROS parameters in the launch file:

```bash
ros2 launch px4_control px4_control.launch.py \
  kp_pos:='[2.0, 2.0, 4.0]' \
  hover_thrust:=0.55
```

Defaults are tuned for the `gz_x500` model. The iris quad in gazebo-classic
typically wants slightly higher attitude gains.

## Architecture notes

- The PD outputs thrust and torque in **normalized PX4 body-FRD frame** (`[-1, 1]^3`).
  PX4's control allocator (`control_allocator`) then maps these into per-motor
  commands. We bypass PX4's outer-loop position and attitude controllers but
  keep its low-level mixer/ESC drivers.
- The controller is the standard geometric PD (position → desired acceleration
  → desired body z-axis + thrust magnitude; attitude error via the vee map of
  `R_desᵀ R - Rᵀ R_des`).
- For PX4 to accept thrust/torque setpoints we stream
  `OffboardControlMode{thrust_and_torque=true}` continuously at 200 Hz, then
  request the OFFBOARD mode (`VEHICLE_CMD_DO_SET_MODE`, custom_main_mode=6)
  after a 1 s warmup window.
- The companion node falls back to publishing neutral setpoints whenever
  odometry is stale (>200 ms), so an agent or PX4 disconnect won't produce
  diverging outputs.
