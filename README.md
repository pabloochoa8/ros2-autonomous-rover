# ROS2 Autonomous Rover

A fully autonomous wheeled rover built on the JPL Open Source Rover platform, running ROS2 Jazzy. The system performs marker-based global localization using ArUco detections fused through an EKF — with wheel odometry deliberately disabled — and navigates via the Nav2 stack with MPPI local control and Hybrid-A* global planning. A robotic arm subsystem with TMC2209 stepper drivers and PCA9685 servo control is included for sample retrieval tasks. The codebase was developed and validated through iterative field testing on a physical rover in a competition environment, requiring real hardware debugging across servo kinematics, sensor timing, and EKF frame configuration.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        rover_bringup                            │
│  (Nav2, EKF, ArUco config, launch orchestration, pure pursuit)  │
└───────────┬──────────────┬──────────────┬───────────────────────┘
            │              │              │
     ┌──────▼──────┐ ┌─────▼──────┐ ┌────▼───────────────┐
     │ osr-rover-  │ │ ros2_aruco │ │  ydlidar_ros2_      │
     │    code     │ │            │ │     driver           │
     │  (drive +   │ │ ArUco pose │ │  2D LiDAR scan      │
     │  steering)  │ │ detections │ │  → costmap          │
     └──────┬──────┘ └─────┬──────┘ └────────────────────┘
            │              │
     ┌──────▼──────┐ ┌─────▼──────────────────────────────┐
     │  Roboclaw   │ │          robot_localization          │
     │  (3 units,  │ │  EKF: fuses ArUco pose only         │
     │  6 motors + │ │  publishes map → base_link directly  │
     │  4 servos)  │ └────────────────────────────────────┘
     └─────────────┘
            │
     ┌──────▼──────────────────────────────────────────────┐
     │                  Nav2 Stack                          │
     │  Global: SmacPlannerHybrid (Hybrid A*)               │
     │  Local:  MPPIController                              │
     │  Fallback: SimplePurePursuit (custom, /goal_pose)   │
     └──────────────────────────────────────────────────────┘
            │
     ┌──────▼──────┐
     │   brazo_    │
     │   control   │
     │ (arm: TMC   │
     │  2209 +     │
     │  ServoKit)  │
     └─────────────┘
```

**Localization approach:** Wheel odometry is disabled. The EKF receives only absolute pose measurements from ArUco detections and publishes the `map → base_link` transform directly by setting `odom_frame = map`. This eliminates odometric drift at the cost of requiring marker visibility for continuous localization.

---

## Tech Stack

| Layer | Technology |
|---|---|
| Middleware | ROS2 Jazzy Jalisco |
| Simulation | Gazebo Harmonic |
| Navigation | Nav2 (Humble-compatible params) |
| Localization | `robot_localization` EKF |
| Global planner | `nav2_smac_planner::SmacPlannerHybrid` |
| Local controller | `nav2_mppi_controller::MPPIController` |
| Vision | OpenCV + ArUco (DICT_5X5_250, 20 cm markers) |
| 2D LiDAR | YDLiDAR (5 Hz scan rate, 9 kHz sample rate) |
| RGB camera | V4L2 (640×480, 10 FPS) |
| Depth camera | Orbbec Astra (RGB-D, driver included) |
| Drive MCU | RoboClaw (×3, serial 115200 baud, duty mode) |
| Servo board | PCA9685 via Adafruit ServoKit |
| Arm stepper | TMC2209 (UART, CRC, StealthChop) |
| Platform | Raspberry Pi (assumed) + rocker-bogie chassis |

---

## Packages

### `osr-rover-code`
Fork of the JPL Open Source Rover ROS2 codebase. Drives 6 goBilda motors (26.9:1 gear ratio) through three RoboClaw units at addresses 128/129/130. Corner steering is handled by 4 servos via PCA9685. Includes a servo direction fix for indices 2 and 3, required to match physical wiring on this specific build. The URDF is extended with Gazebo camera and LiDAR plugins for simulation parity.

### `rover_bringup`
Mission-level orchestration package. Contains:
- **EKF config** (`ekf_simple.yaml`): ArUco-only localization, yaw fusion enabled, odometry disabled.
- **Nav2 config** (`nav2_params.yaml`): MPPI + Hybrid A*, `route_server` disabled (caused segfault on this hardware).
- **Launch files**: `real.launch.py` (full Nav2 stack), `real_simple.launch.py` (minimal: ArUco + EKF + pure pursuit, no Nav2).
- **`simple_pure_pursuit.py`**: Custom P-controller that reads `/goal_pose` from RViz and commands `/cmd_vel_intuitive`. Max linear velocity 0.3 m/s, angular 0.8 rad/s, arrival tolerance 0.15 m.
- **ArUco models** for Gazebo: 12 markers (DICT_5X5_250) placed in a 7×7 m simulated arena.

### `ros2_aruco`
ArUco detection node. Subscribes to `/image_raw` and `/camera_info`, publishes detected marker poses. Configured for 20 cm markers with `DICT_5X5_250`. Detections are remapped and fed to the EKF as `geometry_msgs/PoseWithCovarianceStamped` on `/aruco_pose`.

### `brazo_control`
ROS2 node controlling a 5-DOF robotic arm attachment. Uses Adafruit ServoKit for joints on PCA9685 channels 4, 5, 6, 7, 11 (300° actuation range, 500–2500 µs pulse width). Linear axis driven by a TMC2209 stepper over UART with CRC-validated register writes and StealthChop enabled. Includes checkpoint-based motion sequencing.

### `ydlidar_ros2_driver`
YDLiDAR 2D LiDAR driver for `/scan` output. Configured at 5 Hz scan frequency (reduced from 10 Hz to cut CPU load on embedded hardware), full 360° sweep, range 0.01–64 m. Feeds the Nav2 costmap layers.

### `ros2_astra_camera`
Orbbec Astra RGB-D camera driver. Provides RGB stream and depth-to-color alignment. Used for marker detection via the RGB channel; depth stream available but not currently fused.

---

## Running It

### Dependencies

```bash
sudo apt install \
  ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-v4l2-camera \
  ros-jazzy-tf2-ros \
  python3-adafruit-circuitpython-servokit \
  python3-serial
```

### Build

```bash
cd rover_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch — Real Rover (full Nav2 stack)

```bash
ros2 launch rover_bringup real.launch.py
```

### Launch — Real Rover (minimal: EKF + ArUco + pure pursuit)

```bash
ros2 launch rover_bringup real_simple.launch.py
```

Send a navigation goal from RViz2 via the **2D Goal Pose** tool. The pure pursuit controller will drive the rover to the target using the EKF-fused map pose.

### Simulation (Gazebo Harmonic)

```bash
ros2 launch rover_bringup simulation.launch.py
```

---

## Results & Validation

The system was developed through iterative hardware testing in a competition-representative environment (7×7 m arena with 12 ArUco markers).

**Key issues resolved during field testing:**

- **Servo direction (indices 2 & 3):** Physical wiring on this chassis reverses the effective servo direction for two corner actuators. Fixed in `servo_control.py` with a per-index sign condition rather than a global direction parameter.
- **LiDAR rate (10 Hz → 5 Hz):** Scan processing at 10 Hz caused sustained CPU saturation on the onboard computer. Halving the rate resolved the load without measurable impact on obstacle avoidance latency.
- **Camera framerate (30 FPS → 10 FPS):** USB bandwidth contention with the LiDAR driver caused dropped frames. Capped at 10 FPS, ArUco detection remained stable.
- **EKF frame trick:** Setting `odom_frame = map` in `robot_localization` collapses the odom→map transform and lets the EKF publish `map → base_link` directly, avoiding a second TF layer when odometry is not used.
- **`route_server` disabled:** The Nav2 route server caused a segmentation fault on startup with this hardware/OS configuration. Removed from params; navigation operates without it.
- **Nav2 simulation blockers:** Camera URDF frame, 7×7 arena walls, TF camera offset, and `use_sim_time` propagation to the EKF were each debugged and corrected to achieve simulation parity with the real hardware configuration.

---

## Demo

> *Video / GIF of autonomous navigation run — to be added.*

---

## Author

**Pablo Ochoa Izaguirre**  
MSc Automation, Robotics & AI — Universidad Politécnica de Madrid  
[p.ochoaiza@gmail.com](mailto:p.ochoaiza@gmail.com) · [github.com/pabloochoa8](https://github.com/pabloochoa8)
