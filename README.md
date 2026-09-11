# PX4 Autonomous Aerial Reconnaissance & Target Survey System

[![PX4 Autopilot](https://img.shields.io/badge/PX4-Autopilot%20v1.15-blue.svg)](https://px4.io/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/)
[![MAVSDK-Python](https://img.shields.io/badge/MAVSDK--Python-v2.0+-green.svg)](https://mavsdk.mavlink.io/main/en/python/)
[![Hardware Acceleration](https://img.shields.io/badge/NVIDIA-RTX%202050%20Accelerated-76B900.svg)](https://www.nvidia.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An end-to-end, ROS-independent autonomous aerial survey and optical reconnaissance platform engineered for multirotor UAVs using **PX4 Autopilot SITL**, **Gazebo Harmonic**, and **MAVSDK-Python**. The system deploys an electro-optical camera angled at $45^\circ$ forward-downward pitch, executing high-speed, continuous boustrophedon swaths with jerk-limited **Dubins arc turns**, real-time **target classification and georeferencing**, and a tactical **Ground Control Station (GCS)** adhering to a military obsidian-grey and crimson aesthetic.

---

## System Architecture

```
                                  +-----------------------------+
                                  |     Gazebo Harmonic Sim     |
                                  | (120m x 90m Proving Ground) |
                                  +--------------+--------------+
                                                 |
                       +-------------------------+-------------------------+
                       | Clock & Physics (ODE)                              | Image Transport (R8G8B8)
                       v                                                    v
         +----------------------------+                            +-------------------------+
         |    PX4 Autopilot SITL      |                            |   camera_stream.py      |
         | (EKF2 Odometry @ 100 Hz)   |                            | (Direct gz-transport)   |
         +-------------+--------------+                            +------------+------------+
                       |                                                        |
                       | MAVLink / UDP                                          | Raw Frames (30 FPS)
                       v                                                        v
         +----------------------------+                            +-------------------------+
         |     survey_mission.py      |                            |     box_detector.py     |
         | (MAVSDK Offboard Trajectory|<---------------------------+  (Classification, 2D    |
         |  Planner & State Machine)  |   Confirmed Target State   |   EKF & Georeferencing) |
         +-------------+--------------+                            +------------+------------+
                       |                                                        |
                       | Real-Time Telemetry & Log Queue                        | Annotated Optical HUD
                       +-------------------------+------------------------------+
                                                 |
                                                 v
                                  +-----------------------------+
                                  |       gcs_tkinter.py        |
                                  |  (Interactive Canvas, 2D    |
                                  |   Map, Telemetry & Table)   |
                                  +-----------------------------+
```

---

## 1. Proving Ground Environment (`worlds/box_survey.sdf`)

The simulation takes place within an authentic $120\text{m} \times 90\text{m}$ ($10,800\text{ m}^2$) operational proving ground designed to stress-test aerial reconnaissance algorithms:

- **Surface Demarcation:** A $90\text{m} \times 120\text{m}$ weathered tarmac apron enclosed by a $4\text{m}$-wide compacted gravel perimeter access road.
- **Topographic Variations:**
  - **Northwest Knoll:** Multi-tier natural rolling elevation ($Z = +0.65\text{m}$) with three terraced elevation rings.
  - **Central-East Ridge:** Elevated rock formation ($Z = +0.68\text{m}$) testing high-elevation slant perspectives.
- **Aviation & Survey Assets:**
  - **4 Corner Aviation Pylons:** Monotone survey beacons at arena corners with red obstacle warning strobes.
  - **Photogrammetric GCPs:** Standard high-contrast checkered ground control points for visual georeferencing.
  - **Operational Helipad Complex:** Marked takeoff and Return-to-Launch (RTL) hub centered at coordinates $(0.0, 0.0)$.
- **Dispersed Tactical Target Cargo Crates:**
  Seven distinct challenge crates positioned across the proving ground to stress-test the optical edges of the camera field of view:
  1. `Target #1 [RED_CUBE]`: $(+28.0\text{m N}, -42.5\text{m E})$ — Far-west perimeter edge (tests outer-left lens margin on Lane 1).
  2. `Target #2 [YELLOW_CUBE]`: $(+86.0\text{m N}, -24.0\text{m E})$ — Elevated on NW Knoll Tier 2 summit ($Z = -0.44\text{m}$ NED).
  3. `Target #3 [GREEN_CUBE]`: $(+56.0\text{m N}, -6.0\text{m E})$ — Inter-swath lateral flank between Lanes 2 and 3.
  4. `Target #4 [ORANGE_CUBE]`: $(+18.0\text{m N}, +8.0\text{m E})$ — Southern approach sector between Lanes 3 and 4.
  5. `Target #5 [BLUE_CUBE]`: $(+44.0\text{m N}, +24.0\text{m E})$ — Elevated on CE Natural Ridge base ($Z = -0.30\text{m}$ NED).
  6. `Target #6 [PURPLE_CUBE]`: $(+96.0\text{m N}, +42.5\text{m E})$ — Far-east perimeter edge (tests outer-right lens margin on Lane 5).
  7. `Target #7 [CYAN_CUBE]`: $(+106.0\text{m N}, +2.0\text{m E})$ — Far-north turnaround boundary (tests forward longitudinal optical limit).
- **Industrial Distractors (Non-Cube Geometries Filtered by Computer Vision):**
  - Chemical spherical tank, industrial drums, highway pylons, pressurized vessels, and modular concrete Jersey barriers.

---

## 2. Mathematical Foundations

### 2.1 Optical Ray-Ground Georeferencing
Given an image pixel $(u, v)$ from a camera with principal point $(c_x, c_y)$ and focal length $(f_x, f_y)$, the normalized optical ray in the camera frame $\mathbf{r}_c$ is:

$$\mathbf{r}_c = \begin{bmatrix} \frac{u - c_x}{f_x} \\ \frac{v - c_y}{f_y} \\ 1 \end{bmatrix}$$

The camera is rigidly mounted with a fixed pitch angle $\theta_c = 45^\circ$ relative to the UAV airframe. Transforming from camera frame to vehicle body frame:

$$\mathbf{r}_b = \mathbf{R}_y(\theta_c) \mathbf{r}_c = \begin{bmatrix} \cos\theta_c & 0 & \sin\theta_c \\ 0 & 1 & 0 \\ -\sin\theta_c & 0 & \cos\theta_c \end{bmatrix} \mathbf{r}_c$$

Using the vehicle's instantaneous attitude $(\phi, \theta, \psi)$ (roll, pitch, yaw) and the physical lever-arm offset $\mathbf{p}_{\text{lever}} = [0.18, 0, -0.242]^T\text{ m}$ relative to the vehicle center of gravity (CG), the camera optical center in world North-East-Down (NED) is:

$$\mathbf{p}_{\text{cam}} = \mathbf{p}_{\text{uav}} + \mathbf{R}_{b}^{w}(\phi, \theta, \psi) \mathbf{p}_{\text{lever}}$$

The ray direction in world NED is $\mathbf{r}_w = \mathbf{R}_{b}^{w} \mathbf{r}_b$. The intersection with a target plane at elevation $z_{\text{target}}$ is determined by scale parameter $t$:

$$t = \frac{z_{\text{target}} - p_{\text{cam}, z}}{r_{w, z}}, \quad \mathbf{p}_{\text{target}} = \mathbf{p}_{\text{cam}} + t \mathbf{r}_w$$

The algorithm incorporates dynamic terrain elevation lookups $z_{\text{terrain}}(x, y)$ over the knoll and ridge features to eliminate along-track projection bias.

### 2.2 Recursive 2D Kalman Filter (EKF)
For each detected target, a linear-quadratic estimator tracks the ground coordinates $\mathbf{x} = [p_{\text{north}}, p_{\text{east}}]^T$ with process noise covariance $\mathbf{Q} = 10^{-4} \cdot \mathbf{I}_2$ and range-dependent measurement covariance:

$$\mathbf{R} = \sigma_m^2 \mathbf{I}_2, \quad \sigma_m = \max(0.18, 0.10 + 0.025 \cdot d_{\text{range}})$$

State and error covariance updates follow the Joseph-stabilized formulation:

$$\mathbf{K}_k = \mathbf{P}_{k|k-1} (\mathbf{P}_{k|k-1} + \mathbf{R}_k)^{-1}$$

$$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T$$

### 2.3 Continuous Coordinated Dubins Turn Trajectory
Rather than stopping at lane ends, the UAV executes continuous $180^\circ$ circular arcs ($R = 9.0\text{m}$) connecting adjacent lanes ($18\text{m}$ lane spacing). The centripetal acceleration $a_c$ and required bank angle $\phi_{\text{bank}}$ at turn speed $V_t = 3.6\text{ m/s}$ are strictly bounded:

$$a_c = \frac{V_t^2}{R} = \frac{3.6^2}{9.0} = 1.44\text{ m/s}^2$$

$$\phi_{\text{bank}} = \arctan\left(\frac{a_c}{g}\right) = \arctan\left(\frac{1.44}{9.81}\right) \approx 8.35^\circ$$

This low bank angle ensures the optical sensor remains level and stable throughout turns without inducing perspective blur.

---

## 3. Dedicated Tactical Ground Control Station (`src/gcs_tkinter.py`)

A responsive, high-contrast tactical GCS engineered in Python Tkinter adhering to an aerospace obsidian/charcoal monotone theme with dark red status indicators:

- **Monotone Color Scheme:** Pitch black (`#000000`), neutral panels (`#070707`, `#0c0c0c`, `#141414`), silver/off-white typography (`#a0a0a0` / `#f0f0f0`), and restrained crimson accents (`#991b1b` / `#b91c1c`). Zero blue or slate tints.
- **Glass-Cockpit Telemetry Ribbon:** 6 primary avionics instruments displaying Altitude (AGL), Groundspeed, Heading (Yaw with Cardinal indicator), Attitude Stability (Pitch/Roll monitoring), NED Coordinates, and Swath Progress pips (`[■■□□□]`).
- **Live Optical Viewport:** Real-time 30 FPS camera feed with boresight HUD, horizon pitch lines, and georeferenced target bounding boxes.
- **2D Tactical Canvas:** Dynamic world-to-canvas coordinate mapping supporting interactive pan (`Mouse Drag`), zoom (`Mouse Wheel` / `[-]`/`[+]`), and automated UAV tracking (`[FOLLOW]`). Renders topographic contours, photogrammetric GCPs, planned swaths, Dubins arcs, dual-tone flight breadcrumbs, and confirmed target badges with 1-sigma uncertainty circles.
- **Target Reconnaissance Registry:** In-place Treeview displaying target ID, entity classification, georeferenced NED coordinates, 1-sigma uncertainty ($\sigma$), observation hits, and lock status.
- **Mission Event Console:** Non-blocking ticker streaming events directly from the flight autonomy queue.

---

## 4. Setup & Execution Guide

### Prerequisites
- **Host System:** Linux x86_64 with NVIDIA GPU (configured for Prime offload).
- **Container Environment:** Ubuntu 22.04 LTS container (`Khojo-o-Drone`) managed via Distrobox.
- **Core Dependencies:** PX4-Autopilot v1.15, Gazebo Harmonic, MAVSDK-Python, OpenCV (`python3-opencv`), NumPy, Pillow.

### Running the Autonomous Simulation
Execute the master launcher from the project root:

```bash
./run_sim.sh
```

### Command-Line Arguments
```bash
# Set custom ground cruise speed (default: 5.5 m/s)
./run_sim.sh --speed 6.0

# Run simulation in headless mode (no 3D Gazebo GUI)
./run_sim.sh --headless
```

### Standalone GCS Verification
To verify the tactical GCS user interface without launching PX4 and Gazebo:

```bash
distrobox-enter -n Khojo-o-Drone -- bash -c "python3 /home/devi/px4_drone_survey/src/test_gcs_render.py --keep-open"
```

---

## 5. Repository Structure

```
px4_drone_survey/
├── run_sim.sh               # Master simulation launcher (SITL + MAVSDK + GCS + Traps)
├── README.md                # System documentation & mathematical derivations
├── LICENSE                  # MIT License
├── stream.sdp               # SDP session description for video streaming
├── models/
│   ├── mono_cam_45/         # 45-deg angled electro-optical sensor model
│   └── x500_mono_cam_45/    # x500 airframe model with rigid camera mount
├── worlds/
│   └── box_survey.sdf       # 120m x 90m tactical proving ground with topography
├── detections/              # Optical target captures and MP4 mission recordings
│   ├── target_*_box.jpg
│   └── survey_mission_recording.mp4
└── src/
    ├── survey_mission.py    # MAVSDK offboard flight controller & state machine
    ├── box_detector.py      # Shape classifier, EKF tracking & DEM ray-casting
    ├── camera_stream.py     # gz-transport camera subscriber bridge
    ├── gcs_tkinter.py       # Tactical Ground Control Station GUI
    └── test_gcs_render.py   # Standalone GCS UI testing harness
```

---

## 6. License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
