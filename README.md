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

## 1. Tactical HADR Proving Ground (`worlds/box_survey.sdf`)

The simulation models an authentic $120\text{m} \times 90\text{m}$ ($10,800\text{ m}^2$) Humanitarian Assistance & Disaster Relief (HADR) / post-calamity proving ground:

- **Surface Demarcation:** A $90\text{m} \times 120\text{m}$ weathered tarmac survey apron enclosed by a $4\text{m}$-wide compacted gravel perimeter access road.
- **Topographic Variations:**
  - **Northwest Knoll:** Multi-tier natural rolling elevation ($Z = +0.65\text{m}$) with three terraced elevation rings.
  - **Central-East Ridge:** Elevated rock formation ($Z = +0.68\text{m}$) testing high-elevation slant perspectives.
- **Structural Infrastructure Assets:**
  - **ISO 20ft Shipping Containers:** Industrial cargo boxes placed at $(+78.0\text{m N}, -18.0\text{m E})$ and $(+38.0\text{m N}, +18.0\text{m E})$ providing authentic vertical scale, acoustic/visual occlusion, and line-of-sight bounds.
  - **Collapse Debris Fields:** Concrete rubble and masonry collapse clusters at $(+60.0\text{m N}, 0.0\text{m E})$ and $(+25.0\text{m N}, -22.0\text{m E})$.
  - **Aviation Corner Pylons & GCPs:** 4 corner aviation warning towers with crimson beacons and photogrammetric ground control points.
  - **Operational Helipad Hub:** Takeoff and Return-to-Launch (RTL) zone centered at $(0.0, 0.0)$.
- **Tactical HADR Triage Inventory:**

| ID | Priority | Tactical Classification | Local NED (X, Y) | Target Dimensions | Mission Action |
| :---: | :---: | :--- | :---: | :---: | :--- |
| **#01** | `P1 - CRITICAL` | **SURVIVOR CASUALTY LZ MARKER** | `(+28.05m, -42.45m)` | $1.0\text{m} \times 1.0\text{m} \times 0.6\text{m}$ | Rapid extraction / Airdrop medical pack |
| **#02** | `P1 - CRITICAL` | **BIOHAZARD CONTAINMENT DRUM** | `(+86.12m, -23.95m)` | $0.8\text{m} \times 0.8\text{m} \times 1.1\text{m}$ | Hazmat isolation & decontamination |
| **#03** | `P2 - HIGH` | **FIELD MEDICAL SUPPLY CACHE** | `(+55.95m, -6.08m)` | $1.2\text{m} \times 1.2\text{m} \times 0.8\text{m}$ | Dispatch triage ground team |
| **#04** | `P2 - HIGH` | **POTABLE WATER DISTRIBUTION** | `(+17.92m, +8.04m)` | $1.0\text{m} \times 1.0\text{m} \times 1.0\text{m}$ | Allocate purification filtration unit |
| **#05** | `P3 - LOGISTICS`| **EMERGENCY TELECOM REPEATER** | `(+44.10m, +23.90m)` | $0.7\text{m} \times 0.7\text{m} \times 1.4\text{m}$ | Deploy mesh backhaul gateway |
| **#06** | `P3 - LOGISTICS`| **SECONDARY GENERATOR / FUEL** | `(+95.88m, +42.40m)` | $1.1\text{m} \times 0.9\text{m} \times 0.8\text{m}$ | Schedule forward power restoration |
| **#07** | `P2 - HIGH` | **SEARCH & RESCUE RALLY ZONE** | `(+105.95m, +2.05m)` | $1.5\text{m} \times 1.5\text{m} \times 0.5\text{m}$ | Establish forward evac corridor |

---

## 2. Mathematical Foundations

### 2.1 Dynamic 3D Terrain-Following Altitude Control
To maintain optimal sensor resolution and constant ground sampling distance ($GSD \approx 1.2\text{ cm/px}$), the UAV locks altitude at a constant $12.0\text{m}$ Above Ground Level (AGL) across varying topography. Given local DEM surface elevation $Z_{\text{elev}}(p_x, p_y)$ where $Z < 0$ in NED represents ground above the datum:

$$z_{\text{cmd}} = -\left(12.0 - Z_{\text{elev}}(p_x, p_y)\right)$$

The vertical velocity command $v_z$ is driven by a proportional controller:

$$v_z = -0.85 \cdot (z_{\text{cmd}} - p_z)$$

This dynamically raises the UAV over the $+0.65\text{m}$ knoll and $+0.68\text{m}$ ridge while keeping flight velocity smooth and vibration-free.

### 2.2 Optical Ray-Ground Georeferencing
Given an image pixel $(u, v)$ from a camera with principal point $(c_x, c_y)$ and focal length $(f_x, f_y)$, the normalized optical ray in the camera frame $\mathbf{r}_c$ is:

$$\mathbf{r}_c = \begin{bmatrix} \frac{u - c_x}{f_x} \\ \frac{v - c_y}{f_y} \\ 1 \end{bmatrix}$$

The camera is rigidly mounted with a fixed pitch angle $\theta_c = 45^\circ$ relative to the UAV airframe. Transforming from camera frame to vehicle body frame:

$$\mathbf{r}_b = \mathbf{R}_y(\theta_c) \mathbf{r}_c = \begin{bmatrix} \cos\theta_c & 0 & \sin\theta_c \\ 0 & 1 & 0 \\ -\sin\theta_c & 0 & \cos\theta_c \end{bmatrix} \mathbf{r}_c$$

Using the vehicle's instantaneous attitude $(\phi, \theta, \psi)$ (roll, pitch, yaw) and the physical lever-arm offset $\mathbf{p}_{\text{lever}} = [0.18, 0, -0.242]^T\text{ m}$ relative to the vehicle center of gravity (CG), the camera optical center in world North-East-Down (NED) is:

$$\mathbf{p}_{\text{cam}} = \mathbf{p}_{\text{uav}} + \mathbf{R}_{b}^{w}(\phi, \theta, \psi) \mathbf{p}_{\text{lever}}$$

The ray direction in world NED is $\mathbf{r}_w = \mathbf{R}_{b}^{w} \mathbf{r}_b$. The intersection with a target plane at elevation $z_{\text{target}}$ is determined by scale parameter $t$:

$$t = \frac{z_{\text{target}} - p_{\text{cam}, z}}{r_{w, z}}, \quad \mathbf{p}_{\text{target}} = \mathbf{p}_{\text{cam}} + t \mathbf{r}_w$$

Local NED coordinates $(p_{\text{north}}, p_{\text{east}})$ map directly to WGS84 coordinates $(\phi_{\text{lat}}, \lambda_{\text{lon}})$ using the proving ground origin $(47.397971^\circ\text{N}, 8.546164^\circ\text{E})$:

$$\phi_{\text{lat}} = \phi_0 + \frac{p_{\text{north}}}{111,139}, \quad \lambda_{\text{lon}} = \lambda_0 + \frac{p_{\text{east}}}{111,139 \cdot \cos\phi_0}$$

### 2.3 Recursive 2D Kalman Filter (EKF)
For each detected target, a linear-quadratic estimator tracks the ground coordinates $\mathbf{x} = [p_{\text{north}}, p_{\text{east}}]^T$ with process noise covariance $\mathbf{Q} = 10^{-4} \cdot \mathbf{I}_2$ and range-dependent measurement covariance:

$$\mathbf{R} = \sigma_m^2 \mathbf{I}_2, \quad \sigma_m = \max(0.18, 0.10 + 0.025 \cdot d_{\text{range}})$$

State and error covariance updates follow the Joseph-stabilized formulation:

$$\mathbf{K}_k = \mathbf{P}_{k|k-1} (\mathbf{P}_{k|k-1} + \mathbf{R}_k)^{-1}$$

$$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T$$

### 2.4 Continuous Coordinated Dubins Turn Trajectory
Rather than stopping at lane ends, the UAV executes continuous $180^\circ$ circular arcs ($R = 9.0\text{m}$) connecting adjacent lanes ($18\text{m}$ lane spacing). The centripetal acceleration $a_c$ and required bank angle $\phi_{\text{bank}}$ at turn speed $V_t = 3.6\text{ m/s}$ are strictly bounded:

$$a_c = \frac{V_t^2}{R} = \frac{3.6^2}{9.0} = 1.44\text{ m/s}^2, \quad \phi_{\text{bank}} = \arctan\left(\frac{a_c}{g}\right) = \arctan\left(\frac{1.44}{9.81}\right) \approx 8.35^\circ$$

### 2.5 Phase 2: Adaptive Priority Contact Verification Pass
Upon completing the 5 survey swaths, if any Priority 1 (`P1 - CRITICAL`) contacts have been confirmed, the mission state machine transitions to `VERIFY_CONTACTS`. The drone plans an optimal transit vector directly to the estimated coordinates of the critical contact, descends to an inspection altitude of $9.0\text{m}$ AGL, decelerates to $1.8\text{ m/s}$, and collects high-resolution verification vignettes before commanding Return-to-Launch (RTL).

---

## 3. Dedicated Tactical Ground Control Station (`src/gcs_tkinter.py`)

A responsive, high-contrast tactical GCS engineered in Python Tkinter adhering to an aerospace obsidian/charcoal monotone theme with dark red status indicators:

- **Monotone Color Scheme:** Pitch black (`#000000`), neutral panels (`#070707`, `#0c0c0c`, `#141414`), silver/off-white typography (`#a0a0a0` / `#f0f0f0`), and restrained crimson accents (`#991b1b` / `#b91c1c`). Zero blue or slate tints.
- **Glass-Cockpit Telemetry Ribbon:** 6 primary avionics instruments displaying Altitude (AGL), Groundspeed, Heading (Yaw with Cardinal indicator), Attitude Stability (Pitch/Roll monitoring), NED Coordinates, and Swath Progress pips (`[■■□□□]`).
- **Live Optical Viewport:** Real-time 30 FPS camera feed with boresight HUD, horizon pitch lines, and georeferenced target bounding boxes.
- **Photogrammetric Ground Coverage Heatmap:** As the drone flies, the 4-corner ground projection of the optical field of view is accumulated in real time into an aggregated coverage swath on the 2D map, providing visual proof of $100\%$ proving ground coverage.
- **2D Tactical Canvas:** Dynamic world-to-canvas coordinate mapping supporting interactive pan (`Mouse Drag`), zoom (`Mouse Wheel` / `[-]`/`[+]`), and automated UAV tracking (`[FOLLOW]`). Renders topographic contours, ISO shipping containers, rubble collapse zones, photogrammetric GCPs, planned swaths, Dubins arcs, dual-tone flight breadcrumbs, and confirmed target badges with 1-sigma uncertainty circles.
- **Target Reconnaissance Registry:** In-place Treeview displaying target ID, Priority tag (`[P1]`, `[P2]`, `[P3]`), tactical classification, georeferenced NED coordinates, 1-sigma uncertainty ($\sigma$), observation hits, and lock status.
- **Interactive Target Vignette & Analysis Modal:** Double-clicking any contact row in the registry opens a centered tactical inspection window with a $240 \times 240$ cropped optical vignette, WGS84 GPS coordinates, triage priority, and operational action recommendations.
- **Automated SITREP Export:** Clicking `[SITREP]` generates standardized `MISSION_RECON_SITREP.json` and formatted Markdown `MISSION_RECON_SITREP.md` reports with embedded optical vignettes and full georeferenced inventory.

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
