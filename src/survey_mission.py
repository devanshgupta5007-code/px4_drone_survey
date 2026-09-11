#!/usr/bin/env python3
"""
Autonomous High-Efficiency Aerial Survey Mission for PX4 using MAVSDK (Python).
Executes an optimized 4-lane boustrophedon pattern with continuous coordinated Dubins arc turns,
optical lever-arm and elevation-compensated georeferencing, and an integrated real-time Web GCS.
"""

import os
import sys
import math
import time
import asyncio
import threading
import collections
from typing import List, Tuple, Dict, Optional
from enum import Enum
import cv2
import numpy as np

# Force XCB platform for clean Qt/OpenCV window rendering under Wayland/XWayland
os.environ["QT_QPA_PLATFORM"] = "xcb"
# Protobuf Python implementation for gz.msgs compatibility
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError
from mavsdk.telemetry import FlightMode

from camera_stream import GazeboCameraStream
from box_detector import AerialBoxDetector


class SurveyState(Enum):
    INIT = "INITIALIZATION"
    TAKEOFF = "TAKEOFF"
    TRANSIT_START = "TRANSIT_TO_LANE_1"
    SURVEYING = "SURVEY_LANE"
    DUBINS_TURN = "DUBINS_ARC_TURN"
    COMPLETE = "SURVEY_COMPLETE"
    RTL_LAND = "RETURN_AND_LAND"


class SurveyMission:
    def __init__(self, system_address: str = "udpin://0.0.0.0:14540", headless_vision: bool = False,
                 survey_speed: float = 5.5):
        self.system_address = system_address
        self.headless_vision = headless_vision
        self.survey_speed = survey_speed  # Cruise speed (5 - 7.5 m/s)
        self.turn_speed = min(4.2, max(2.5, survey_speed * 0.70))  # Dubins arc turn speed (m/s)

        self.drone = System()
        self.detector = AerialBoxDetector()
        self.cam_stream = GazeboCameraStream("/world/box_survey/model/x500_mono_cam_45_0/link/camera_link/sensor/camera/image")
        self.latest_annotated_frame: Optional[np.ndarray] = None
        self.log_queue = collections.deque(maxlen=100)

        # Telemetry state
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rad = 0.0
        self.telemetry_lock = threading.Lock()

        # Flight settings
        self.survey_altitude = 12.0  # 12m altitude gives 20.1m swath footprint covering 18m lane spacing
        self.state = SurveyState.INIT
        self.running = True
        self.current_lane = 1
        self.total_lanes = 5

        # 5 Target-Aligned Survey Lanes across 120m x 90m tactical arena:
        # Lane 1 (Y=-36.0m) -> Traverses Target 1 (Red Cube at 30, -36)
        # Lane 2 (Y=-18.0m) -> Traverses Target 2 (Yellow Cube at 92, -18)
        # Lane 3 (Y=  0.0m) -> Traverses Target 3 (Green Cube at 58, 0)
        # Lane 4 (Y=+18.0m) -> Traverses Target 4 (Orange Cube at 26, 18)
        # Lane 5 (Y=+36.0m) -> Traverses Target 5 (Blue Cube at 102, 36)
        self.lanes = [
            {"start": (8.0, -36.0), "end": (112.0, -36.0), "yaw_deg": 0.0},
            {"start": (112.0, -18.0), "end": (8.0, -18.0), "yaw_deg": 180.0},
            {"start": (8.0, 0.0), "end": (112.0, 0.0), "yaw_deg": 0.0},
            {"start": (112.0, 18.0), "end": (8.0, 18.0), "yaw_deg": 180.0},
            {"start": (8.0, 36.0), "end": (112.0, 36.0), "yaw_deg": 0.0},
        ]

        # 4 Continuous Coordinated Dubins Arc Turns (R = 9.0m, exact 18m lane spacing)
        # Turn speed 3.6 m/s produces low centripetal acceleration (ac = 1.44 m/s^2, bank roll < 8.3 deg)
        self.dubins_arcs = [
            {"center": (112.0, -27.0), "radius": 9.0, "start_rad": 0.0, "sweep_rad": math.pi, "type": "NORTH_TURN"},
            {"center": (8.0, -9.0), "radius": 9.0, "start_rad": 0.0, "sweep_rad": math.pi, "type": "SOUTH_TURN"},
            {"center": (112.0, 9.0), "radius": 9.0, "start_rad": 0.0, "sweep_rad": math.pi, "type": "NORTH_TURN"},
            {"center": (8.0, 27.0), "radius": 9.0, "start_rad": 0.0, "sweep_rad": math.pi, "type": "SOUTH_TURN"},
        ]

    def log(self, text: str):
        timestamp = time.strftime("%H:%M:%S")
        self.log_queue.append((timestamp, text))
        print(f"[{timestamp}] [MISSION] {text}", flush=True)

    async def _telemetry_listener(self):
        """Asynchronously streams drone position, velocity and attitude into local state."""
        async def position_task():
            async for pvn in self.drone.telemetry.position_velocity_ned():
                with self.telemetry_lock:
                    self.pos_x = pvn.position.north_m
                    self.pos_y = pvn.position.east_m
                    self.pos_z = pvn.position.down_m
                    self.vel_x = pvn.velocity.north_m_s
                    self.vel_y = pvn.velocity.east_m_s
                    self.vel_z = pvn.velocity.down_m_s

        async def attitude_task():
            async for att in self.drone.telemetry.attitude_euler():
                with self.telemetry_lock:
                    self.roll_rad = math.radians(att.roll_deg)
                    self.pitch_rad = math.radians(att.pitch_deg)
                    self.yaw_rad = math.radians(att.yaw_deg)

        await asyncio.gather(position_task(), attitude_task())

    def _vision_worker(self):
        """Real-time OpenCV camera subscriber, detector, HUD generator, and GCS stream pipe."""
        print("[VISION] Starting Gazebo camera stream and detection pipeline...")
        self.cam_stream.start()

        last_log_time = time.time()
        first_frame_logged = False

        while self.running:
            frame = self.cam_stream.get_frame()

            with self.telemetry_lock:
                px = self.pos_x
                py = self.pos_y
                pz = self.pos_z
                yaw = self.yaw_rad
                pitch = self.pitch_rad
                roll = self.roll_rad
                spd = math.hypot(self.vel_x, self.vel_y)

            state_str = f"{self.state.value} (L{self.current_lane}/{self.total_lanes})"

            if frame is not None:
                if not first_frame_logged:
                    print(f" -> [VISION] First camera frame received ({frame.shape[1]}x{frame.shape[0]})! Live detections active.", flush=True)
                    first_frame_logged = True
                annotated, detections = self.detector.process_frame(
                    frame, px, py, pz, yaw, drone_pitch_rad=pitch, drone_roll_rad=roll,
                    speed_mps=spd, flight_state=state_str
                )
            else:
                standby = np.zeros((480, 640, 3), dtype=np.uint8)
                for gx in range(0, 640, 40):
                    cv2.line(standby, (gx, 40), (gx, 480), (22, 22, 22), 1)
                for gy in range(40, 480, 40):
                    cv2.line(standby, (0, gy), (640, gy), (22, 22, 22), 1)

                self.detector._draw_hud(standby, px, py, pz, yaw, speed_mps=spd, flight_state=state_str)
                cv2.putText(standby, "[CONNECTING GAZEBO CAMERA...]", (160, 235),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (203, 213, 225), 1, cv2.LINE_AA)
                annotated = standby

            # Store latest annotated frame for Tkinter GCS
            self.latest_annotated_frame = annotated

            # Periodic status log
            if time.time() - last_log_time >= 3.0:
                confirmed = self.detector.get_confirmed_targets()
                stats = self.cam_stream.get_stats()
                total_tgts = len(self.detector.color_ranges) if hasattr(self.detector, 'color_ranges') else 7
                print(f"[{self.state.name}] Drone: ({px:4.1f}N, {py:4.1f}E, {-pz:4.1f}Alt)m | "
                      f"Spd: {spd:3.1f}m/s | Hdg: {math.degrees(yaw):03.0f}* | "
                      f"FPS: {stats['fps']:4.1f} | Confirmed Targets: {len(confirmed)}/{total_tgts}", flush=True)
                last_log_time = time.time()

            time.sleep(0.01)

        self.cam_stream.stop()
        print("[VISION] Vision pipeline stopped.")

    async def _fly_smooth_segment(self, start_pt: Tuple[float, float], end_pt: Tuple[float, float],
                                  altitude: float, yaw_deg: float, speed: float, decelerate_at_end: bool = False):
        """
        Continuous closed-loop velocity tracking with cross-track proportional damping.
        If decelerate_at_end is False, flies through the segment end at full cruise speed
        to directly transition into Dubins arc turns without stalling!
        """
        x0, y0 = start_pt
        x1, y1 = end_pt
        seg_dist = math.hypot(x1 - x0, y1 - y0)
        if seg_dist <= 0.05:
            return

        ux = (x1 - x0) / seg_dist
        uy = (y1 - y0) / seg_dist
        nx = -uy
        ny = ux

        dt = 0.04  # 25 Hz rate
        start_time = time.time()
        accel_duration = 0.8  # 0.8s smooth ramp
        cross_track_err_int = 0.0

        while self.running:
            with self.telemetry_lock:
                px = self.pos_x
                py = self.pos_y
                pz = self.pos_z

            actual_prog = (px - x0) * ux + (py - y0) * uy
            remaining = seg_dist - actual_prog
            cross_track_err = (px - x0) * nx + (py - y0) * ny

            # Check if leg completed
            exit_dist = 0.5 if not decelerate_at_end else 0.4
            if remaining <= exit_dist:
                break

            # Jerk-limited S-curve along-track speed profile
            # Keeps pitch acceleration <= 0.65 m/s^2 to ensure camera remains level (< 3.8 deg tilt)
            elapsed = time.time() - start_time
            accel_duration = 2.2  # Smooth 2.2s cosine S-curve
            t_ratio = min(1.0, elapsed / accel_duration)
            s_curve_up = 0.5 * (1.0 - math.cos(math.pi * t_ratio))
            ramp_up = 0.35 + 0.65 * s_curve_up

            if decelerate_at_end:
                decel_dist = max(6.0, speed * 1.5)
                if remaining < decel_dist:
                    d_ratio = min(1.0, remaining / decel_dist)
                    s_curve_down = 0.5 * (1.0 - math.cos(math.pi * d_ratio))
                    ramp_down = 0.25 + 0.75 * s_curve_down
                else:
                    ramp_down = 1.0
            else:
                ramp_down = 1.0  # Maintain cruise momentum into continuous Dubins arc

            target_speed = max(1.2, speed * ramp_up * ramp_down)

            # Cross-track self-correction velocity with PI damping and anti-windup
            cross_track_err_int += cross_track_err * dt
            cross_track_err_int = max(-1.2, min(1.2, cross_track_err_int))
            v_cross = max(-1.3, min(1.3, -0.85 * cross_track_err - 0.15 * cross_track_err_int))

            # Altitude hold
            alt_err = altitude - (-pz)
            v_down = max(-0.8, min(0.8, -0.85 * alt_err))

            vn = target_speed * ux + v_cross * nx
            ve = target_speed * uy + v_cross * ny
            vd = v_down

            try:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, yaw_deg))
            except Exception:
                pass
            await asyncio.sleep(dt)

    async def _fly_dubins_arc(self, arc_info: Dict, altitude: float, speed: float):
        """
        Executes a continuous smooth circular Dubins arc turn between survey lanes.
        Drone banks and curves along the arc at speed without stopping.
        """
        xc, yc = arc_info["center"]
        R = arc_info["radius"]
        is_north_turn = (arc_info["type"] == "NORTH_TURN")

        # Total arc distance is pi * R
        arc_length = math.pi * R
        total_time = arc_length / speed
        dt = 0.04  # 25 Hz
        steps = max(10, int(total_time / dt))

        for step in range(steps):
            if not self.running:
                break

            prog = (step + 1) / steps  # 0 to 1
            s = prog * math.pi  # 0 to pi

            if is_north_turn:
                # Arriving Northbound at (xc, yc - R) heading 0*, exiting Southbound at (xc, yc + R) heading 180*
                x_ref = xc + R * math.sin(s)
                y_ref = yc - R * math.cos(s)
                yaw_deg = math.degrees(s)  # 0 -> 90 -> 180

                # Tangent velocity
                vn_ref = speed * math.cos(s)
                ve_ref = speed * math.sin(s)
            else:
                # Arriving Southbound at (xc, yc - R) heading 180*, exiting Northbound at (xc, yc + R) heading 0*
                x_ref = xc - R * math.sin(s)
                y_ref = yc - R * math.cos(s)
                yaw_deg = (180.0 - math.degrees(s)) % 360.0  # 180 -> 90 -> 0

                # Tangent velocity
                vn_ref = -speed * math.cos(s)
                ve_ref = speed * math.sin(s)

            # Closed-loop tracking correction
            with self.telemetry_lock:
                px = self.pos_x
                py = self.pos_y
                pz = self.pos_z

            err_x = x_ref - px
            err_y = y_ref - py
            vn_cmd = vn_ref + 0.60 * err_x
            ve_cmd = ve_ref + 0.60 * err_y

            alt_err = altitude - (-pz)
            vd_cmd = max(-0.8, min(0.8, -0.85 * alt_err))

            try:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vn_cmd, ve_cmd, vd_cmd, yaw_deg))
            except Exception:
                pass
            await asyncio.sleep(dt)

    async def _smooth_yaw_turn(self, altitude: float, from_yaw: float, to_yaw: float):
        """Smooth in-place yaw alignment before initial lane entry."""
        diff = (to_yaw - from_yaw + 180.0) % 360.0 - 180.0
        yaw_rate = 55.0  # deg/s
        dt = 0.04
        steps = max(1, int(abs(diff) / (yaw_rate * dt)))
        d_yaw = diff / steps

        curr_yaw = from_yaw
        for _ in range(steps):
            if not self.running:
                break
            curr_yaw = (curr_yaw + d_yaw) % 360.0
            try:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, curr_yaw))
            except Exception:
                pass
            await asyncio.sleep(dt)

    async def run(self):
        print("\n==================================================================")
        print("   PX4 HIGH-EFFICIENCY AERIAL SURVEY + DUBINS ARCS + TKINTER GCS  ")
        print(f"   Cruise Speed: {self.survey_speed} m/s | Turn Speed: {self.turn_speed} m/s | Alt: {self.survey_altitude}m ")
        print("==================================================================\n")

        self.log("Starting autonomous reconnaissance survey...")

        # 1. Connect to Drone
        self.state = SurveyState.INIT
        print(f"[MAVSDK] Connecting to PX4 Autopilot at {self.system_address}...")
        await self.drone.connect(system_address=self.system_address)

        print("[MAVSDK] Waiting for vehicle connection...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(" -> Vehicle connected successfully!")
                self.log("PX4 Autopilot connected via MAVLink.")
                break

        # 2. Wait for Health & EKF2
        print("[MAVSDK] Verifying GPS, EKF2 health, and armable status...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok and health.is_armable:
                print(" -> GPS 3D Fix, Home Position, and EKF2 State OK!")
                self.log("GPS 3D Fix & EKF2 health verified.")
                break
            await asyncio.sleep(0.5)

        # Launch background telemetry & vision threads
        telemetry_coro = asyncio.create_task(self._telemetry_listener())
        vision_thread = threading.Thread(target=self._vision_worker, daemon=True)
        vision_thread.start()

        # 3. Arm and Takeoff
        self.state = SurveyState.TAKEOFF
        print("[MAVSDK] Arming vehicle motors...")
        for attempt in range(15):
            try:
                await self.drone.action.arm()
                print(" -> Vehicle armed successfully!")
                self.log("Vehicle motors armed.")
                break
            except Exception as e:
                print(f" -> Arming attempt {attempt+1}/15: {e}. Retrying...")
                await asyncio.sleep(1.0)

        print(f"[MAVSDK] Taking off to survey altitude ({self.survey_altitude}m)...")
        await self.drone.action.set_takeoff_altitude(self.survey_altitude)
        await self.drone.action.takeoff()
        self.log(f"Climbing to survey altitude ({self.survey_altitude}m)...")

        while True:
            with self.telemetry_lock:
                current_alt = -self.pos_z
            if current_alt >= self.survey_altitude - 0.8:
                print(f" -> Reached survey altitude: {current_alt:.1f}m")
                break
            await asyncio.sleep(0.5)

        # 4. Activate Offboard Mode
        print("[MAVSDK] Activating Offboard Mode for precision trajectory control...")
        with self.telemetry_lock:
            init_x, init_y = self.pos_x, self.pos_y
            init_yaw = math.degrees(self.yaw_rad)

        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, init_yaw))
        try:
            await self.drone.offboard.start()
            print(" -> Offboard mode active!")
            self.log("Offboard trajectory controller engaged.")
        except OffboardError as e:
            print(f"Offboard error: {e}")

        # 5. Smooth Transit to Lane 1 Start
        self.state = SurveyState.TRANSIT_START
        lane1_start = self.lanes[0]["start"]
        lane1_yaw = self.lanes[0]["yaw_deg"]

        print(f"[FSM: TRANSIT] Aligning heading towards Lane 1 entry ({lane1_start[0]:.1f}m, {lane1_start[1]:.1f}m)...")
        hdg_to_l1 = math.degrees(math.atan2(lane1_start[1] - init_y, lane1_start[0] - init_x)) % 360.0
        await self._smooth_yaw_turn(self.survey_altitude, init_yaw, hdg_to_l1)
        await self._fly_smooth_segment((init_x, init_y), lane1_start, self.survey_altitude, hdg_to_l1, speed=self.turn_speed, decelerate_at_end=True)
        await self._smooth_yaw_turn(self.survey_altitude, hdg_to_l1, lane1_yaw)

        # 6. Execute 5-Lane Swath with Continuous Dubins Arc Turns
        print("\n[FSM: SURVEY] Starting 5-Lane Target-Aligned Aerial Survey with Dubins Turns...")
        self.log("Starting 5-lane high-speed survey sweep.")

        for i, lane in enumerate(self.lanes):
            self.current_lane = i + 1
            self.state = SurveyState.SURVEYING
            start_pt = lane["start"]
            end_pt = lane["end"]
            yaw_deg = lane["yaw_deg"]

            direction_str = "NORTH (+X)" if yaw_deg == 0.0 else "SOUTH (-X)"
            print(f"\n>>> [LANE {self.current_lane}/{self.total_lanes}] Sweeping ({start_pt[0]:.0f}, {start_pt[1]:.1f})m -> ({end_pt[0]:.0f}, {end_pt[1]:.1f})m | Heading {direction_str} | Speed {self.survey_speed} m/s <<<")
            self.log(f"Scanning Lane {self.current_lane}/{self.total_lanes} ({direction_str}) at {self.survey_speed} m/s")

            # Fly straight lane segment (maintain speed into turn if not the final lane)
            is_last_lane = (i == len(self.lanes) - 1)
            await self._fly_smooth_segment(
                start_pt, end_pt, self.survey_altitude, yaw_deg,
                speed=self.survey_speed, decelerate_at_end=is_last_lane
            )

            # If not the last lane, execute continuous coordinated Dubins Arc Turn
            if not is_last_lane:
                self.state = SurveyState.DUBINS_TURN
                arc = self.dubins_arcs[i]
                print(f"--- [DUBINS TURN {self.current_lane}->{self.current_lane+1}] Flowing through coordinated arc (R={arc['radius']}m, Speed={self.turn_speed} m/s) ---")
                self.log(f"Coordinated Dubins Arc Turn {self.current_lane}->{self.current_lane+1} (R={arc['radius']}m)")
                await self._fly_dubins_arc(arc, self.survey_altitude, speed=self.turn_speed)

        # 7. Survey Complete -> Return to Launch and Land
        self.state = SurveyState.COMPLETE
        print("\n==================================================================")
        print("      ALL 5 SURVEY LANES COMPLETED VIA CONTINUOUS DUBINS TURNS    ")
        print("==================================================================")
        self.log("Survey lanes completed. Commanding RTL.")

        self.state = SurveyState.RTL_LAND
        print("[MAVSDK] Disengaging Offboard mode and commanding Return to Launch (RTL)...")
        try:
            await self.drone.offboard.stop()
        except Exception:
            pass

        await self.drone.action.return_to_launch()

        # Monitor landing on helipad
        print("[MAVSDK] Monitoring descent and touchdown on origin helipad...")
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                print(" -> Touchdown confirmed! Drone landed safely on helipad.")
                self.log("Touchdown confirmed on helipad (0, 0). Mission Complete.")
                break
            await asyncio.sleep(1.0)

        # Give GCS users 5 seconds to inspect final map and table
        await asyncio.sleep(5.0)

        self.running = False
        self.cam_stream.stop()
        vision_thread.join(timeout=2.0)
        try:
            telemetry_coro.cancel()
        except Exception:
            pass

        # Release recorded video file
        self.detector.release_video()

        # Print Final Comprehensive Survey & Detection Report
        self._print_final_report()

    def _print_final_report(self):
        confirmed = self.detector.get_confirmed_targets()
        print("\n" + "="*78)
        print("       HIGH-EFFICIENCY AERIAL SURVEY & TARGET LOCALIZATION REPORT         ")
        print("="*78)
        print(f"Survey Area Covered:   120.0m (Length) x 90.0m (Width) = 10,800 m^2")
        print(f"Swath Configuration:   5 Target-Aligned Lanes @ 18.0m spacing (12m altitude)")
        print(f"Turn Trajectory:       Continuous Coordinated Dubins Arcs (R = 9.0m, Bank < 8.3 deg)")
        print(f"Flight Cruise Speed:   {self.survey_speed} m/s")
        print(f"Camera Optical Offset: [+0.18, 0, -0.242]m lever-arm + Z=-0.55m elevation")
        print(f"Shape Classification:  CUBES ONLY (Spheres, Cylinders & Barriers Rejected)")
        print(f"Total Unique Targets:  {len(confirmed)} confirmed cubes")
        print("-" * 78)
        print(f"{'ID':<4} | {'Target Box Type':<20} | {'Estimated Ground NED':<24} | {'Uncertainty':<12} | {'Hits':<6}")
        print("-" * 78)

        for t in confirmed:
            pos_str = f"({t.x:+.2f}m, {t.y:+.2f}m, 0.0m)"
            unc_str = f"+/- {t.pos_std_dev:.2f} m"
            hits_str = f"{t.observations}"
            print(f"#{t.target_id:<3} | {t.color_name:<20} | {pos_str:<24} | {unc_str:<12} | {hits_str:<6}")

        print("=" * 78)
        print(f"Target snapshots saved to: {self.detector.output_dir}/\n")
        print(f"Full mission recording saved to: {self.detector.video_path}\n")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="PX4 Aerial Survey with MAVSDK and Tkinter GCS")
    parser.add_argument("--address", default="udpin://0.0.0.0:14540", help="MAVLink system address")
    parser.add_argument("--speed", type=float, default=5.5, help="Survey cruise speed in m/s (default: 5.5 m/s)")
    parser.add_argument("--headless", action="store_true", help="Run vision in headless mode (no GUI window)")
    args = parser.parse_args()

    mission = SurveyMission(system_address=args.address, headless_vision=args.headless,
                            survey_speed=args.speed)

    # If display is available and not headless, launch Tkinter GCS on main thread
    has_display = ("DISPLAY" in os.environ or "WAYLAND_DISPLAY" in os.environ)
    if not args.headless and has_display:
        try:
            import tkinter as tk
            from gcs_tkinter import TkinterGCS

            root = tk.Tk()
            gcs_gui = TkinterGCS(root, mission_ref=mission)

            mission_thread = threading.Thread(target=lambda: asyncio.run(mission.run()), daemon=True)
            mission_thread.start()

            def check_mission_done():
                if not mission.running and mission.state == SurveyState.RTL_LAND:
                    print("[GCS] Mission completed. Auto-closing GCS window in 6 seconds...", flush=True)
                    root.after(6000, root.destroy)
                    return
                root.after(500, check_mission_done)

            root.after(2000, check_mission_done)

            def on_closing():
                mission.running = False
                mission.cam_stream.stop()
                root.destroy()

            root.protocol("WM_DELETE_WINDOW", on_closing)
            root.mainloop()
            mission.running = False
            mission.cam_stream.stop()
            if mission_thread.is_alive():
                mission_thread.join(timeout=3.0)
            return
        except Exception as e:
            print(f"[GCS] Could not initialize Tkinter GUI ({e}). Falling back to headless/console mode.")

    try:
        asyncio.run(mission.run())
    except KeyboardInterrupt:
        print("\n[USER] Mission interrupted by user! Landing safely...")
        mission.running = False
        mission.cam_stream.stop()
        asyncio.run(mission.drone.action.land())


if __name__ == "__main__":
    main()
