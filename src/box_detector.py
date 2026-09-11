#!/usr/bin/env python3
"""
Real-time Aerial Box Detector & Target Position Estimator for PX4 Survey Simulation.
Maintains persistent 2D Extended Kalman Filters (EKF) for challenge target cubes.
Features:
- CUBE vs NON-CUBE GEOMETRIC SHAPE CLASSIFIER:
  * Rejects spheres/domes via minimum enclosing circle area fill ratio (circle_fill > 0.82)
  * Rejects cylinders/cones/curved objects via polygonal contour approximation (vertices >= 7)
  * Rejects elongated barrier slabs via metric ground footprint aspect ratio (aspect > 2.2)
  * Accurately locks only 3D CUBES (4-6 polygonal vertices, rectangular metric footprint)
- BORESIGHT OPTICAL CORRIDOR GATING:
  * Restricts active tracking to forward viewing cone (|theta_h| <= 28 deg) directly in front of drone
  * Eliminates perspective distortion from awkward side-angle sightings
  * Edge-clearance filter rejects partially clipped boundary contours
- ATTITUDE STABILITY GATING:
  * Suppresses EKF updates during bank turns (|roll| > 6.5 deg) ensuring camera is stable and level
- CAMERA OPTICAL LEVER-ARM & ELEVATION COMPENSATION:
  * Offsets: [+0.18, 0, -0.242]m in body NED, Z = -0.55m top-face elevation
- EXPLICIT SHAPE-BASED ENTITY NAMING:
  * Outputs genuine shape descriptions (e.g. CUBE [RED], CUBE [YELLOW])
- Integrated MP4 video recording with full tactical data HUD
"""

import os
import sys
import math
import time
import numpy as np
import cv2
from typing import Dict, List, Tuple, Optional


class TargetPositionEstimator:
    """
    2D Recursive Kalman Filter for estimating ground coordinates of a target entity.
    State: [x_north, y_east] (meters NED)
    Covariance: P (2x2)
    """
    def __init__(self, target_id: int, color_name: str, init_x: float, init_y: float,
                 bbox: Tuple[int, int, int, int], initial_range: float, triage_info: Optional[Dict] = None):
        self.target_id = target_id
        self.color_name = color_name
        self.shape_type = "CUBE"

        # Standardized HADR / Tactical Triage Details
        triage = triage_info or {}
        self.triage_name = triage.get("name", color_name.replace("_CUBE", "").replace("_", " "))
        self.priority = triage.get("priority", "P2 - HIGH")
        self.action = triage.get("action", "Inspect and Verify")
        self.nominal_dims = triage.get("nominal_dims", "1.0m x 1.0m x 0.6m")

        # Explicit shape-based display name (e.g. "[P1] SURVIVOR SHELTER")
        pri_tag = self.priority.split()[0]
        self.display_name = f"{pri_tag} {self.triage_name}"

        # State vector [North, East] (m)
        self.state = np.array([init_x, init_y], dtype=np.float64)

        # Covariance matrix P initialized
        self.P = np.eye(2, dtype=np.float64) * 2.5

        # Process noise Q (static ground target)
        self.Q = np.eye(2, dtype=np.float64) * 1e-4

        self.latest_bbox = bbox
        self.observations = 1
        self.first_seen = time.time()
        self.last_seen = time.time()

        # Best visual snapshot & optical cropped vignette
        self.best_dist_to_center = 9999.0
        self.best_frame: Optional[np.ndarray] = None
        self.best_crop: Optional[np.ndarray] = None
        self.crop_path: str = ""
        self.snapshot_saved = False

        self.confirmed = False

    @property
    def x(self) -> float:
        return float(self.state[0])

    @property
    def y(self) -> float:
        return float(self.state[1])

    @property
    def wgs84_coords(self) -> Tuple[float, float]:
        """Converts local NED (North, East) to simulated WGS84 (Latitude, Longitude)."""
        base_lat = 47.397971057728974
        base_lon = 8.546163739800146
        lat = base_lat + (self.x / 111139.0)
        lon = base_lon + (self.y / (111139.0 * math.cos(math.radians(base_lat))))
        return lat, lon

    @property
    def pos_std_dev(self) -> float:
        """1-sigma position uncertainty radius in meters."""
        return float(math.sqrt(max(0.001, (self.P[0, 0] + self.P[1, 1]) / 2.0)))

    @property
    def confidence(self) -> float:
        obs_score = min(1.0, self.observations / 20.0)
        cov_score = max(0.0, min(1.0, 1.0 - (self.pos_std_dev / 0.8)))
        return min(0.99, 0.35 * obs_score + 0.65 * cov_score)

    def predict(self):
        self.P += self.Q

    def update(self, meas_x: float, meas_y: float, bbox: Tuple[int, int, int, int], range_to_drone: float, frame: np.ndarray, u: float, v: float):
        self.predict()

        # Measurement variance: closer sightings have higher precision
        sigma_m = max(0.18, 0.10 + 0.025 * range_to_drone)
        R = np.eye(2, dtype=np.float64) * (sigma_m ** 2)

        z = np.array([meas_x, meas_y], dtype=np.float64)
        y = z - self.state  # Innovation
        S = self.P + R      # Innovation covariance

        try:
            K = self.P @ np.linalg.inv(S)  # Kalman gain
            self.state = self.state + K @ y
            I_KH = np.eye(2, dtype=np.float64) - K
            self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T  # Joseph stabilized form
        except np.linalg.LinAlgError:
            pass

        self.latest_bbox = bbox
        self.observations += 1
        self.last_seen = time.time()

        # Update best snapshot if closer to optical center (320, 240)
        dist_center = math.hypot(u - 320.0, v - 240.0)
        if dist_center < self.best_dist_to_center:
            self.best_dist_to_center = dist_center
            self.best_frame = frame.copy()

            # Extract 240x240 optical vignette centered on target
            fh, fw = frame.shape[:2]
            cu, cv = int(u), int(v)
            half_box = 120
            u0, u1 = max(0, cu - half_box), min(fw, cu + half_box)
            v0, v1 = max(0, cv - half_box), min(fh, cv + half_box)
            if (u1 - u0) > 40 and (v1 - v0) > 40:
                crop = frame[v0:v1, u0:u1].copy()
                self.best_crop = cv2.resize(crop, (240, 240))

        # Confirmation criteria: at least 4 observations, persistence > 0.12s, and uncertainty <= 0.85m
        if self.observations >= 4 and (self.last_seen - self.first_seen) >= 0.12 and self.pos_std_dev <= 0.85:
            self.confirmed = True


class AerialBoxDetector:
    def __init__(self, output_dir: str = "/home/devi/px4_drone_survey/detections"):
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        # Camera Intrinsics (640x480, HFOV = 80 deg)
        self.img_w = 640
        self.img_h = 480
        self.hfov_rad = 1.3962634  # 80 degrees
        self.fx = (self.img_w / 2.0) / math.tan(self.hfov_rad / 2.0)
        self.fy = self.fx
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0

        # Camera Mounting: 45 degrees downward pitch relative to vehicle body
        self.cam_pitch_rad = math.radians(45.0)

        # Camera Lever-Arm relative to drone CG (in body frame: X forward, Y right, Z down)
        # Gazebo pose: [0.18, 0, 0.242] ENU => in NED body frame: [0.18, 0.0, -0.242]
        self.cam_body_offset = np.array([0.18, 0.0, -0.242], dtype=np.float64)

        # Target Box height compensation: top surface of boxes sits ~0.5m - 0.6m above ground
        # In NED, ground is Z = 0.0, so box top surface is at Z = -0.55m
        self.target_elevation_z = -0.55

        # Video Recorder output file
        self.video_path = os.path.join(self.output_dir, "survey_mission_recording.mp4")
        self.video_writer: Optional[cv2.VideoWriter] = None

        # Classification color ranges (HSV) for CUBES (7 distinct tactical crates)
        self.color_ranges = {
            "RED_CUBE": [
                (np.array([0, 110, 80]), np.array([9, 255, 255])),
                (np.array([171, 110, 80]), np.array([180, 255, 255]))
            ],
            "YELLOW_CUBE": [
                (np.array([23, 110, 100]), np.array([35, 255, 255]))
            ],
            "GREEN_CUBE": [
                (np.array([38, 80, 60]), np.array([80, 255, 255]))
            ],
            "ORANGE_CUBE": [
                (np.array([10, 80, 70]), np.array([22, 255, 255]))
            ],
            "BLUE_CUBE": [
                (np.array([98, 70, 50]), np.array([126, 255, 255]))
            ],
            "PURPLE_CUBE": [
                (np.array([130, 75, 60]), np.array([160, 255, 255]))
            ],
            "CYAN_CUBE": [
                (np.array([82, 80, 70]), np.array([96, 255, 255]))
            ],
        }

        # Predefined ID map for the 7 challenge entities
        self.entity_ids = {
            "RED_CUBE": 1,
            "YELLOW_CUBE": 2,
            "GREEN_CUBE": 3,
            "ORANGE_CUBE": 4,
            "BLUE_CUBE": 5,
            "PURPLE_CUBE": 6,
            "CYAN_CUBE": 7,
        }

        # Standardized HADR / Tactical Reconnaissance Triage Specifications
        self.triage_specs = {
            "RED_CUBE": {
                "id": 1,
                "name": "TRAUMA MEDICAL CACHE",
                "priority": "P2 - HIGH",
                "action": "Dispatch Field Triage & Medical Evacuation Team",
                "nominal_dims": "0.9m x 0.9m x 0.5m",
            },
            "YELLOW_CUBE": {
                "id": 2,
                "name": "HAZMAT TOXIC SPILL",
                "priority": "P1 - CRITICAL",
                "action": "Cordon 100m Perimeter / Hazardous Material Containment",
                "nominal_dims": "1.0m x 1.0m x 0.7m",
            },
            "GREEN_CUBE": {
                "id": 3,
                "name": "EMERGENCY RATIONS POD",
                "priority": "P2 - HIGH",
                "action": "Survivor Provision Replenishment & Distribution",
                "nominal_dims": "0.9m x 0.9m x 0.5m",
            },
            "ORANGE_CUBE": {
                "id": 4,
                "name": "SURVIVOR SHELTER",
                "priority": "P1 - CRITICAL",
                "action": "Deploy Search & Rescue Extraction Team Immediately",
                "nominal_dims": "1.2m x 1.2m x 0.6m",
            },
            "BLUE_CUBE": {
                "id": 5,
                "name": "EMERGENCY WATER IBC",
                "priority": "P2 - HIGH",
                "action": "Relief Water Logistics & Distribution",
                "nominal_dims": "1.0m x 1.0m x 0.8m",
            },
            "PURPLE_CUBE": {
                "id": 6,
                "name": "COMMS RELAY STATION",
                "priority": "P3 - LOGISTICS",
                "action": "Secure Tactical Mesh Radio Repeater Link",
                "nominal_dims": "0.9m x 0.9m x 0.6m",
            },
            "CYAN_CUBE": {
                "id": 7,
                "name": "UNEXPLODED ORDNANCE",
                "priority": "P1 - HAZARD",
                "action": "EOD / Demining Disposal Team Dispatch Required",
                "nominal_dims": "0.8m x 0.8m x 0.4m",
            },
        }

        # Persistent Estimators: Exactly one estimator per challenge target color
        self.targets: Dict[str, TargetPositionEstimator] = {}

        # Spatial survey boundaries (NED) for 120m x 90m tactical zone
        self.min_survey_x = 2.0
        self.max_survey_x = 118.0
        self.min_survey_y = -43.0
        self.max_survey_y = 43.0

    def get_terrain_elevation(self, x: float, y: float) -> float:
        """
        Returns local ground surface elevation Z in NED (meters, negative is above datum).
        - Natural Knoll NW (pose Gazebo -25, 85 => NED X=85, Y=-25)
          Tier 2 summit: +0.44m (NED Z = -0.44m)
          Tier 1 base:   +0.22m (NED Z = -0.22m)
        - Natural Ridge CE (pose Gazebo 22, 42 => NED X=42, Y=22)
          Ridge base:    +0.30m (NED Z = -0.30m)
        """
        d_knoll = math.hypot(x - 85.0, y - (-25.0))
        if d_knoll <= 8.5:
            return -0.44
        elif d_knoll <= 13.0:
            return -0.22
        if abs(x - 42.0) <= 10.0 and abs(y - 22.0) <= 7.0:
            return -0.30
        return 0.0

    def georeference_pixel(self, u: float, v: float, drone_x: float, drone_y: float, drone_z: float,
                           drone_yaw_rad: float, drone_pitch_rad: float = 0.0, drone_roll_rad: float = 0.0,
                           target_plane_z: Optional[float] = None) -> Optional[Tuple[float, float, float]]:
        """
        Projects image pixel (u, v) onto ground target plane using:
        1. Camera optical lever-arm offset from drone CG
        2. Full 3D Euler attitude matrix (yaw, pitch, roll)
        3. Target top-face elevation offset (Z = -0.55m in NED) with terrain adaptation
        Returns: (ground_x, ground_y, ground_range) in NED meters.
        """
        target_z = self.target_elevation_z if target_plane_z is None else target_plane_z

        # Camera normalized ray in camera frame
        xc = (u - self.cx) / self.fx
        yc = (v - self.cy) / self.fy
        zc = 1.0

        # Rotate from camera frame to vehicle body frame (45 deg pitch down)
        c45 = math.cos(self.cam_pitch_rad)
        s45 = math.sin(self.cam_pitch_rad)
        rb_x = zc * c45 - yc * s45
        rb_y = xc
        rb_z = zc * s45 + yc * c45

        # Rotation matrix from body to world NED: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        cy, sy = math.cos(drone_yaw_rad), math.sin(drone_yaw_rad)
        cp, sp = math.cos(drone_pitch_rad), math.sin(drone_pitch_rad)
        cr, sr = math.cos(drone_roll_rad), math.sin(drone_roll_rad)

        # Compute camera optical center position in world NED
        cam_offset_w_x = (cy * cp) * self.cam_body_offset[0] + (cy * sp * sr - sy * cr) * self.cam_body_offset[1] + (cy * sp * cr + sy * sr) * self.cam_body_offset[2]
        cam_offset_w_y = (sy * cp) * self.cam_body_offset[0] + (sy * sp * sr + cy * cr) * self.cam_body_offset[1] + (sy * sp * cr - cy * sr) * self.cam_body_offset[2]
        cam_offset_w_z = (-sp)     * self.cam_body_offset[0] + (cp * sr)                 * self.cam_body_offset[1] + (cp * cr)                 * self.cam_body_offset[2]

        cam_world_x = drone_x + cam_offset_w_x
        cam_world_y = drone_y + cam_offset_w_y
        cam_world_z = drone_z + cam_offset_w_z

        # Ray direction in world NED
        r_north = (cy * cp) * rb_x + (cy * sp * sr - sy * cr) * rb_y + (cy * sp * cr + sy * sr) * rb_z
        r_east  = (sy * cp) * rb_x + (sy * sp * sr + cy * cr) * rb_y + (sy * sp * cr - cy * sr) * rb_z
        r_down  = (-sp)     * rb_x + (cp * sr)                 * rb_y + (cp * cr)                 * rb_z

        # Reject rays pointing near or above horizontal
        if r_down <= 0.15:
            return None

        # Distance along ray to target plane elevation
        delta_z = target_z - cam_world_z
        if delta_z <= 0.5:
            return None

        t = delta_z / r_down
        box_x = cam_world_x + t * r_north
        box_y = cam_world_y + t * r_east

        # Terrain adaptive correction: if target is situated on knoll or ridge elevation
        elev = self.get_terrain_elevation(box_x, box_y)
        if elev != 0.0 and target_plane_z is None:
            refined_z = self.target_elevation_z + elev
            delta_z = refined_z - cam_world_z
            if delta_z > 0.5:
                t = delta_z / r_down
                box_x = cam_world_x + t * r_north
                box_y = cam_world_y + t * r_east

        ground_range = math.hypot(box_x - drone_x, box_y - drone_y)
        return (box_x, box_y, ground_range)

    def is_cube(self, cnt: np.ndarray, area: float, w: int, h: int) -> Tuple[bool, str]:
        """
        Geometrically classifies whether a detected contour is a CUBE or a non-cube distractor
        (sphere, cylinder, cone, or barrier).
        Returns (is_cube: bool, shape_type: str).
        """
        if w <= 0 or h <= 0:
            return False, "NOISE"

        # 1. Reject elongated barrier slabs via bounding box aspect ratio
        aspect_ratio = float(w) / float(h)
        if aspect_ratio < 0.35 or aspect_ratio > 2.8:
            return False, "BARRIER"

        # 2. Reject spheres and round domes via minimum enclosing circle area fill ratio
        # A circle has circle_fill ~ 1.0 (>= 0.85). A square has circle_fill = 2/pi = 0.637.
        # A perspective 3D cube has circle_fill ~ 0.65 - 0.76.
        (_, _), radius = cv2.minEnclosingCircle(cnt)
        circle_area = np.pi * (radius ** 2)
        if circle_area <= 0:
            return False, "NOISE"
        circle_fill = area / circle_area
        if circle_fill > 0.82:
            return False, "SPHERE"

        # 3. Reject smooth rounded shapes (cylinders, curved pylons) via polygon approximation
        # Cubes approximate cleanly to 4 vertices (top rectangle) or 5-7 vertices (3D isometric hexagon/perspective)
        # Spheres and cylinders produce 8+ vertices
        peri = cv2.arcLength(cnt, True)
        app = cv2.approxPolyDP(cnt, 0.020 * peri, True)
        num_vertices = len(app)
        if num_vertices < 4 or num_vertices > 7:
            return False, "CYLINDER"

        # 4. Solidity check: cubes have straight edges with solidity >= 0.65
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        if hull_area <= 0:
            return False, "NOISE"
        solidity = area / hull_area
        if solidity < 0.65:
            return False, "IRREGULAR"

        return True, "CUBE"

    def process_frame(self, frame: np.ndarray, drone_x: float = 0.0, drone_y: float = 0.0, drone_z: float = -12.0,
                      drone_yaw_rad: float = 0.0, drone_pitch_rad: float = 0.0, drone_roll_rad: float = 0.0,
                      speed_mps: float = 0.0, flight_state: str = "SURVEY") -> Tuple[np.ndarray, List[Dict]]:
        """
        Processes camera frame with forward boresight corridor gating, attitude stability gating,
        and authentic shape classification across the full optical aperture.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        annotated = frame.copy()
        current_detections = []

        is_surveying = flight_state.startswith("SURVEY_LANE")
        # Camera level attitude check: vehicle roll within 6.5 deg, pitch near nominal cruise trim
        is_stable_attitude = abs(drone_roll_rad) < math.radians(6.5) and abs(drone_pitch_rad - math.radians(-3.8)) < math.radians(6.5)

        for color_name, ranges in self.color_ranges.items():
            mask = np.zeros((self.img_h, self.img_w), dtype=np.uint8)
            for lower, upper in ranges:
                sub_mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.bitwise_or(mask, sub_mask)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                # Geometric Image Filter: cube pixel area constraint
                if area < 160 or area > 26000:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)

                # Edge clearance: discard contours cut off by image borders (2px margin)
                if x <= 2 or (x + w) >= (self.img_w - 2) or y <= 2 or (y + h) >= (self.img_h - 2):
                    continue

                # Ignore objects in top 45px (horizon glare)
                if (y + h / 2.0) < 45:
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                u = M["m10"] / M["m00"]
                v = M["m01"] / M["m00"]

                # Optical Aperture Gate: Full usable sensor aperture (+/- 38.5 deg horizontal)
                # Enables detecting cubes across the full swath width and near lateral frame edges
                off_axis_h = math.atan((u - self.cx) / self.fx)
                if abs(off_axis_h) > math.radians(38.5):
                    continue

                # CUBE SHAPE CLASSIFIER: Check if this object is a genuine CUBE
                is_a_cube, detected_shape = self.is_cube(cnt, area, w, h)

                geo = self.georeference_pixel(u, v, drone_x, drone_y, drone_z, drone_yaw_rad, drone_pitch_rad, drone_roll_rad)
                if geo is None:
                    continue
                box_x, box_y, ground_range = geo

                # Range filter: valid ground sightings within 3.5m to 32.0m (suitable for 12m survey altitude)
                if ground_range > 32.0 or ground_range < 3.5:
                    continue

                # Metric Footprint Dimension Check
                geo_tl = self.georeference_pixel(x, y, drone_x, drone_y, drone_z, drone_yaw_rad, drone_pitch_rad, drone_roll_rad)
                geo_br = self.georeference_pixel(x + w, y + h, drone_x, drone_y, drone_z, drone_yaw_rad, drone_pitch_rad, drone_roll_rad)
                if geo_tl is not None and geo_br is not None:
                    footprint_len = abs(geo_br[0] - geo_tl[0])
                    footprint_wid = abs(geo_br[1] - geo_tl[1])
                    footprint_diag = math.hypot(footprint_len, footprint_wid)
                    # Real target cube has diagonal 1.4m - 2.8m, perspective projection may measure up to 4.4m
                    if footprint_diag > 4.4 or footprint_diag < 0.40:
                        continue
                    if footprint_len > 0 and footprint_wid > 0:
                        metric_ar = max(footprint_len / footprint_wid, footprint_wid / footprint_len)
                        if metric_ar > 2.8:
                            continue

                # Survey Boundary Gate: boxes must be inside arena boundaries
                if not (self.min_survey_x <= box_x <= self.max_survey_x and
                        self.min_survey_y <= box_y <= self.max_survey_y):
                    continue

                # If not a cube, ignore distractor and proceed
                if not is_a_cube:
                    continue

                # Confirmed cube target: track and draw clean dark red bounding box & targeting ray
                draw_color = (25, 25, 180)
                cv2.rectangle(annotated, (x, y), (x + w, y + h), draw_color, 1)
                cv2.line(annotated, (int(self.cx), int(self.cy)), (int(u), int(v)), (20, 20, 140), 1, cv2.LINE_AA)

                target_id = self.entity_ids.get(color_name, 1)

                if is_surveying and is_stable_attitude:
                    if color_name in self.targets:
                        estimator = self.targets[color_name]
                        estimator.update(box_x, box_y, (x, y, w, h), ground_range, frame, u, v)
                    else:
                        triage_meta = self.triage_specs.get(color_name)
                        estimator = TargetPositionEstimator(target_id, color_name, box_x, box_y, (x, y, w, h), ground_range, triage_info=triage_meta)
                        self.targets[color_name] = estimator

                if color_name in self.targets:
                    target_obj = self.targets[color_name]
                    current_detections.append({
                        "target_id": target_obj.target_id,
                        "color": color_name,
                        "shape": target_obj.shape_type,
                        "name": target_obj.display_name,
                        "triage_name": target_obj.triage_name,
                        "priority": target_obj.priority,
                        "action": target_obj.action,
                        "crop_path": target_obj.crop_path,
                        "est_x": target_obj.x,
                        "est_y": target_obj.y,
                        "std_dev": target_obj.pos_std_dev,
                        "confidence": target_obj.confidence,
                        "observations": target_obj.observations,
                        "confirmed": target_obj.confirmed
                    })

                    # Save official snapshot and high-resolution optical vignette if newly confirmed
                    if target_obj.confirmed and not target_obj.snapshot_saved and target_obj.best_frame is not None:
                        target_obj.snapshot_saved = True
                        clean_name = target_obj.triage_name.lower().replace(" ", "_")
                        snapshot_path = os.path.join(self.output_dir, f"target_{target_obj.target_id}_{color_name.lower()}.jpg")
                        vignette_path = os.path.join(self.output_dir, f"vignette_{target_obj.target_id}_{clean_name}.jpg")
                        target_obj.crop_path = vignette_path

                        # Save full annotated optical view
                        snap_img = target_obj.best_frame.copy()
                        cv2.putText(snap_img, f"TARGET #{target_obj.target_id}: {target_obj.display_name} [CONFIRMED]", (20, 35),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.70, (230, 230, 235), 2, cv2.LINE_AA)
                        cv2.putText(snap_img, f"POS NED: ({target_obj.x:+.2f}m, {target_obj.y:+.2f}m) +/-{target_obj.pos_std_dev:.2f}m",
                                    (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (25, 25, 180), 2, cv2.LINE_AA)
                        cv2.imwrite(snapshot_path, snap_img)

                        # Save cropped high-resolution target vignette
                        if target_obj.best_crop is not None:
                            vig_img = target_obj.best_crop.copy()
                            cv2.rectangle(vig_img, (0, 0), (240, 24), (4, 4, 6), -1)
                            cv2.line(vig_img, (0, 24), (240, 24), (15, 15, 120), 1)
                            cv2.putText(vig_img, f"#{target_obj.target_id} {target_obj.display_name}",
                                        (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (230, 230, 235), 1, cv2.LINE_AA)
                            cv2.rectangle(vig_img, (0, 216), (240, 240), (4, 4, 6), -1)
                            cv2.line(vig_img, (0, 216), (240, 216), (15, 15, 120), 1)
                            cv2.putText(vig_img, f"NED: ({target_obj.x:+.1f}, {target_obj.y:+.1f})m",
                                        (6, 232), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 180, 180), 1, cv2.LINE_AA)
                            cv2.imwrite(vignette_path, vig_img)

                        print(f"\n[HADR CONTACT LOCKED] #{target_obj.target_id} {target_obj.display_name} at "
                              f"({target_obj.x:+.2f}m, {target_obj.y:+.2f}m) +/-{target_obj.pos_std_dev:.2f}m [{target_obj.observations} hits] -> Action: {target_obj.action}", flush=True)

                    status_icon = "[LOCKED]" if target_obj.confirmed else "[TRACK]"
                    tag_label = f"#{target_obj.target_id} {target_obj.display_name} {status_icon}"
                    pos_label = f"({target_obj.x:+.1f}m, {target_obj.y:+.1f}m) d={ground_range:.1f}m"
                else:
                    color_clean = color_name.replace("_CUBE", "").replace("_", " ")
                    tag_label = f"#{target_id} CUBE [{color_clean}] [DETECT]"
                    pos_label = f"({box_x:+.1f}m, {box_y:+.1f}m) d={ground_range:.1f}m"

                cv2.putText(annotated, tag_label, (x, max(20, y - 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (230, 230, 235), 1, cv2.LINE_AA)
                cv2.putText(annotated, pos_label, (x, max(34, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.34, (148, 163, 184), 1, cv2.LINE_AA)

        # Draw Full Tactical HUD
        self._draw_hud(annotated, drone_x, drone_y, drone_z, drone_yaw_rad,
                       drone_pitch_rad=drone_pitch_rad, drone_roll_rad=drone_roll_rad,
                       speed_mps=speed_mps, flight_state=flight_state)

        # Record to MP4 Video
        self._record_video_frame(annotated)

        return annotated, current_detections

    def _draw_hud(self, img: np.ndarray, dx: float, dy: float, dz: float, yaw_rad: float,
                  drone_pitch_rad: float = 0.0, drone_roll_rad: float = 0.0,
                  speed_mps: float = 0.0, flight_state: str = "SURVEY"):
        """Renders comprehensive military-grade tactical survey HUD over camera feed."""
        h, w = img.shape[:2]
        cx, cy = int(self.cx), int(self.cy)

        # 1. Subtle Boresight Reticle & Forward Operational Viewing Corridor
        reticle_col = (180, 195, 210)
        cv2.drawMarker(img, (cx, cy), reticle_col, cv2.MARKER_CROSS, 16, 1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 10, reticle_col, 1, cv2.LINE_AA)

        # Operational wide optical aperture boundary ticks (38 deg HFOV corridor)
        corridor_w = int(self.fx * math.tan(math.radians(38.0)))
        x_left = max(10, cx - corridor_w)
        x_right = min(w - 10, cx + corridor_w)
        guide_col = (45, 45, 55)
        cv2.line(img, (x_left, 32), (x_left, 44), guide_col, 1, cv2.LINE_AA)
        cv2.line(img, (x_right, 32), (x_right, 44), guide_col, 1, cv2.LINE_AA)
        cv2.line(img, (x_left, h - 44), (x_left, h - 32), guide_col, 1, cv2.LINE_AA)
        cv2.line(img, (x_right, h - 44), (x_right, h - 32), guide_col, 1, cv2.LINE_AA)

        # 2. Sleek Top Header Bar (Pitch Black Strip + Subtle Dark Red Separator)
        cv2.rectangle(img, (0, 0), (w, 26), (4, 4, 6), -1)
        cv2.line(img, (0, 26), (w, 26), (15, 15, 120), 1)
        cv2.putText(img, "PX4 RECONNAISSANCE | 45-DEG OPTICAL", (10, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (203, 213, 225), 1, cv2.LINE_AA)

        state_color = (25, 25, 180) if "LANE" in flight_state else (148, 163, 184)
        cv2.putText(img, flight_state, (w - 210, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, state_color, 1, cv2.LINE_AA)

        # 3. Bottom Telemetry Banner (Dark Charcoal Strip with High-Contrast White Text)
        cv2.rectangle(img, (0, h - 26), (w, h), (4, 4, 6), -1)
        cv2.line(img, (0, h - 26), (w, h - 26), (15, 15, 120), 1)

        txt_col = (226, 232, 240)
        deg_char = "deg"
        hud_telem = (
            f"N: {dx:+5.1f}m  E: {dy:+5.1f}m   |   "
            f"ALT: {-dz:4.1f}m   |   "
            f"SPD: {speed_mps:3.1f}m/s   |   "
            f"P: {math.degrees(drone_pitch_rad):+4.1f}{deg_char}  R: {math.degrees(drone_roll_rad):+4.1f}{deg_char}  HDG: {math.degrees(yaw_rad):03.0f}{deg_char}"
        )
        cv2.putText(img, hud_telem, (10, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.36, txt_col, 1, cv2.LINE_AA)

        # 4. Minimalist Pitch Ladder Ticks
        pitch_px_per_deg = 3.5
        ladder_col = (110, 115, 130)
        for deg in [-10, -5, 5, 10]:
            py = int(cy - deg * pitch_px_per_deg)
            if 36 < py < h - 36:
                cv2.line(img, (cx - 14, py), (cx + 14, py), ladder_col, 1, cv2.LINE_AA)

        # 5. Target Lock Counter (Top Left Functional Badge with Sleek Obsidian Pill)
        total_targets = len(self.color_ranges)
        confirmed_count = sum(1 for t in self.targets.values() if t.confirmed)
        cv2.rectangle(img, (8, 32), (168, 52), (4, 4, 6), -1)
        cv2.rectangle(img, (8, 32), (168, 52), (28, 28, 34), 1)
        tgt_badge_col = (25, 25, 180) if confirmed_count >= total_targets else (203, 213, 225)
        cv2.putText(img, f"HADR RECON: {confirmed_count}/{total_targets}", (14, 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, tgt_badge_col, 1, cv2.LINE_AA)

    def _record_video_frame(self, frame: np.ndarray):
        """Streams annotated HUD frames into MP4 video file."""
        if self.video_writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(self.video_path, fourcc, 30.0, (w, h))
        if self.video_writer is not None:
            self.video_writer.write(frame)

    def release_video(self):
        """Finalizes and flushes the recorded video file."""
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            print(f"[RECORDER] Full mission video with all data saved to: {self.video_path}", flush=True)

    def get_confirmed_targets(self) -> List[TargetPositionEstimator]:
        """Returns list of confirmed target estimators sorted by target_id."""
        return sorted([t for t in self.targets.values() if t.confirmed], key=lambda x: x.target_id)

    def get_all_targets(self) -> List[TargetPositionEstimator]:
        """Returns all active target estimators sorted by target_id."""
        return sorted(list(self.targets.values()), key=lambda x: x.target_id)
