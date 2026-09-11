#!/usr/bin/env python3
import os
import sys
import time
import math
import threading
import tkinter as tk
import cv2
import numpy as np

sys.path.insert(0, '/home/devi/px4_drone_survey/src')
from gcs_tkinter import TkinterGCS

class MockStream:
    def __init__(self):
        sample_path = '/home/devi/px4_drone_survey/detections/sample_survey_hud_frame.jpg'
        if os.path.exists(sample_path):
            self.frame = cv2.imread(sample_path)
        else:
            self.frame = None
        if self.frame is None:
            self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(self.frame, "HUD OPTICAL STREAM", (180, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

    def get_frame(self):
        return self.frame

    def get_stats(self):
        return {"fps": 30.2}

class MockTarget:
    def __init__(self, tid, name, triage_name, pri, action, dims, x, y, sigma, hits, rgb_color):
        self.target_id = tid
        self.color_name = name
        self.display_name = name.replace("_", " ")
        self.triage_name = triage_name
        self.priority = pri
        self.action = action
        self.nominal_dims = dims
        self.x = x
        self.y = y
        self.pos_std_dev = sigma
        self.observations = hits
        self.confirmed = True

        # Compute WGS84
        base_lat = 47.3979710
        base_lon = 8.5461640
        m_per_lat = 111139.0
        m_per_lon = 111139.0 * math.cos(math.radians(base_lat))
        self.wgs84_coords = (base_lat + self.x / m_per_lat, base_lon + self.y / m_per_lon)

        # Generate synthetic optical crop vignette
        crop_img = np.full((160, 160, 3), 18, dtype=np.uint8)
        cv2.rectangle(crop_img, (20, 20), (140, 140), rgb_color, -1)
        cv2.rectangle(crop_img, (20, 20), (140, 140), (255, 255, 255), 2)
        cv2.putText(crop_img, f"#{tid:02d} {pri.split()[0]}", (28, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        self.best_crop = crop_img
        self.crop_path = f"/home/devi/px4_drone_survey/detections/vignette_{tid}_{name.lower()}.jpg"

class MockDetector:
    def __init__(self):
        self.output_dir = "/home/devi/px4_drone_survey/detections"
        self.video_path = "/home/devi/px4_drone_survey/detections/mission_optical_recon.mp4"
        self.color_ranges = {
            "RED_CUBE": [],
            "YELLOW_CUBE": [],
            "GREEN_CUBE": [],
            "ORANGE_CUBE": [],
            "BLUE_CUBE": [],
            "PURPLE_CUBE": [],
            "CYAN_CUBE": [],
        }
        self.targets = [
            MockTarget(1, "RED_CUBE", "SURVIVOR CASUALTY LZ MARKER", "P1 - CRITICAL",
                       "RAPID EXTRACTION / AIRDROP MEDICAL PACK", "1.0m x 1.0m x 0.6m",
                       28.05, -42.45, 0.06, 142, (0, 0, 220)),
            MockTarget(2, "YELLOW_CUBE", "BIOHAZARD CONTAINMENT DRUM", "P1 - CRITICAL",
                       "HAZMAT ISOLATION / DECONTAMINATION", "0.8m x 0.8m x 1.1m",
                       86.12, -23.95, 0.05, 118, (0, 220, 220)),
            MockTarget(3, "GREEN_CUBE", "FIELD MEDICAL SUPPLY CACHE", "P2 - HIGH",
                       "DISPATCH TRIAGE GROUND TEAM", "1.2m x 1.2m x 0.8m",
                       55.95, -6.08, 0.05, 126, (0, 200, 0)),
            MockTarget(4, "ORANGE_CUBE", "POTABLE WATER DISTRIBUTION CUBE", "P2 - HIGH",
                       "ALLOCATE PURIFICATION UNIT", "1.0m x 1.0m x 1.0m",
                       17.92, 8.04, 0.06, 95, (0, 140, 255)),
            MockTarget(5, "BLUE_CUBE", "EMERGENCY TELECOM REPEATER NODE", "P3 - LOGISTICS",
                       "DEPLOY MESH BACKHAUL GATEWAY", "0.7m x 0.7m x 1.4m",
                       44.10, 23.90, 0.05, 110, (220, 80, 0)),
            MockTarget(6, "PURPLE_CUBE", "SECONDARY GENERATOR / FUEL CELL", "P3 - LOGISTICS",
                       "SCHEDULE POWER RESTORATION", "1.1m x 0.9m x 0.8m",
                       95.88, 42.40, 0.06, 88, (180, 0, 180)),
            MockTarget(7, "CYAN_CUBE", "SEARCH & RESCUE RALLY ZONE", "P2 - HIGH",
                       "ESTABLISH FORWARD EVAC CORRIDOR", "1.5m x 1.5m x 0.5m",
                       105.95, 2.05, 0.05, 104, (220, 220, 0)),
        ]

    def get_confirmed_targets(self):
        return self.targets

    def get_all_targets(self):
        return self.targets

class MockState:
    def __init__(self, val):
        self.value = val

class MockMission:
    def __init__(self):
        self.telemetry_lock = threading.Lock()
        self.pos_x = 65.0
        self.pos_y = -18.0
        self.pos_z = -12.44  # Terrain-following: 12.0m AGL + 0.44m ground knoll
        self.vel_x = -5.5
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.roll_rad = math.radians(-1.2)
        self.pitch_rad = math.radians(-3.2)
        self.yaw_rad = math.radians(180.0)

        self.state = MockState("SURVEY_LANE")
        self.current_lane = 2
        self.total_lanes = 5
        self.survey_speed = 5.5
        self.mission_start_time = time.time() - 94.0

        self.cam_stream = MockStream()
        self.latest_annotated_frame = self.cam_stream.get_frame()
        self.detector = MockDetector()

        import collections
        self.log_queue = collections.deque(maxlen=100)
        self.log_queue.append((time.strftime("%H:%M:%S"), "HADR Reconnaissance Mission initialized."))
        self.log_queue.append((time.strftime("%H:%M:%S"), "Proving Ground: 120m x 90m (ISO Containers & Collapse Debris mapped)."))
        self.log_queue.append((time.strftime("%H:%M:%S"), "Active terrain-following engaged: locked at constant 12.0m AGL."))
        self.log_queue.append((time.strftime("%H:%M:%S"), "Surveying Lane 2 Southbound (HDOP: 0.65, RTK Fixed)."))

        self.lanes = [
            {"start": (8.0, -36.0), "end": (112.0, -36.0), "yaw_deg": 0.0},
            {"start": (112.0, -18.0), "end": (8.0, -18.0), "yaw_deg": 180.0},
            {"start": (8.0, 0.0), "end": (112.0, 0.0), "yaw_deg": 0.0},
            {"start": (112.0, 18.0), "end": (8.0, 18.0), "yaw_deg": 180.0},
            {"start": (8.0, 36.0), "end": (112.0, 36.0), "yaw_deg": 0.0},
        ]
        self.dubins_arcs = [
            {"center": (112.0, -27.0), "radius": 9.0, "start_rad": 0.0, "sweep_rad": math.pi, "type": "NORTH_TURN"},
            {"center": (8.0, -9.0), "radius": 9.0, "start_rad": 0.0, "sweep_rad": math.pi, "type": "SOUTH_TURN"},
            {"center": (112.0, 9.0), "radius": 9.0, "start_rad": 0.0, "sweep_rad": math.pi, "type": "NORTH_TURN"},
            {"center": (8.0, 27.0), "radius": 9.0, "start_rad": 0.0, "sweep_rad": math.pi, "type": "SOUTH_TURN"},
        ]

    def export_sitrep(self, filename_prefix: str = "MISSION_RECON_SITREP"):
        os.makedirs(self.detector.output_dir, exist_ok=True)
        sitrep_json = os.path.join(self.detector.output_dir, f"{filename_prefix}.json")
        with open(sitrep_json, "w") as f:
            f.write('{"status": "OK", "contacts": 7}\n')
        sitrep_md = os.path.join(self.detector.output_dir, f"{filename_prefix}.md")
        with open(sitrep_md, "w") as f:
            f.write('# HADR Reconnaissance SITREP\n')

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Test Tkinter GCS Rendering")
    parser.add_argument("--keep-open", action="store_true", help="Keep GCS window open until closed manually")
    parser.add_argument("--show-modal", action="store_true", help="Trigger target inspection modal for Target #1")
    parser.add_argument("--timeout", type=float, default=4.0, help="Auto-close timeout in seconds (default: 4.0s)")
    args = parser.parse_args()

    root = tk.Tk()
    mission = MockMission()
    gcs = TkinterGCS(root, mission_ref=mission)

    # Populate realistic flown breadcrumb trail
    # 1. Lane 1 Northbound (8.0, -36.0) -> (112.0, -36.0)
    for i in range(40):
        prog = i / 39.0
        gcs.trail.append((8.0 + prog * 104.0, -36.0))
    # 2. Dubins North Turn Arc (R=9m) around (112.0, -27.0)
    for i in range(20):
        ang = (i / 19.0) * math.pi
        gcs.trail.append((112.0 + 9.0 * math.sin(ang), -27.0 - 9.0 * math.cos(ang)))
    # 3. Lane 2 Southbound (112.0, -18.0) -> (65.0, -18.0)
    for i in range(25):
        prog = i / 24.0
        gcs.trail.append((112.0 - prog * 47.0, -18.0))

    # Pre-populate photogrammetric coverage heatmap patches along Lane 1 and Lane 2
    # Lane 1 (Northbound): y = -36m, swath width = 18m (-45m to -27m)
    for step_x in range(10, 110, 8):
        nl = (step_x - 5.0, -45.0)
        fl = (step_x + 15.0, -45.0)
        fr = (step_x + 15.0, -27.0)
        nr = (step_x - 5.0, -27.0)
        gcs.coverage_patches.append((nl[0], nl[1], fl[0], fl[1], fr[0], fr[1], nr[0], nr[1]))

    # Turn 1 Arc Coverage
    for step_a in range(0, 180, 20):
        rad = math.radians(step_a)
        cx = 112.0 + 9.0 * math.sin(rad)
        cy = -27.0 - 9.0 * math.cos(rad)
        gcs.coverage_patches.append((cx - 8, cy - 8, cx + 8, cy - 8, cx + 8, cy + 8, cx - 8, cy + 8))

    # Lane 2 (Southbound): y = -18m, swath width = 18m (-27m to -9m) from 112 down to 65
    for step_x in range(110, 60, -8):
        nl = (step_x + 5.0, -9.0)
        fl = (step_x - 15.0, -9.0)
        fr = (step_x - 15.0, -27.0)
        nr = (step_x + 5.0, -27.0)
        gcs.coverage_patches.append((nl[0], nl[1], fl[0], fl[1], fr[0], fr[1], nr[0], nr[1]))

    gcs.log_msg("Offboard trajectory controller engaged. Cruise Speed: 5.5 m/s.")
    gcs.log_msg("Locked Contact #1 [P1 - CRITICAL] SURVIVOR CASUALTY LZ MARKER (+28.05m, -42.45m).")
    gcs.log_msg("Dubins North Turn Arc (R=9.0m) completed smoothly.")
    gcs.log_msg("Locked Contact #2 [P1 - CRITICAL] BIOHAZARD CONTAINMENT DRUM (+86.12m, -23.95m).")

    if args.show_modal:
        root.after(400, lambda: gcs._show_target_vignette(1))

    if not args.keep_open:
        def auto_close():
            time.sleep(args.timeout)
            try:
                if root.winfo_exists():
                    root.after(100, root.destroy)
            except Exception:
                pass

        threading.Thread(target=auto_close, daemon=True).start()

    root.mainloop()

if __name__ == "__main__":
    main()
