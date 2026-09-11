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
        self.frame = cv2.imread('/home/devi/px4_drone_survey/detections/sample_survey_hud_frame.jpg')
        if self.frame is None:
            self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
    def get_frame(self):
        return self.frame
    def get_stats(self):
        return {"fps": 30.2}

class MockTarget:
    def __init__(self, tid, name, x, y, sigma, hits):
        self.target_id = tid
        self.color_name = name
        self.x = x
        self.y = y
        self.pos_std_dev = sigma
        self.observations = hits
        self.confirmed = True

class MockDetector:
    def __init__(self):
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
            MockTarget(1, "RED_CUBE", 28.05, -42.45, 0.06, 142),
            MockTarget(2, "YELLOW_CUBE", 86.12, -23.95, 0.05, 118),
            MockTarget(3, "GREEN_CUBE", 55.95, -6.08, 0.05, 126),
            MockTarget(4, "ORANGE_CUBE", 17.92, 8.04, 0.06, 95),
            MockTarget(5, "BLUE_CUBE", 44.10, 23.90, 0.05, 110),
            MockTarget(6, "PURPLE_CUBE", 95.88, 42.40, 0.06, 88),
            MockTarget(7, "CYAN_CUBE", 105.95, 2.05, 0.05, 104),
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
        self.pos_z = -12.0
        self.vel_x = -5.5
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.roll_rad = math.radians(-1.2)
        self.pitch_rad = math.radians(-3.2)
        self.yaw_rad = math.radians(180.0)

        self.state = MockState("SURVEY_LANE")
        self.current_lane = 2
        self.total_lanes = 5

        self.cam_stream = MockStream()
        self.latest_annotated_frame = self.cam_stream.get_frame()
        self.detector = MockDetector()

        import collections
        self.log_queue = collections.deque(maxlen=100)
        self.log_queue.append((time.strftime("%H:%M:%S"), "RTK GNSS differential fix acquired (Fix Type: RTK Fixed, HDOP: 0.65)."))
        self.log_queue.append((time.strftime("%H:%M:%S"), "Operational apron demarcated: 120m x 90m tarmac + perimeter gravel route."))
        self.log_queue.append((time.strftime("%H:%M:%S"), "Aviation obstacle beacons verified at 4 perimeter coordinates."))

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

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Test Tkinter GCS Rendering")
    parser.add_argument("--keep-open", action="store_true", help="Keep GCS window open until closed manually")
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

    gcs.log_msg("Offboard trajectory controller engaged. Survey Speed: 5.5 m/s.")
    gcs.log_msg("Locked Target #1 [CUBE RED] (+28.05m, -42.45m) [West Perimeter].")
    gcs.log_msg("Dubins North Turn Arc (R=9.0m) completed smoothly.")
    gcs.log_msg("Locked Target #2 [CUBE YELLOW] (+86.12m, -23.95m) [NW Knoll Summit].")

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
