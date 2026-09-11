#!/usr/bin/env python3
"""
Custom Tactical Ground Control Station (GCS) in Python Tkinter.
Features:
- Dynamic Responsive Scaling: Adapts cleanly to any window or monitor size.
- High-Readability Military/Aerospace Dark UI with high-contrast typography.
- Live 30 FPS Optical Video Feed with tactical boresight HUD and target bounding boxes.
- Real-time 2D Tactical Proving Ground Canvas:
  * Realistic 120m x 90m tactical proving ground: asphalt apron, gravel perimeter road, GCP crosses
  * ICAO aviation-striped survey towers with warning beacon pips
  * Natural terraced rolling knoll (+0.65m) and ridge (+0.68m) with topographic contour rings
  * Natural rocky outcrops (Alpha & Beta) and concrete Jersey barriers
  * 5 Target-Aligned survey lanes, waypoint indicators, and continuous coordinated Dubins arcs
  * Real-time drone silhouette, heading vector, and 45-deg camera nadir ground footprint
  * Dual-tone flight history trail (faded carbon-slate to active crimson)
  * Confirmed CUBE targets with shape glyphs and 1-sigma covariance uncertainty circles
  * Interactive Pan & Zoom controls (mouse drag, scroll wheel, follow-drone, reset)
- Primary Flight Telemetry Instruments: Altitude, Groundspeed, Heading, Attitude (pitch/roll stability), NED coords.
- Confirmed Cubes Registry Table with live updates and coverage statistics.
- Thread-safe Mission Event Console with live auto-scrolling ticker.
"""

import os
import sys
import math
import time
import random
import tkinter as tk
from tkinter import ttk
from typing import Dict, List, Tuple, Optional
import cv2
import numpy as np
from PIL import Image, ImageTk


class TkinterGCS:
    def __init__(self, root: tk.Tk, mission_ref=None):
        self.root = root
        self.mission = mission_ref
        self.root.title("PX4 TACTICAL GCS — AERIAL RECONNAISSANCE SYSTEM")
        self.root.geometry("1440x880")
        self.root.minsize(750, 480)
        self.root.configure(bg="#000000")

        # Strictly Pitch Black, Monotone Obsidian & Tasteful Dark Red Aerospace Aesthetic
        self.c_black = "#000000"
        self.c_panel = "#070707"
        self.c_panel_alt = "#0c0c0c"
        self.c_header = "#101010"
        self.c_card = "#141414"
        self.c_border = "#202020"
        self.c_border_subtle = "#181818"

        self.c_text_high = "#f0f0f0"      # Crisp Monotone Off-White
        self.c_text_med = "#a0a0a0"       # Neutral Silver Grey
        self.c_text_low = "#555555"       # Dim Neutral Grey

        # Strictly Dark Red Accents (tasteful, small places only)
        self.c_red = "#991b1b"            # Deep crimson
        self.c_red_bright = "#b91c1c"     # Primary dark red indicator
        self.c_red_dark = "#450a0a"       # Deep dark red border/tint
        self.c_red_subtle = "#200606"     # Very dark red backdrop

        self.photo_image: Optional[ImageTk.PhotoImage] = None
        self.start_time = time.time()
        self.trail: List[Tuple[float, float]] = []
        self.last_trail_pt = (0.0, 0.0)
        self.coverage_patches: List[Tuple[float, float, float, float, float, float, float, float]] = []
        self.last_coverage_time = 0.0
        self.tree_tick = 0

        # Interactive Map Navigation State (Pan & Zoom)
        self.zoom_factor = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.auto_follow = False
        self._drag_start_x = 0
        self._drag_start_y = 0

        self._build_ui()
        self.root.bind("<Configure>", self._on_resize)

        # Default graceful close protocol
        def _default_close():
            if self.mission and hasattr(self.mission, 'running'):
                self.mission.running = False
                if hasattr(self.mission, 'cam_stream'):
                    self.mission.cam_stream.stop()
            try:
                self.root.destroy()
            except Exception:
                pass
        self.root.protocol("WM_DELETE_WINDOW", _default_close)

        self.update_loop()

    def _build_ui(self):
        # Master Grid Layout
        self.root.rowconfigure(0, weight=0)  # Top Bar
        self.root.rowconfigure(1, weight=1)  # Main Workspace
        self.root.columnconfigure(0, weight=1)

        # ----------------------------------------------------------------------
        # 1. TOP STATUS BAR (Clean, Sleek, Minimalist)
        # ----------------------------------------------------------------------
        top_bar = tk.Frame(self.root, bg=self.c_panel, height=44,
                           highlightbackground=self.c_border, highlightthickness=1)
        top_bar.grid(row=0, column=0, sticky="ew", padx=6, pady=(6, 3))
        top_bar.columnconfigure(0, weight=1)

        brand_frame = tk.Frame(top_bar, bg=self.c_panel)
        brand_frame.pack(side=tk.LEFT, padx=10, pady=5)

        tk.Label(brand_frame, text="PX4 GCS", font=("Consolas", 11, "bold"),
                 fg=self.c_text_high, bg=self.c_panel).pack(side=tk.LEFT)
        tk.Label(brand_frame, text="|", font=("Consolas", 9),
                 fg=self.c_border, bg=self.c_panel).pack(side=tk.LEFT, padx=6)
        tk.Label(brand_frame, text="AUTONOMOUS RECONNAISSANCE", font=("Consolas", 9, "bold"),
                 fg=self.c_text_low, bg=self.c_panel).pack(side=tk.LEFT)

        chips_frame = tk.Frame(top_bar, bg=self.c_panel)
        chips_frame.pack(side=tk.RIGHT, padx=6, pady=5)

        self.chip_state = self._create_status_chip(chips_frame, "MODE", "STANDBY", self.c_red_bright)
        self.chip_lane = self._create_status_chip(chips_frame, "SWATH", "LANE 1/5", self.c_text_high)
        self.chip_targets = self._create_status_chip(chips_frame, "TARGETS", "0/7", self.c_text_high)
        self.chip_time = self._create_status_chip(chips_frame, "MISSION", "00:00", self.c_text_high)
        self.chip_fps = self._create_status_chip(chips_frame, "FPS", "30", self.c_text_med)

        # ----------------------------------------------------------------------
        # 2. MAIN WORKSPACE (Left: Video + Telemetry, Right: Map + Table + Log)
        # ----------------------------------------------------------------------
        workspace = tk.Frame(self.root, bg=self.c_black)
        workspace.grid(row=1, column=0, sticky="nsew", padx=6, pady=(0, 6))
        workspace.rowconfigure(0, weight=1)
        workspace.columnconfigure(0, weight=1, uniform="work_col")  # Left Column
        workspace.columnconfigure(1, weight=1, uniform="work_col")  # Right Column

        # ======================================================================
        # LEFT COLUMN: Optical Video & Primary Telemetry
        # ======================================================================
        left_col = tk.Frame(workspace, bg=self.c_panel,
                            highlightbackground=self.c_border, highlightthickness=1)
        left_col.grid(row=0, column=0, sticky="nsew", padx=(0, 3))
        left_col.rowconfigure(1, weight=1)  # Video expands
        left_col.rowconfigure(2, weight=0)  # Telemetry fixed
        left_col.columnconfigure(0, weight=1)

        # Video Section Header
        v_head = tk.Frame(left_col, bg=self.c_header, height=28)
        v_head.grid(row=0, column=0, sticky="ew")

        live_tag = tk.Frame(v_head, bg=self.c_header)
        live_tag.pack(side=tk.RIGHT, padx=8, pady=4)
        tk.Label(live_tag, text="● OPTICAL LIVE", font=("Consolas", 8, "bold"),
                 fg=self.c_red_bright, bg=self.c_header).pack(side=tk.LEFT)

        tk.Label(v_head, text="PRIMARY RECONNAISSANCE FEED (45° NADIR)", font=("Consolas", 9, "bold"),
                 fg=self.c_text_med, bg=self.c_header).pack(side=tk.LEFT, padx=8, pady=4)

        # Video Viewport (Pitch Black backing)
        self.video_container = tk.Frame(left_col, bg=self.c_black)
        self.video_container.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        self.video_label = tk.Label(self.video_container, bg=self.c_black,
                                    text="INITIALIZING OPTICAL STREAM...",
                                    font=("Consolas", 9), fg=self.c_text_low)
        self.video_label.pack(fill=tk.BOTH, expand=True)

        # Primary Telemetry Cluster (Glass Cockpit Style)
        telem_ribbon = tk.Frame(left_col, bg=self.c_panel_alt,
                                highlightbackground=self.c_border_subtle, highlightthickness=1)
        telem_ribbon.grid(row=2, column=0, sticky="ew", padx=4, pady=(0, 4))
        telem_ribbon.columnconfigure(0, weight=1, uniform="col")
        telem_ribbon.columnconfigure(1, weight=1, uniform="col")
        telem_ribbon.rowconfigure((0, 1, 2), weight=1)

        self.lbl_alt, self.sub_alt = self._create_telem_cell(telem_ribbon, "ALTITUDE (AGL)", "0.0", "m", 0, 0)
        self.lbl_spd, self.sub_spd = self._create_telem_cell(telem_ribbon, "GROUND SPEED", "0.0", "m/s", 0, 1)
        self.lbl_hdg, self.sub_hdg = self._create_telem_cell(telem_ribbon, "HEADING (YAW)", "000°", "[N]", 1, 0)
        self.lbl_att, self.sub_att = self._create_telem_cell(telem_ribbon, "ATTITUDE STABILITY", "P +0.0° R +0.0°", "STABLE", 1, 1)
        self.lbl_pos, self.sub_pos = self._create_telem_cell(telem_ribbon, "POSITION (NED)", "+0.0N, +0.0E", "LOCAL REF", 2, 0)
        self.lbl_leg, self.sub_leg = self._create_telem_cell(telem_ribbon, "SWATH PROGRESS", "LANE 1 OF 5", "[□□□□□]", 2, 1)

        # ======================================================================
        # RIGHT COLUMN: Tactical Proving Ground Map, Target Table, Event Log
        # ======================================================================
        right_col = tk.Frame(workspace, bg=self.c_panel,
                             highlightbackground=self.c_border, highlightthickness=1)
        right_col.grid(row=0, column=1, sticky="nsew", padx=(3, 0))
        right_col.rowconfigure(1, weight=1)  # Map canvas takes all available space
        right_col.rowconfigure(3, weight=0)  # Table fits exactly 5 target rows
        right_col.rowconfigure(5, weight=0)  # Console fixed height
        right_col.columnconfigure(0, weight=1)

        # Tactical Map Header with Interactive Controls
        m_head = tk.Frame(right_col, bg=self.c_header, height=28)
        m_head.grid(row=0, column=0, sticky="ew")

        # Map Navigation Buttons (Zoom +, Zoom -, Follow, Reset)
        nav_btn_frame = tk.Frame(m_head, bg=self.c_header)
        nav_btn_frame.pack(side=tk.RIGHT, padx=6, pady=3)

        btn_sitrep = tk.Button(nav_btn_frame, text="SITREP", font=("Consolas", 7, "bold"),
                               bg="#181818", fg=self.c_red_bright, activebackground="#242424",
                               activeforeground="#ffffff", relief=tk.FLAT, bd=0, padx=4, pady=1,
                               command=self._export_sitrep_action)
        btn_sitrep.pack(side=tk.RIGHT, padx=2)

        self.btn_follow = tk.Button(nav_btn_frame, text="FOLLOW", font=("Consolas", 7, "bold"),
                                    bg="#141414", fg=self.c_text_low, activebackground="#242424",
                                    activeforeground="#ffffff", relief=tk.FLAT, bd=0, padx=4, pady=1,
                                    command=self._toggle_follow)
        self.btn_follow.pack(side=tk.RIGHT, padx=2)

        btn_reset = tk.Button(nav_btn_frame, text="RESET", font=("Consolas", 7),
                              bg="#141414", fg=self.c_text_med, activebackground="#242424",
                              activeforeground="#ffffff", relief=tk.FLAT, bd=0, padx=4, pady=1,
                              command=self._reset_map_view)
        btn_reset.pack(side=tk.RIGHT, padx=2)

        btn_zin = tk.Button(nav_btn_frame, text="+", font=("Consolas", 8, "bold"),
                            bg="#141414", fg=self.c_text_high, activebackground="#242424",
                            activeforeground="#ffffff", relief=tk.FLAT, bd=0, padx=4, pady=0,
                            command=lambda: self._zoom(1.25))
        btn_zin.pack(side=tk.RIGHT, padx=1)

        btn_zout = tk.Button(nav_btn_frame, text="-", font=("Consolas", 8, "bold"),
                             bg="#141414", fg=self.c_text_high, activebackground="#242424",
                             activeforeground="#ffffff", relief=tk.FLAT, bd=0, padx=4, pady=0,
                             command=lambda: self._zoom(0.8))
        btn_zout.pack(side=tk.RIGHT, padx=1)

        tk.Label(m_head, text="TACTICAL PROVING GROUND (120x90m)", font=("Consolas", 9, "bold"),
                 fg=self.c_text_med, bg=self.c_header).pack(side=tk.LEFT, padx=8, pady=4)

        # Tactical Map Canvas
        self.map_canvas = tk.Canvas(right_col, bg="#000000", highlightthickness=0, height=140)
        self.map_canvas.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)

        # Bind Interactive Mouse Gestures to Canvas
        self.map_canvas.bind("<ButtonPress-1>", self._on_map_drag_start)
        self.map_canvas.bind("<B1-Motion>", self._on_map_drag_move)
        self.map_canvas.bind("<MouseWheel>", self._on_map_mousewheel)
        self.map_canvas.bind("<Button-4>", lambda e: self._zoom(1.15))
        self.map_canvas.bind("<Button-5>", lambda e: self._zoom(0.87))
        self.map_canvas.bind("<Double-Button-1>", lambda e: self._reset_map_view())

        # Target Registry Header
        t_head = tk.Frame(right_col, bg=self.c_header, height=26)
        t_head.grid(row=2, column=0, sticky="ew", pady=(2, 0))
        self.lbl_target_summary = tk.Label(t_head, text="0/7 TARGETS ACQUIRED", font=("Consolas", 8, "bold"),
                                           fg=self.c_text_low, bg=self.c_header)
        self.lbl_target_summary.pack(side=tk.RIGHT, padx=8, pady=3)
        tk.Label(t_head, text="TARGET RECONNAISSANCE REGISTRY", font=("Consolas", 9, "bold"),
                 fg=self.c_text_med, bg=self.c_header).pack(side=tk.LEFT, padx=8, pady=3)

        # Target Table (Custom Dark Treeview)
        table_frame = tk.Frame(right_col, bg=self.c_panel)
        table_frame.grid(row=3, column=0, sticky="nsew", padx=4, pady=2)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Treeview",
                        background="#070707",
                        foreground="#f0f0f0",
                        fieldbackground="#070707",
                        font=("Consolas", 9),
                        rowheight=20,
                        borderwidth=0)
        style.configure("Treeview.Heading",
                        background="#101010",
                        foreground="#a0a0a0",
                        font=("Consolas", 8, "bold"),
                        borderwidth=0)
        style.map("Treeview",
                  background=[("selected", "#2b0a0a")],
                  foreground=[("selected", "#ffffff")])

        columns = ("id", "pri", "class", "pos", "sigma", "hits", "status")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=7)
        self.tree.heading("id", text="ID")
        self.tree.heading("pri", text="PRI")
        self.tree.heading("class", text="TACTICAL RECON ENTITY")
        self.tree.heading("pos", text="COORDINATES (NED)")
        self.tree.heading("sigma", text="1-SIGMA")
        self.tree.heading("hits", text="HITS")
        self.tree.heading("status", text="STATUS")

        self.tree.column("id", width=24, minwidth=20, anchor=tk.CENTER)
        self.tree.column("pri", width=36, minwidth=28, anchor=tk.CENTER)
        self.tree.column("class", width=120, minwidth=85, anchor=tk.W)
        self.tree.column("pos", width=110, minwidth=80, anchor=tk.CENTER)
        self.tree.column("sigma", width=65, minwidth=45, anchor=tk.CENTER)
        self.tree.column("hits", width=34, minwidth=26, anchor=tk.CENTER)
        self.tree.column("status", width=65, minwidth=45, anchor=tk.CENTER)
        self.tree.pack(fill=tk.BOTH, expand=True)

        self.tree.bind("<Double-Button-1>", self._on_tree_double_click)

        def _on_table_resize(event):
            tw = event.width - 16
            if tw > 180:
                self.tree.column("id", width=max(20, int(tw * 0.06)))
                self.tree.column("pri", width=max(28, int(tw * 0.08)))
                self.tree.column("class", width=max(85, int(tw * 0.30)))
                self.tree.column("pos", width=max(80, int(tw * 0.24)))
                self.tree.column("sigma", width=max(45, int(tw * 0.13)))
                self.tree.column("hits", width=max(26, int(tw * 0.06)))
                self.tree.column("status", width=max(45, int(tw * 0.13)))
        table_frame.bind("<Configure>", _on_table_resize)

        # System Log Header
        l_head = tk.Frame(right_col, bg=self.c_header, height=22)
        l_head.grid(row=4, column=0, sticky="ew", pady=(2, 0))
        tk.Label(l_head, text="MISSION EVENT CONSOLE", font=("Consolas", 8, "bold"),
                 fg=self.c_text_low, bg=self.c_header).pack(side=tk.LEFT, padx=8, pady=2)

        # System Console Text Box
        self.log_text = tk.Text(right_col, bg="#040404", fg="#a0a0a0", font=("Consolas", 8),
                                height=3, wrap=tk.WORD, relief=tk.FLAT,
                                highlightbackground=self.c_border_subtle, highlightthickness=1)
        self.log_text.grid(row=5, column=0, sticky="ew", padx=4, pady=(1, 4))
        self.log_msg("PX4 Tactical Reconnaissance GCS online. Ready for mission telemetry.")

    def _export_sitrep_action(self):
        """Action handler to export mission SITREP markdown and JSON."""
        if self.mission is not None and hasattr(self.mission, 'export_sitrep'):
            try:
                self.mission.export_sitrep()
                self.log_msg("SITREP EXPORT SUCCESS: MISSION_RECON_SITREP.json and MISSION_RECON_SITREP.md generated.")
            except Exception as e:
                self.log_msg(f"SITREP EXPORT FAILED: {e}")
        else:
            self.log_msg("SITREP EXPORT: No active mission telemetry controller.")

    def _on_tree_double_click(self, event):
        """Handles double click on a target row to open target inspection modal."""
        item = self.tree.identify_row(event.y)
        if not item:
            item = self.tree.focus()
        if item and item.startswith("tgt_"):
            try:
                tid = int(item.split("_")[1])
                self._show_target_vignette(tid)
            except Exception as e:
                self.log_msg(f"Target inspection error: {e}")

    def _show_target_vignette(self, target_id: int):
        """Displays interactive high-contrast tactical modal for target vignette and photogrammetric triage."""
        target = None
        if self.mission is not None and hasattr(self.mission, 'detector'):
            for t in self.mission.detector.get_all_targets():
                if t.target_id == target_id:
                    target = t
                    break

        modal = tk.Toplevel(self.root)
        modal.title(f"TACTICAL RECON VIGNETTE — TARGET #{target_id:02d}")
        # Center modal relative to root window
        rx = self.root.winfo_rootx()
        ry = self.root.winfo_rooty()
        rw = self.root.winfo_width()
        rh = self.root.winfo_height()
        pos_x = max(60, rx + (rw - 580) // 2)
        pos_y = max(60, ry + (rh - 620) // 2)
        modal.geometry(f"580x620+{pos_x}+{pos_y}")
        modal.minsize(500, 550)
        modal.configure(bg="#080808")
        modal.transient(self.root)

        # Header banner
        header = tk.Frame(modal, bg=self.c_header, height=36)
        header.pack(fill=tk.X, side=tk.TOP)
        tk.Label(header, text=f"TARGET #{target_id:02d} OPTICAL VIGNETTE & ANALYSIS",
                 font=("Consolas", 10, "bold"), fg=self.c_red_bright, bg=self.c_header).pack(side=tk.LEFT, padx=12, pady=8)

        btn_close = tk.Button(header, text="✕ CLOSE", font=("Consolas", 8, "bold"),
                              bg="#1a1a1a", fg="#a0a0a0", activebackground="#2a2a2a",
                              activeforeground="#ffffff", relief=tk.FLAT, bd=0, padx=8, pady=2,
                              command=modal.destroy)
        btn_close.pack(side=tk.RIGHT, padx=10, pady=6)

        content = tk.Frame(modal, bg="#080808")
        content.pack(fill=tk.BOTH, expand=True, padx=16, pady=12)

        # Image Container (240x240 optical vignette)
        img_frame = tk.Frame(content, bg="#000000", highlightbackground="#222222", highlightthickness=1, width=260, height=260)
        img_frame.pack(side=tk.TOP, pady=(0, 12))
        img_frame.pack_propagate(False)

        img_label = tk.Label(img_frame, bg="#000000")
        img_label.pack(fill=tk.BOTH, expand=True)

        crop = getattr(target, 'best_crop', None) if target else None
        if crop is None and target and hasattr(target, 'crop_path') and os.path.exists(target.crop_path):
            try:
                crop = cv2.imread(target.crop_path)
            except Exception:
                crop = None

        if crop is not None:
            try:
                rgb_crop = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
                rgb_crop = cv2.resize(rgb_crop, (240, 240), interpolation=cv2.INTER_AREA)
                pil_img = Image.fromarray(rgb_crop)
                photo_ref = ImageTk.PhotoImage(image=pil_img)
                img_label.config(image=photo_ref)
                img_label.image = photo_ref  # keep reference to prevent GC
            except Exception:
                img_label.config(text="IMAGE RENDERING ERROR", fg="#555555", font=("Consolas", 9))
        else:
            # Placeholder reticle
            ph_canvas = tk.Canvas(img_frame, width=240, height=240, bg="#020202", highlightthickness=0)
            ph_canvas.pack(fill=tk.BOTH, expand=True)
            ph_canvas.create_line(120, 20, 120, 220, fill="#1c1c1c", width=1)
            ph_canvas.create_line(20, 120, 220, 120, fill="#1c1c1c", width=1)
            ph_canvas.create_oval(70, 70, 170, 170, outline="#222222", width=1)
            ph_canvas.create_text(120, 110, text="[OPTICAL VIGNETTE]", fill="#444444", font=("Consolas", 8, "bold"))
            ph_canvas.create_text(120, 130, text="NO CROPPED FRAME CAPTURED YET", fill="#333333", font=("Consolas", 7))

        # Target metadata details grid
        data_frame = tk.Frame(content, bg="#0d0d0d", highlightbackground="#1c1c1c", highlightthickness=1)
        data_frame.pack(fill=tk.BOTH, expand=True)

        if target is not None:
            t_name = getattr(target, 'triage_name', target.display_name if hasattr(target, 'display_name') else target.color_name)
            t_pri = getattr(target, 'priority', 'P2 - HIGH')
            t_act = getattr(target, 'action', 'Aerial Reconnaissance')
            t_dims = getattr(target, 'nominal_dims', '1.0m x 1.0m x 0.6m')
            lat, lon = target.wgs84_coords if hasattr(target, 'wgs84_coords') else (47.397971, 8.546164)
            pos_str = f"({target.x:+.2f}m N, {target.y:+.2f}m E)"
            gps_str = f"{lat:.6f}°N, {lon:.6f}°E"
            sig_str = f"±{target.pos_std_dev:.2f} m"
            obs_str = f"{target.observations} detections (Kalman EKF filtered)"
            stat_str = "● LOCKED & CONFIRMED" if target.confirmed else "◌ TRACKING (UNCONFIRMED)"
        else:
            t_name = f"TARGET #{target_id:02d}"
            t_pri = "UNKNOWN"
            t_act = "N/A"
            t_dims = "N/A"
            pos_str = "N/A"
            gps_str = "N/A"
            sig_str = "N/A"
            obs_str = "0"
            stat_str = "NOT LOCATED"

        pri_color = "#ff3333" if "P1" in t_pri else (self.c_red_bright if "P2" in t_pri else "#888888")

        rows = [
            ("TACTICAL ENTITY", t_name, self.c_text_high),
            ("PRIORITY LEVEL", t_pri, pri_color),
            ("RECOMMENDED ACTION", t_act, "#ffffff"),
            ("PHYSICAL DIMS", t_dims, "#aaaaaa"),
            ("LOCAL POSITION (NED)", pos_str, "#ffffff"),
            ("GPS WGS84 COORDS", gps_str, "#ffffff"),
            ("POSITION UNCERTAINTY", sig_str, self.c_text_med),
            ("SAMPLE OBSERVATIONS", obs_str, "#aaaaaa"),
            ("EKF FILTER STATUS", stat_str, pri_color),
        ]

        for label, val, col in rows:
            r_frame = tk.Frame(data_frame, bg="#0d0d0d")
            r_frame.pack(fill=tk.X, padx=12, pady=3)
            tk.Label(r_frame, text=label, font=("Consolas", 7, "bold"), fg=self.c_text_low, bg="#0d0d0d", width=22, anchor="w").pack(side=tk.LEFT)
            tk.Label(r_frame, text=val, font=("Consolas", 8, "bold"), fg=col, bg="#0d0d0d", anchor="w").pack(side=tk.LEFT, fill=tk.X, expand=True)

        modal.bind("<Escape>", lambda e: modal.destroy())

    def _create_status_chip(self, parent, label_text, val_default, val_color):
        chip = tk.Frame(parent, bg=self.c_card, highlightbackground=self.c_border_subtle, highlightthickness=1)
        chip.pack(side=tk.LEFT, padx=2)

        tk.Label(chip, text=label_text, font=("Consolas", 8, "bold"),
                 fg=self.c_text_low, bg=self.c_card).pack(side=tk.LEFT, padx=(5, 3), pady=2)
        val_lbl = tk.Label(chip, text=val_default, font=("Consolas", 9, "bold"),
                            fg=val_color, bg=self.c_card)
        val_lbl.pack(side=tk.LEFT, padx=(0, 5), pady=2)
        return val_lbl

    def _create_telem_cell(self, parent, label_text, val_default, sub_default, r, c):
        cell = tk.Frame(parent, bg=self.c_card, highlightbackground=self.c_border_subtle, highlightthickness=1)
        cell.grid(row=r, column=c, sticky="nsew", padx=2, pady=2)

        top_f = tk.Frame(cell, bg=self.c_card)
        top_f.pack(fill=tk.X, padx=5, pady=(3, 0))
        tk.Label(top_f, text=label_text, font=("Consolas", 7, "bold"),
                 fg=self.c_text_low, bg=self.c_card, anchor="w").pack(side=tk.LEFT)
        sub_lbl = tk.Label(top_f, text=sub_default, font=("Consolas", 7),
                           fg=self.c_text_low, bg=self.c_card, anchor="e")
        sub_lbl.pack(side=tk.RIGHT)

        val_lbl = tk.Label(cell, text=val_default, font=("Consolas", 10, "bold"),
                           fg=self.c_text_high, bg=self.c_card, anchor="w")
        val_lbl.pack(fill=tk.X, padx=5, pady=(0, 3))
        return val_lbl, sub_lbl

    def _zoom(self, factor: float):
        self.zoom_factor = max(0.4, min(6.0, self.zoom_factor * factor))
        self._redraw_map_only()

    def _toggle_follow(self):
        self.auto_follow = not self.auto_follow
        if self.auto_follow:
            self.btn_follow.config(fg=self.c_red_bright, text="● FOLLOWING")
        else:
            self.btn_follow.config(fg=self.c_text_low, text="FOLLOW")
        self._redraw_map_only()

    def _reset_map_view(self):
        self.zoom_factor = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.auto_follow = False
        self.btn_follow.config(fg=self.c_text_low, text="FOLLOW")
        self._redraw_map_only()

    def _on_map_drag_start(self, event):
        self._drag_start_x = event.x
        self._drag_start_y = event.y

    def _on_map_drag_move(self, event):
        dx = event.x - self._drag_start_x
        dy = event.y - self._drag_start_y
        self.pan_x += dx
        self.pan_y += dy
        self._drag_start_x = event.x
        self._drag_start_y = event.y
        if abs(dx) > 2 or abs(dy) > 2:
            if self.auto_follow:
                self.auto_follow = False
                self.btn_follow.config(fg=self.c_text_low, text="FOLLOW")
        self._redraw_map_only()

    def _on_map_mousewheel(self, event):
        if event.delta > 0:
            self._zoom(1.15)
        else:
            self._zoom(0.87)

    def log_msg(self, msg: str):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {msg}\n")
        self.log_text.see(tk.END)

    def _on_resize(self, event):
        if event.widget == self.root:
            w = self.root.winfo_width()
            val_size = 9 if w < 1150 else (11 if w < 1450 else 13)
            val_font = ("Consolas", val_size, "bold")
            if hasattr(self, 'lbl_alt'):
                for lbl in [self.lbl_alt, self.lbl_spd, self.lbl_hdg, self.lbl_att, self.lbl_pos, self.lbl_leg]:
                    lbl.config(font=val_font)
            self._redraw_map_only()

    def world_to_canvas(self, x_north: float, y_east: float, cw: float, ch: float):
        """Dynamically scales NED world coordinates to current Canvas width/height for 120m x 90m arena with pan/zoom."""
        arena_w = 104.0  # East-West span (-52 to +52)
        arena_h = 138.0  # North-South span (-9 to +129)
        margin = 20.0
        base_scale = min((cw - margin * 2) / arena_w, (ch - margin * 2) / arena_h)
        base_scale = max(1.5, base_scale)
        scale = base_scale * self.zoom_factor

        cx = cw / 2.0 + self.pan_x
        cy = ch / 2.0 + 4.0 + self.pan_y

        canvas_x = cx + y_east * scale
        canvas_y = cy - (x_north - 60.0) * scale
        return canvas_x, canvas_y, scale

    def update_loop(self):
        """Periodic 30 FPS Tkinter GUI refresh loop."""
        try:
            if not self.root.winfo_exists():
                return
        except Exception:
            return

        if self.mission is not None:
            # Drain mission log queue into console
            if hasattr(self.mission, 'log_queue'):
                while self.mission.log_queue:
                    ts, msg = self.mission.log_queue.popleft()
                    self.log_text.insert(tk.END, f"[{ts}] {msg}\n")
                    self.log_text.see(tk.END)

            # 1. Update Telemetry
            with self.mission.telemetry_lock:
                px = self.mission.pos_x
                py = self.mission.pos_y
                pz = self.mission.pos_z
                vx = self.mission.vel_x
                vy = self.mission.vel_y
                roll_deg = math.degrees(self.mission.roll_rad)
                pitch_deg = math.degrees(self.mission.pitch_rad)
                yaw_deg = math.degrees(self.mission.yaw_rad) % 360.0
                spd = math.hypot(vx, vy)

            state_str = self.mission.state.value
            lane_num = self.mission.current_lane
            total_lanes = self.mission.total_lanes

            # Record breadcrumbs
            d = math.hypot(px - self.last_trail_pt[0], py - self.last_trail_pt[1])
            if d >= 0.4:
                self.trail.append((px, py))
                self.last_trail_pt = (px, py)
                if len(self.trail) > 2000:
                    self.trail = self.trail[-2000:]

            # Cardinal direction text
            cardinal = "N" if (yaw_deg < 22.5 or yaw_deg >= 337.5) else \
                       "NE" if yaw_deg < 67.5 else \
                       "E" if yaw_deg < 112.5 else \
                       "SE" if yaw_deg < 157.5 else \
                       "S" if yaw_deg < 202.5 else \
                       "SW" if yaw_deg < 247.5 else \
                       "W" if yaw_deg < 292.5 else "NW"

            # Attitude stability assessment
            att_stable = (abs(roll_deg) <= 6.5)
            att_status_str = "STABLE (LEVEL)" if att_stable else "BANKING (TURN)"
            att_status_col = "#a0a0a0" if att_stable else self.c_red_bright

            # Progress pips
            pips = "".join(["■" if i < lane_num else "□" for i in range(total_lanes)])

            # Update Telemetry Readouts
            self.lbl_alt.config(text=f"{-pz:.1f}")
            self.lbl_spd.config(text=f"{spd:.1f}")
            self.lbl_hdg.config(text=f"{yaw_deg:03.0f}°")
            self.sub_hdg.config(text=f"[{cardinal}]")
            self.lbl_pos.config(text=f"{px:+4.1f}N, {py:+4.1f}E")
            self.lbl_att.config(text=f"P {pitch_deg:+.1f}° R {roll_deg:+.1f}°")
            self.sub_att.config(text=att_status_str, fg=att_status_col)
            self.lbl_leg.config(text=f"LANE {lane_num} OF {total_lanes}")
            self.sub_leg.config(text=f"[{pips}]")

            # Status chips
            STATE_LABELS = {
                "INITIALIZATION": "INIT",
                "TAKEOFF": "TAKEOFF",
                "TRANSIT_TO_LANE_1": "TRANSIT",
                "SURVEY_LANE": "SURVEY",
                "DUBINS_ARC_TURN": "TURN",
                "SURVEY_COMPLETE": "COMPLETE",
                "RETURN_AND_LAND": "RTL LAND",
            }
            state_disp = STATE_LABELS.get(state_str, state_str.replace("_LANE", "").replace("_TURN", "")[:8])
            state_col = self.c_red_bright if "SURVEY" in state_str else (self.c_red if "DUBINS" in state_str or state_disp == "TURN" else self.c_text_med)
            self.chip_state.config(text=f"● {state_disp}", fg=state_col)
            self.chip_lane.config(text=f"LANE {lane_num}/{total_lanes}")

            elapsed = int(time.time() - self.start_time)
            self.chip_time.config(text=f"{elapsed//60:02d}:{elapsed%60:02d}")

            stats = self.mission.cam_stream.get_stats()
            self.chip_fps.config(text=f"{stats['fps']:.0f}")

            # 2. Update Video Display (Dynamically scales preserving 4:3)
            frame = self.mission.latest_annotated_frame if (self.mission and self.mission.latest_annotated_frame is not None) else self.mission.cam_stream.get_frame()
            if frame is not None:
                vw = max(240, self.video_container.winfo_width())
                vh = max(180, self.video_container.winfo_height())
                target_w = vw
                target_h = int(target_w * 0.75)
                if target_h > vh:
                    target_h = vh
                    target_w = int(target_h / 0.75)

                target_w = max(10, target_w)
                target_h = max(10, target_h)

                disp_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                if disp_frame.shape[1] != target_w or disp_frame.shape[0] != target_h:
                    disp_frame = cv2.resize(disp_frame, (target_w, target_h), interpolation=cv2.INTER_AREA)

                img = Image.fromarray(disp_frame)
                self.photo_image = ImageTk.PhotoImage(image=img)
                self.video_label.config(image=self.photo_image, text="")

            # 3. Auto-follow drone on tactical map if active
            if self.auto_follow:
                # Center view on current drone position
                cw = self.map_canvas.winfo_width()
                ch = self.map_canvas.winfo_height()
                arena_w = 104.0
                arena_h = 138.0
                base_scale = min((cw - 40.0) / arena_w, (ch - 40.0) / arena_h) * self.zoom_factor
                self.pan_x = -py * base_scale
                self.pan_y = (px - 60.0) * base_scale

            # 4. Redraw Tactical 2D Arena Canvas
            self._draw_canvas_map(px, py, pz, yaw_deg)

            # 5. Update Target Table
            confirmed_cubes = self.mission.detector.get_confirmed_targets()
            n_conf = len(confirmed_cubes)
            total_tgts = len(self.mission.detector.color_ranges) if hasattr(self.mission.detector, 'color_ranges') else 7
            self.chip_targets.config(text=f"{n_conf}/{total_tgts}")
            if n_conf >= total_tgts:
                self.lbl_target_summary.config(text=f"{total_tgts}/{total_tgts} LOCKED (100% RECONNAISSANCE)", fg=self.c_red_bright)
            elif n_conf > 0:
                self.lbl_target_summary.config(text=f"{n_conf}/{total_tgts} ACQUIRED", fg=self.c_text_high)
            else:
                self.lbl_target_summary.config(text=f"0/{total_tgts} TARGETS ACQUIRED", fg=self.c_text_low)

            self.tree_tick += 1
            if self.tree_tick % 10 == 0:
                existing_iids = set(self.tree.get_children())
                active_iids = set()
                for t in self.mission.detector.get_all_targets():
                    status_text = "● LOCKED" if t.confirmed else "◌ TRACKING"
                    raw_col = t.color_name.upper()
                    for color_key in ["RED", "YELLOW", "GREEN", "ORANGE", "BLUE", "PURPLE", "CYAN"]:
                        if color_key in raw_col:
                            color_token = color_key
                            break
                    else:
                        color_token = raw_col.split("_")[0]
                    shape_name = f"■ CUBE [{color_token}]"
                    pri_raw = getattr(t, 'priority', 'P2 - HIGH')
                    pri_badge = pri_raw.split(" ")[0] if " " in pri_raw else pri_raw
                    triage_title = getattr(t, 'triage_name', shape_name)
                    vals = (
                        f"{t.target_id:02d}",
                        f"[{pri_badge}]",
                        triage_title,
                        f"({t.x:+.1f}, {t.y:+.1f})",
                        f"±{t.pos_std_dev:.2f} m",
                        f"{t.observations}",
                        status_text
                    )
                    iid = f"tgt_{t.target_id}"
                    active_iids.add(iid)
                    if self.tree.exists(iid):
                        self.tree.item(iid, values=vals)
                    else:
                        self.tree.insert("", tk.END, iid=iid, values=vals)
                for old_iid in existing_iids - active_iids:
                    self.tree.delete(old_iid)

        try:
            if self.root.winfo_exists():
                self.root.after(33, self.update_loop)
        except Exception:
            pass

    def _redraw_map_only(self):
        if self.mission is not None:
            with self.mission.telemetry_lock:
                px = self.mission.pos_x
                py = self.mission.pos_y
                pz = self.mission.pos_z
                yaw_deg = math.degrees(self.mission.yaw_rad) % 360.0
            self._draw_canvas_map(px, py, pz, yaw_deg)

    def _draw_canvas_map(self, px: float, py: float, pz: float, yaw_deg: float):
        """Draws photorealistic overhead tactical map with organic terrain, shadows, and debris."""
        c = self.map_canvas
        cw = c.winfo_width()
        ch = c.winfo_height()
        if cw < 60 or ch < 60:
            return

        c.delete("all")

        # ── 1. Layered Ground Surface ────────────────────────────────────────
        # Outer perimeter: compacted dirt/gravel shoulder
        r_bl_x, r_bl_y, _ = self.world_to_canvas(-6.0, -51.0, cw, ch)
        r_tr_x, r_tr_y, _ = self.world_to_canvas(126.0, 51.0, cw, ch)
        c.create_rectangle(r_bl_x, r_tr_y, r_tr_x, r_bl_y, fill="#0a0908", outline="#151412", width=1)

        # Inner survey apron: cracked asphalt
        a_bl_x, a_bl_y, _ = self.world_to_canvas(0.0, -45.0, cw, ch)
        a_tr_x, a_tr_y, _ = self.world_to_canvas(120.0, 45.0, cw, ch)
        c.create_rectangle(a_bl_x, a_tr_y, a_tr_x, a_bl_y, fill="#050504", outline="#161514", width=1)

        # Dirt patches on apron surface (irregular worn areas)
        dirt_patches = [
            (15.0, -30.0, 8.0, 5.0), (45.0, -10.0, 6.0, 4.5), (80.0, 15.0, 7.0, 5.5),
            (100.0, -35.0, 5.5, 4.0), (30.0, 32.0, 9.0, 3.5), (65.0, -40.0, 6.0, 4.0),
            (110.0, 25.0, 5.0, 6.0), (50.0, 38.0, 7.0, 4.0), (10.0, 10.0, 4.5, 3.5),
        ]
        for dx, dy, rw, rh in dirt_patches:
            dp_x, dp_y, sc = self.world_to_canvas(dx, dy, cw, ch)
            c.create_oval(dp_x - rw * sc * 0.5, dp_y - rh * sc * 0.5,
                          dp_x + rw * sc * 0.5, dp_y + rh * sc * 0.5,
                          fill="#080706", outline="")

        # Pavement cracks (long hairline fractures across the tarmac)
        cracks = [
            [(5.0, -20.0), (18.0, -22.0), (35.0, -19.0), (48.0, -21.0)],
            [(60.0, 10.0), (72.0, 8.0), (85.0, 12.0), (98.0, 9.0), (110.0, 11.0)],
            [(20.0, 30.0), (38.0, 28.0), (55.0, 32.0), (70.0, 29.0)],
        ]
        for crack in cracks:
            pts = []
            for cx_m, cy_m in crack:
                cx_p, cy_p, _ = self.world_to_canvas(cx_m, cy_m, cw, ch)
                pts.extend([cx_p, cy_p])
            if len(pts) >= 4:
                c.create_line(*pts, fill="#0e0d0b", width=1, smooth=True)

        # ── 2. Photogrammetric Coverage Heatmap ──────────────────────────────
        for p in self.coverage_patches:
            cp1_x, cp1_y, _ = self.world_to_canvas(p[0], p[1], cw, ch)
            cp2_x, cp2_y, _ = self.world_to_canvas(p[2], p[3], cw, ch)
            cp3_x, cp3_y, _ = self.world_to_canvas(p[4], p[5], cw, ch)
            cp4_x, cp4_y, _ = self.world_to_canvas(p[6], p[7], cw, ch)
            c.create_polygon(cp1_x, cp1_y, cp2_x, cp2_y, cp3_x, cp3_y, cp4_x, cp4_y,
                             fill="#0f0407", outline="")

        # ── 3. Subtle Coordinate Grid (faint, like GIS overlay) ──────────────
        grid_col = "#0b0b0a"
        tick_col = "#2a2a28"
        for x in range(0, 121, 20):
            p1_x, p1_y, _ = self.world_to_canvas(x, -45, cw, ch)
            p2_x, p2_y, _ = self.world_to_canvas(x, 45, cw, ch)
            c.create_line(p1_x, p1_y, p2_x, p2_y, fill=grid_col, width=1, dash=(1, 6))
            c.create_text(p1_x - 14, p1_y, text=f"{x}m", fill=tick_col, font=("Consolas", 6))
        for y in range(-40, 41, 20):
            p1_x, p1_y, _ = self.world_to_canvas(0, y, cw, ch)
            p2_x, p2_y, _ = self.world_to_canvas(120, y, cw, ch)
            c.create_line(p1_x, p1_y, p2_x, p2_y, fill=grid_col, width=1, dash=(1, 6))
            c.create_text(p1_x, p1_y + 9, text=f"{y:+d}m", fill=tick_col, font=("Consolas", 6))

        # ── 4. Photogrammetric Ground Control Points (survey markers) ────────
        gcps = [(-25.0, 30.0, "GCP-1"), (25.0, 30.0, "GCP-2"), (-25.0, 90.0, "GCP-3"), (25.0, 90.0, "GCP-4")]
        for g_east, g_north, g_lbl in gcps:
            gx, gy, sc = self.world_to_canvas(g_north, g_east, cw, ch)
            # Checkerboard pattern (4 quadrants)
            sz = 3
            c.create_rectangle(gx - sz, gy - sz, gx, gy, fill="#383838", outline="")
            c.create_rectangle(gx, gy, gx + sz, gy + sz, fill="#383838", outline="")
            c.create_rectangle(gx, gy - sz, gx + sz, gy, fill="#1a1a1a", outline="")
            c.create_rectangle(gx - sz, gy, gx, gy + sz, fill="#1a1a1a", outline="")
            c.create_text(gx + 8, gy, text=g_lbl, fill="#222220", font=("Consolas", 5), anchor="w")

        # ── 5. Organic Terrain Features ──────────────────────────────────────
        _, _, scale = self.world_to_canvas(60, 0, cw, ch)

        # 5a. Northwest Knoll — irregular multi-ring contours
        knoll_cx, knoll_cy, _ = self.world_to_canvas(85.0, -25.0, cw, ch)
        contour_rings_knoll = [
            (12.0, 9.0, "#070706", "#141312", (4, 4), "+0.2m"),
            (8.5, 6.5, "#0b0a09", "#1a1917", (3, 3), "+0.4m"),
            (5.2, 3.8, "#0f0e0c", "#222120", (2, 2), "+0.65m"),
        ]
        for rw, rh, fill, outline, dash, elev_lbl in contour_rings_knoll:
            # Offset each ring slightly for organic feel
            ox = 0.3 * scale
            oy = -0.2 * scale
            c.create_oval(knoll_cx - rw * scale + ox, knoll_cy - rh * scale + oy,
                          knoll_cx + rw * scale + ox, knoll_cy + rh * scale + oy,
                          fill=fill, outline=outline, width=1, dash=dash)
        c.create_text(knoll_cx, knoll_cy - 2, text="+0.65", fill="#3a3a38", font=("Consolas", 6))

        # Vegetation scatter around knoll base (small irregular dots)
        veg_knoll = [
            (-32, 82), (-30, 78), (-28, 88), (-20, 80), (-22, 85), (-18, 90),
            (-35, 85), (-26, 92), (-33, 79), (-19, 83), (-27, 76),
        ]
        for vy, vx in veg_knoll:
            vx_c, vy_c, _ = self.world_to_canvas(vx, vy, cw, ch)
            c.create_oval(vx_c - 1.5, vy_c - 1.5, vx_c + 1.5, vy_c + 1.5,
                          fill="#0d110c", outline="")

        # 5b. Central-East Ridge — elongated irregular contours
        ridge_cx, ridge_cy, _ = self.world_to_canvas(42.0, 22.0, cw, ch)
        contour_rings_ridge = [
            (6.5, 12.0, "#080807", "#151413", (4, 4), "+0.3m"),
            (4.2, 8.5, "#0c0b0a", "#1e1d1b", (2, 3), "+0.68m"),
        ]
        for rw, rh, fill, outline, dash, elev_lbl in contour_rings_ridge:
            ox = -0.4 * scale
            c.create_oval(ridge_cx - rw * scale + ox, ridge_cy - rh * scale,
                          ridge_cx + rw * scale + ox, ridge_cy + rh * scale,
                          fill=fill, outline=outline, width=1, dash=dash)
        c.create_text(ridge_cx, ridge_cy - 2, text="+0.68", fill="#3a3a38", font=("Consolas", 6))

        # Vegetation near ridge
        veg_ridge = [(25, 38), (27, 45), (24, 50), (19, 42), (30, 40), (22, 47)]
        for vx, vy in veg_ridge:
            vx_c, vy_c, _ = self.world_to_canvas(vx, vy, cw, ch)
            c.create_oval(vx_c - 1.5, vy_c - 1.5, vx_c + 1.5, vy_c + 1.5,
                          fill="#0d110c", outline="")

        # 5c. Rocky outcrops — irregular polygon clusters
        rock_clusters = [
            (52.0, -12.0, [(-7, -5), (-3, -8), (4, -6), (8, -1), (6, 5), (1, 7), (-5, 5), (-8, 1)]),
            (92.0, 28.0, [(-6, -4), (-1, -7), (5, -5), (7, 0), (5, 6), (-2, 7), (-6, 3)]),
        ]
        for rx, ry, pts_raw in rock_clusters:
            rcx, rcy, _ = self.world_to_canvas(rx, ry, cw, ch)
            pts = []
            for px_r, py_r in pts_raw:
                pts.extend([rcx + px_r * 0.7, rcy + py_r * 0.7])
            c.create_polygon(*pts, fill="#111110", outline="#1e1d1b", width=1, smooth=True)
            # Individual rock fragments
            for i, (px_r, py_r) in enumerate(pts_raw[::2]):
                c.create_oval(rcx + px_r * 0.4 - 1.5, rcy + py_r * 0.4 - 1.5,
                              rcx + px_r * 0.4 + 1.5, rcy + py_r * 0.4 + 1.5,
                              fill="#171716", outline="#252523")

        # 5d. Concrete Jersey barriers (trapezoidal cross-section shadow)
        barriers = [(70.0, -9.0), (32.0, 9.0), (68.0, 27.0)]
        for bx, by in barriers:
            bcx, bcy, _ = self.world_to_canvas(bx, by, cw, ch)
            bw = 0.8 * scale
            bh = 2.8 * scale
            # Shadow cast (offset southeast)
            c.create_rectangle(bcx - bw + 1.5, bcy - bh + 1.5, bcx + bw + 1.5, bcy + bh + 1.5,
                               fill="#030302", outline="")
            # Barrier body
            c.create_rectangle(bcx - bw, bcy - bh, bcx + bw, bcy + bh,
                               fill="#161615", outline="#282826", width=1)

        # ── 5e. ISO 20ft Shipping Containers (with cast shadow) ──────────────
        # West staging compound (10N, -38E) and East staging compound (110N, 38E)
        containers = [
            (10.0, -38.0, "#161c16", "#283629"),
            (110.0, 38.0, "#181a1e", "#282d38"),
        ]
        for cx_m, cy_m, c_fill, c_outline in containers:
            c_cx, c_cy, sc = self.world_to_canvas(cx_m, cy_m, cw, ch)
            hw = 1.22 * sc  # half-width
            hl = 3.03 * sc  # half-length

            # Cast shadow (offset 2.5px SE)
            c.create_rectangle(c_cx - hw + 2, c_cy - hl + 2, c_cx + hw + 2, c_cy + hl + 2,
                               fill="#020201", outline="")
            # Container body
            c.create_rectangle(c_cx - hw, c_cy - hl, c_cx + hw, c_cy + hl,
                               fill=c_fill, outline=c_outline, width=1)
            # Corrugated roof ribs
            rib_count = 5
            for i in range(rib_count):
                ry = c_cy - hl + (2 * hl) * (i + 0.5) / rib_count
                c.create_line(c_cx - hw + 1, ry, c_cx + hw - 1, ry, fill=c_outline, width=1)
            # Door end line
            c.create_line(c_cx - hw + 2, c_cy + hl - 2, c_cx + hw - 2, c_cy + hl - 2,
                          fill="#354236", width=1)

        # ── 5f. Partially Collapsed 2-Story Concrete Building (95N, 30E) ─────
        bld_cx, bld_cy, sc = self.world_to_canvas(95.0, 30.0, cw, ch)
        bw_px = 3.5 * sc
        bl_px = 3.5 * sc
        # Concrete floor footprint
        c.create_rectangle(bld_cx - bw_px, bld_cy - bl_px, bld_cx + bw_px, bld_cy + bl_px,
                           fill="#0a0a09", outline="#1c1b18", width=1)
        # Cast shadow from standing west wall
        c.create_rectangle(bld_cx - bw_px - 1, bld_cy - bl_px, bld_cx - bw_px + 3, bld_cy + bl_px + 2,
                           fill="#020201", outline="")
        # Standing west wall (heavy solid line)
        c.create_line(bld_cx - bw_px, bld_cy - bl_px, bld_cx - bw_px, bld_cy + bl_px,
                      fill="#3a3834", width=3)
        # Partial south wall
        c.create_line(bld_cx - bw_px, bld_cy + bl_px, bld_cx + 0.5 * sc, bld_cy + bl_px,
                      fill="#32302c", width=2.5)
        # Tilted collapsed roof slab (angled polygon with fracture lines)
        rf_p1_x = bld_cx - 1.5 * sc
        rf_p1_y = bld_cy - 2.5 * sc
        rf_p2_x = bld_cx + 3.2 * sc
        rf_p2_y = bld_cy - 2.0 * sc
        rf_p3_x = bld_cx + 2.8 * sc
        rf_p3_y = bld_cy + 2.5 * sc
        rf_p4_x = bld_cx - 1.0 * sc
        rf_p4_y = bld_cy + 2.0 * sc
        c.create_polygon(rf_p1_x, rf_p1_y, rf_p2_x, rf_p2_y, rf_p3_x, rf_p3_y, rf_p4_x, rf_p4_y,
                         fill="#11100e", outline="#252320", width=1)
        c.create_line(rf_p1_x + 5, rf_p1_y + 8, rf_p3_x - 6, rf_p3_y - 8,
                      fill="#1e1c18", width=1, dash=(3, 2))
        # Label
        c.create_text(bld_cx, bld_cy - bl_px - 6, text="COLLAPSED STRUCT",
                      fill="#383632", font=("Consolas", 5, "bold"))

        # ── 5g. Sandbag Emplacement & Damaged Wall ───────────────────────────
        # Sandbag defensive arc at (90N, 26E)
        sb_cx, sb_cy, _ = self.world_to_canvas(90.0, 26.0, cw, ch)
        c.create_line(sb_cx - 2.0 * sc, sb_cy - 0.5 * sc,
                      sb_cx, sb_cy + 1.2 * sc,
                      sb_cx + 1.8 * sc, sb_cy + 0.8 * sc,
                      fill="#2e2a22", width=2, smooth=True)
        # Damaged free-standing wall at (65N, 20E)
        dw_cx, dw_cy, _ = self.world_to_canvas(65.0, 20.0, cw, ch)
        c.create_line(dw_cx - 1.5 * sc, dw_cy - 2.2 * sc, dw_cx + 1.5 * sc, dw_cy + 2.2 * sc,
                      fill="#282622", width=2)
        c.create_line(dw_cx - 1.5 * sc + 1.5, dw_cy - 2.2 * sc + 1.5,
                      dw_cx + 1.5 * sc + 1.5, dw_cy + 2.2 * sc + 1.5,
                      fill="#040403", width=1)

        # ── 5h. FOB Staging Tent & Logistics Support Complex ─────────────────
        # FOB Staging Tent at (8N, -12E)
        fob_cx, fob_cy, _ = self.world_to_canvas(8.0, -12.0, cw, ch)
        tw = 2.5 * sc
        tl = 4.0 * sc
        # Tent floor
        c.create_rectangle(fob_cx - tw, fob_cy - tl, fob_cx + tw, fob_cy + tl,
                           fill="#0c0e0b", outline="#1e241c", width=1)
        # Tent center ridge line
        c.create_line(fob_cx, fob_cy - tl, fob_cx, fob_cy + tl, fill="#2c3629", width=1.5)
        # Guy lines
        for gy in [-tl * 0.8, 0, tl * 0.8]:
            c.create_line(fob_cx - tw, fob_cy + gy, fob_cx - tw - 3, fob_cy + gy, fill="#161a14", width=1)
            c.create_line(fob_cx + tw, fob_cy + gy, fob_cx + tw + 3, fob_cy + gy, fill="#161a14", width=1)
        c.create_text(fob_cx, fob_cy, text="FOB STAGING", fill="#344030", font=("Consolas", 5, "bold"))

        # Parked Utility Truck at (12N, -20E)
        trk_cx, trk_cy, _ = self.world_to_canvas(12.0, -20.0, cw, ch)
        trw = 1.1 * sc
        trl = 2.75 * sc
        # Shadow
        c.create_rectangle(trk_cx - trw + 1.5, trk_cy - trl + 1.5, trk_cx + trw + 1.5, trk_cy + trl + 1.5,
                           fill="#020201", outline="")
        # Flatbed
        c.create_rectangle(trk_cx - trw, trk_cy - trl * 0.3, trk_cx + trw, trk_cy + trl,
                           fill="#141413", outline="#222220", width=1)
        # Cab (forward/north)
        c.create_rectangle(trk_cx - trw * 0.95, trk_cy - trl, trk_cx + trw * 0.95, trk_cy - trl * 0.3,
                           fill="#242320", outline="#383632", width=1)
        # Windshield line
        c.create_line(trk_cx - trw * 0.8, trk_cy - trl * 0.8, trk_cx + trw * 0.8, trk_cy - trl * 0.8,
                      fill="#0e1012", width=1.5)

        # Supply Pallets at (105N, 36E) & Generator at (5N, -8E)
        pal_cx, pal_cy, _ = self.world_to_canvas(105.0, 36.0, cw, ch)
        c.create_rectangle(pal_cx - 1.2 * sc, pal_cy - 0.8 * sc, pal_cx + 1.2 * sc, pal_cy + 0.8 * sc,
                           fill="#1c1812", outline="#2e261a", width=1)
        gen_cx, gen_cy, _ = self.world_to_canvas(5.0, -8.0, cw, ch)
        c.create_rectangle(gen_cx - 0.5 * sc, gen_cy - 0.6 * sc, gen_cx + 0.5 * sc, gen_cy + 0.6 * sc,
                           fill="#241a08", outline="#3d2c10", width=1)

        # ── 5i. Disaster Rubble & Collapse Debris Clusters ───────────────────
        # Matching Gazebo: Cluster 1 at (72N, 15E), Cluster 2 at (25N, -18E)
        rubble_zones = [
            (72.0, 15.0, "DEBRIS ZONE 1"),
            (25.0, -18.0, "DEBRIS ZONE 2"),
        ]
        for rx_m, ry_m, r_tag in rubble_zones:
            r_cx, r_cy, sc = self.world_to_canvas(rx_m, ry_m, cw, ch)
            rw_px = 3.5 * sc
            # Irregular rubble perimeter
            c.create_oval(r_cx - rw_px, r_cy - rw_px * 0.65,
                          r_cx + rw_px, r_cy + rw_px * 0.65,
                          fill="#0e0d0b", outline="#1a1917", width=1, dash=(2, 3))
            # Scattered rubble fragments inside
            rng = random.Random(int(rx_m * 100 + ry_m * 10))
            for _ in range(14):
                fx = r_cx + rng.uniform(-rw_px * 0.75, rw_px * 0.75)
                fy = r_cy + rng.uniform(-rw_px * 0.50, rw_px * 0.50)
                fs = rng.uniform(1.0, 2.5)
                c.create_rectangle(fx - fs, fy - fs, fx + fs, fy + fs,
                                   fill="#151412", outline="#22201c", width=1)
            # Rebar wire lines protruding from rubble
            c.create_line(r_cx - 1.2 * sc, r_cy - 0.5 * sc, r_cx - 2.0 * sc, r_cy - 1.0 * sc,
                          fill="#2a241e", width=1)
            c.create_line(r_cx + 0.8 * sc, r_cy + 0.6 * sc, r_cx + 1.8 * sc, r_cy + 1.1 * sc,
                          fill="#2a241e", width=1)
            c.create_text(r_cx, r_cy + rw_px * 0.75 + 4, text=r_tag,
                          fill="#3a3834", font=("Consolas", 5))

        # ── 5j. Mud Puddles (Standing Water with Specular Tint) ───────────────
        puddles = [(45.0, -8.0, 2.0, 1.4), (80.0, 12.0, 1.8, 1.2), (95.0, -25.0, 2.2, 1.5)]
        for mx, my, pw, ph in puddles:
            pud_x, pud_y, _ = self.world_to_canvas(mx, my, cw, ch)
            c.create_oval(pud_x - pw * scale * 0.5, pud_y - ph * scale * 0.5,
                          pud_x + pw * scale * 0.5, pud_y + ph * scale * 0.5,
                          fill="#06080a", outline="#12161a", width=1)

        # ── 5k. Tire Tracks / Vehicle Ruts ───────────────────────────────────
        tire_tracks = [
            # Track from helipad to west compound / truck
            [(2.0, -2.0), (6.0, -8.0), (10.0, -15.0), (12.0, -20.0), (10.0, -35.0)],
            [(3.0, -1.0), (7.0, -7.0), (11.0, -14.0), (13.0, -19.0), (11.0, -34.0)],
            # Central supply corridor
            [(15.0, 0.0), (35.0, 5.0), (55.0, 2.0), (75.0, 8.0), (95.0, 15.0), (110.0, 32.0)],
            [(16.0, 1.5), (36.0, 6.5), (56.0, 3.5), (76.0, 9.5), (96.0, 16.5), (111.0, 33.5)],
        ]
        for track in tire_tracks:
            pts = []
            for tx_m, ty_m in track:
                tx_c, ty_c, _ = self.world_to_canvas(tx_m, ty_m, cw, ch)
                pts.extend([tx_c, ty_c])
            if len(pts) >= 4:
                c.create_line(*pts, fill="#090908", width=1.2, smooth=True)

        # ── 6. Survey Swath Corridors ────────────────────────────────────────
        lane_y_coords = [-36.0, -18.0, 0.0, 18.0, 36.0]
        cur_lane_idx = (self.mission.current_lane - 1) if self.mission else 0

        for idx, ly in enumerate(lane_y_coords):
            sw_l_x0, sw_l_y0, _ = self.world_to_canvas(6.0, ly - 9.0, cw, ch)
            sw_r_x1, sw_r_y1, _ = self.world_to_canvas(114.0, ly + 9.0, cw, ch)
            if idx == cur_lane_idx and self.mission and "SURVEY" in self.mission.state.value:
                c.create_rectangle(sw_l_x0, sw_r_y1, sw_r_x1, sw_l_y0,
                                   fill="#0e0408", outline="#220c14", width=1)
            else:
                c.create_rectangle(sw_l_x0, sw_r_y1, sw_r_x1, sw_l_y0,
                                   fill="", outline="#0f0f0e", width=1, dash=(1, 5))

        # ── 7. Planned Survey Lines & Dubins Arcs ────────────────────────────
        if self.mission is not None:
            for idx, lane in enumerate(self.mission.lanes):
                sx, sy, _ = self.world_to_canvas(lane["start"][0], lane["start"][1], cw, ch)
                ex, ey, _ = self.world_to_canvas(lane["end"][0], lane["end"][1], cw, ch)
                is_active = (idx == cur_lane_idx and "SURVEY" in self.mission.state.value)
                line_col = "#6b1a1a" if is_active else "#1e1e1d"
                c.create_line(sx, sy, ex, ey, fill=line_col, dash=(3, 5), width=1.2 if is_active else 0.8)
                c.create_text((sx + ex) / 2 + 8, (sy + ey) / 2, text=f"L{idx+1}",
                              fill=self.c_red_bright if is_active else "#3a3a38",
                              font=("Consolas", 7, "bold" if is_active else "normal"))
                # Waypoint dots
                c.create_oval(sx - 2, sy - 2, sx + 2, sy + 2, fill="#333332", outline="")
                c.create_oval(ex - 2, ey - 2, ex + 2, ey + 2, fill="#333332", outline="")

            # Dubins turn arcs
            for arc in self.mission.dubins_arcs:
                pts = []
                for st in range(22):
                    rad = (st / 21.0) * math.pi
                    if arc["type"] == "NORTH_TURN":
                        ax = arc["center"][0] + arc["radius"] * math.sin(rad)
                        ay = arc["center"][1] - arc["radius"] * math.cos(rad)
                    else:
                        ax = arc["center"][0] - arc["radius"] * math.sin(rad)
                        ay = arc["center"][1] - arc["radius"] * math.cos(rad)
                    cx_p, cy_p, _ = self.world_to_canvas(ax, ay, cw, ch)
                    pts.extend([cx_p, cy_p])
                c.create_line(*pts, fill="#1e1e1d", dash=(2, 4), width=0.8)

        # ── 8. Flight Trail (gradient crimson breadcrumbs) ───────────────────
        if len(self.trail) > 1:
            recent_split = max(0, len(self.trail) - 35)
            if recent_split > 1:
                hist_pts = []
                for tpt in self.trail[:recent_split + 1]:
                    tx_c, ty_c, _ = self.world_to_canvas(tpt[0], tpt[1], cw, ch)
                    hist_pts.extend([tx_c, ty_c])
                c.create_line(*hist_pts, fill="#301018", width=1.5)
            active_pts = []
            for tpt in self.trail[recent_split:]:
                tx_c, ty_c, _ = self.world_to_canvas(tpt[0], tpt[1], cw, ch)
                active_pts.extend([tx_c, ty_c])
            if len(active_pts) >= 4:
                c.create_line(*active_pts, fill=self.c_red_bright, width=2.0)

        # ── 9. Corner Survey Markers ─────────────────────────────────────────
        corners = [(0.0, -45.0), (120.0, -45.0), (120.0, 45.0), (0.0, 45.0)]
        for px_c, py_c in corners:
            tc_x, tc_y, _ = self.world_to_canvas(px_c, py_c, cw, ch)
            # Simple crosshair marker
            c.create_line(tc_x - 5, tc_y, tc_x + 5, tc_y, fill="#2a2a28", width=1)
            c.create_line(tc_x, tc_y - 5, tc_x, tc_y + 5, fill="#2a2a28", width=1)
            c.create_oval(tc_x - 1.5, tc_y - 1.5, tc_x + 1.5, tc_y + 1.5,
                          fill=self.c_red_bright, outline="")

        # ── 10. Helipad / Launch Point ───────────────────────────────────────
        h_x, h_y, _ = self.world_to_canvas(0, 0, cw, ch)
        c.create_oval(h_x - 12, h_y - 12, h_x + 12, h_y + 12, outline="#1e1e1d", width=1.5)
        c.create_oval(h_x - 7, h_y - 7, h_x + 7, h_y + 7, outline="#2a2a28", width=1, dash=(2, 2))
        c.create_text(h_x, h_y, text="H", fill="#d0d0d0", font=("Consolas", 9, "bold"))

        # ── 11. Targets with Priority Badges & Covariance ────────────────────
        if self.mission is not None:
            for t in self.mission.detector.get_all_targets():
                tx_c, ty_c, sc = self.world_to_canvas(t.x, t.y, cw, ch)
                r_cov = max(7, t.pos_std_dev * sc)

                pri_str = getattr(t, 'priority', 'P2 - HIGH')
                pri_tok = pri_str.split(" ")[0] if " " in pri_str else "P2"
                t_name = getattr(t, 'triage_name', t.color_name)

                if "P1" in pri_tok:
                    cov_outline = "#881322"
                    glyph_fill = "#4c0519"
                    glyph_outline = "#ff2b4b"
                    badge_bg = "#0a0203"
                    badge_border = "#661020"
                elif "P2" in pri_tok:
                    cov_outline = "#55101a"
                    glyph_fill = self.c_red_dark
                    glyph_outline = self.c_red_bright
                    badge_bg = "#060303"
                    badge_border = "#2b0a0a"
                else:
                    cov_outline = "#222221"
                    glyph_fill = "#141413"
                    glyph_outline = "#454544"
                    badge_bg = "#060605"
                    badge_border = "#222221"

                # Covariance uncertainty ring
                c.create_oval(tx_c - r_cov, ty_c - r_cov, tx_c + r_cov, ty_c + r_cov,
                              outline=cov_outline, width=1, dash=(3, 2))

                # Target glyph (diamond for P1, square for P2/P3)
                if "P1" in pri_tok:
                    c.create_polygon(tx_c, ty_c - 5, tx_c + 5, ty_c, tx_c, ty_c + 5, tx_c - 5, ty_c,
                                     fill=glyph_fill, outline=glyph_outline, width=1.5)
                else:
                    c.create_rectangle(tx_c - 4, ty_c - 4, tx_c + 4, ty_c + 4,
                                       fill=glyph_fill, outline=glyph_outline, width=1.5)

                tag_str = f"#{t.target_id:02d} [{pri_tok}] {t_name}"

                # Flip label if near right edge
                if tx_c > (cw - 140):
                    tag_x = tx_c - 10
                    tag_anchor = "e"
                    box_x0 = tag_x - len(tag_str) * 5.5 - 4
                    box_x1 = tag_x + 2
                else:
                    tag_x = tx_c + 10
                    tag_anchor = "w"
                    box_x0 = tag_x - 2
                    box_x1 = tag_x + len(tag_str) * 5.5 + 4

                c.create_rectangle(box_x0, ty_c - 7, box_x1, ty_c + 7,
                                   fill=badge_bg, outline=badge_border, width=1)
                c.create_text(tag_x, ty_c, text=tag_str,
                              anchor=tag_anchor, fill="#e0e0de", font=("Consolas", 7, "bold"))

        # ── 12. Drone & Optical FOV Projection ───────────────────────────────
        dp_x, dp_y, scale = self.world_to_canvas(px, py, cw, ch)
        yaw_rad = math.radians(yaw_deg)
        alt = max(1.0, -pz)

        fov_ahead_ctr = alt * 1.0
        fov_half_w = alt * 0.84
        fov_depth_half = alt * 0.45

        p_far_l_x = px + (fov_ahead_ctr + fov_depth_half) * math.cos(yaw_rad) - (fov_half_w * 1.15) * math.sin(yaw_rad)
        p_far_l_y = py + (fov_ahead_ctr + fov_depth_half) * math.sin(yaw_rad) + (fov_half_w * 1.15) * math.cos(yaw_rad)
        p_far_r_x = px + (fov_ahead_ctr + fov_depth_half) * math.cos(yaw_rad) + (fov_half_w * 1.15) * math.sin(yaw_rad)
        p_far_r_y = py + (fov_ahead_ctr + fov_depth_half) * math.sin(yaw_rad) - (fov_half_w * 1.15) * math.cos(yaw_rad)
        p_near_r_x = px + (fov_ahead_ctr - fov_depth_half) * math.cos(yaw_rad) + (fov_half_w * 0.85) * math.sin(yaw_rad)
        p_near_r_y = py + (fov_ahead_ctr - fov_depth_half) * math.sin(yaw_rad) - (fov_half_w * 0.85) * math.cos(yaw_rad)
        p_near_l_x = px + (fov_ahead_ctr - fov_depth_half) * math.cos(yaw_rad) - (fov_half_w * 0.85) * math.sin(yaw_rad)
        p_near_l_y = py + (fov_ahead_ctr - fov_depth_half) * math.sin(yaw_rad) + (fov_half_w * 0.85) * math.cos(yaw_rad)

        # Coverage accumulation
        now = time.time()
        if alt >= 3.0 and (now - self.last_coverage_time) >= 0.25:
            self.last_coverage_time = now
            self.coverage_patches.append((
                p_near_l_x, p_near_l_y, p_far_l_x, p_far_l_y,
                p_far_r_x, p_far_r_y, p_near_r_x, p_near_r_y
            ))
            if len(self.coverage_patches) > 1500:
                self.coverage_patches = self.coverage_patches[-1500:]

        c_fl_x, c_fl_y, _ = self.world_to_canvas(p_far_l_x, p_far_l_y, cw, ch)
        c_fr_x, c_fr_y, _ = self.world_to_canvas(p_far_r_x, p_far_r_y, cw, ch)
        c_nr_x, c_nr_y, _ = self.world_to_canvas(p_near_r_x, p_near_r_y, cw, ch)
        c_nl_x, c_nl_y, _ = self.world_to_canvas(p_near_l_x, p_near_l_y, cw, ch)

        # FOV footprint (subtle)
        c.create_polygon(dp_x, dp_y, c_fl_x, c_fl_y, c_fr_x, c_fr_y,
                         fill="#0a0305", outline="#1a0810", width=1)
        c.create_polygon(c_nl_x, c_nl_y, c_fl_x, c_fl_y, c_fr_x, c_fr_y, c_nr_x, c_nr_y,
                         fill="#0e0508", outline="#280d14", width=1)

        # Drone silhouette (X-frame quadcopter)
        arm = 7.0
        c.create_line(dp_x - arm, dp_y - arm, dp_x + arm, dp_y + arm, fill="#c0c0be", width=1.5)
        c.create_line(dp_x - arm, dp_y + arm, dp_x + arm, dp_y - arm, fill="#c0c0be", width=1.5)
        for rx in [-arm, arm]:
            for ry in [-arm, arm]:
                c.create_oval(dp_x + rx - 2.5, dp_y + ry - 2.5, dp_x + rx + 2.5, dp_y + ry + 2.5,
                              fill=self.c_red, outline=self.c_red_bright)
        c.create_oval(dp_x - 2, dp_y - 2, dp_x + 2, dp_y + 2, fill="#e8e8e6", outline="")

        # Heading vector
        hdg_len = 14.0
        hx = dp_x + hdg_len * math.sin(yaw_rad)
        hy = dp_y - hdg_len * math.cos(yaw_rad)
        c.create_line(dp_x, dp_y, hx, hy, fill=self.c_red_bright, width=2, arrow=tk.LAST)

        # ── 13. Compass Rose (Top-Right) ─────────────────────────────────────
        comp_x = cw - 22.0
        comp_y = 22.0
        c.create_polygon(comp_x, comp_y - 10, comp_x - 3, comp_y - 1, comp_x + 3, comp_y - 1,
                         fill=self.c_red_bright, outline="")
        c.create_polygon(comp_x, comp_y + 7, comp_x - 3, comp_y - 1, comp_x + 3, comp_y - 1,
                         fill="#1a1a19", outline="")
        c.create_text(comp_x, comp_y + 14, text="N", fill="#a0a09e", font=("Consolas", 7, "bold"))

        # ── 14. Scale Bar (Bottom-Left) ──────────────────────────────────────
        bar_len_m = 20.0
        bar_px = bar_len_m * scale
        sb_x = 20.0
        sb_y = ch - 14.0
        c.create_line(sb_x, sb_y, sb_x + bar_px, sb_y, fill="#2a2a28", width=1.5)
        c.create_line(sb_x, sb_y - 3, sb_x, sb_y + 3, fill="#2a2a28", width=1)
        c.create_line(sb_x + bar_px, sb_y - 3, sb_x + bar_px, sb_y + 3, fill="#2a2a28", width=1)
        c.create_text(sb_x + bar_px / 2.0, sb_y - 7, text=f"{int(bar_len_m)} m",
                      fill="#555553", font=("Consolas", 6))

        # ── 15. Zoom Indicator ───────────────────────────────────────────────
        if abs(self.zoom_factor - 1.0) > 0.05:
            c.create_text(28.0, 12.0, text=f"×{self.zoom_factor:.1f}",
                          fill="#555553", font=("Consolas", 7, "bold"))

