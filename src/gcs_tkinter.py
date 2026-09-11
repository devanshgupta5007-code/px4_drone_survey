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
        """Draws upgraded multi-tone tactical map mirroring real-world proving ground features."""
        c = self.map_canvas
        cw = c.winfo_width()
        ch = c.winfo_height()
        if cw < 60 or ch < 60:
            return

        c.delete("all")

        # 1. Compacted Gravel Perimeter Access Road (Outer Proving Ground Envelope)
        r_bl_x, r_bl_y, _ = self.world_to_canvas(-4.0, -49.0, cw, ch)
        r_tr_x, r_tr_y, _ = self.world_to_canvas(124.0, 49.0, cw, ch)
        c.create_rectangle(r_bl_x, r_tr_y, r_tr_x, r_bl_y, fill="#080808", outline="#181818", width=1)

        # 2. Weathered Tarmac Survey Apron (Inner Surface)
        a_bl_x, a_bl_y, _ = self.world_to_canvas(0.0, -45.0, cw, ch)
        a_tr_x, a_tr_y, _ = self.world_to_canvas(120.0, 45.0, cw, ch)
        c.create_rectangle(a_bl_x, a_tr_y, a_tr_x, a_bl_y, fill="#030303", outline="#1c1c1c", width=1.5)

        # 2b. Photogrammetric Ground Coverage Heatmap (Aggregated Area Surveyed)
        for p in self.coverage_patches:
            cp1_x, cp1_y, _ = self.world_to_canvas(p[0], p[1], cw, ch)
            cp2_x, cp2_y, _ = self.world_to_canvas(p[2], p[3], cw, ch)
            cp3_x, cp3_y, _ = self.world_to_canvas(p[4], p[5], cw, ch)
            cp4_x, cp4_y, _ = self.world_to_canvas(p[6], p[7], cw, ch)
            c.create_polygon(cp1_x, cp1_y, cp2_x, cp2_y, cp3_x, cp3_y, cp4_x, cp4_y,
                             fill="#120508", outline="")

        # 3. Coordinate Grid Lines every 20m (Neutral Dark Charcoal)
        grid_col = "#0c0c0c"
        tick_col = "#383838"

        for x in range(0, 121, 20):
            p1_x, p1_y, _ = self.world_to_canvas(x, -45, cw, ch)
            p2_x, p2_y, _ = self.world_to_canvas(x, 45, cw, ch)
            c.create_line(p1_x, p1_y, p2_x, p2_y, fill=grid_col, width=1)
            c.create_text(p1_x - 14, p1_y, text=f"{x}N", fill=tick_col, font=("Consolas", 7))

        for y in range(-40, 41, 20):
            p1_x, p1_y, _ = self.world_to_canvas(0, y, cw, ch)
            p2_x, p2_y, _ = self.world_to_canvas(120, y, cw, ch)
            c.create_line(p1_x, p1_y, p2_x, p2_y, fill=grid_col, width=1)
            c.create_text(p1_x, p1_y + 10, text=f"{y:+d}E", fill=tick_col, font=("Consolas", 7))

        # 4. Photogrammetric Ground Control Points (GCP Checkered Crosses on Tarmac)
        gcps = [(-25.0, 30.0, "GCP-1"), (25.0, 30.0, "GCP-2"), (-25.0, 90.0, "GCP-3"), (25.0, 90.0, "GCP-4")]
        for g_east, g_north, g_lbl in gcps:
            gx, gy, scale = self.world_to_canvas(g_north, g_east, cw, ch)
            c.create_line(gx - 4, gy, gx + 4, gy, fill="#383838", width=1)
            c.create_line(gx, gy - 4, gx, gy + 4, fill="#383838", width=1)
            c.create_rectangle(gx - 2, gy - 2, gx + 2, gy + 2, outline="#4a4a4a", width=1)
            c.create_text(gx + 6, gy - 6, text=g_lbl, fill="#282828", font=("Consolas", 5))

        # 5. Natural Contoured Terrain Features (Neutral Monotone Terraces)
        # 5a. Northwest Rolling Knoll (Multi-Contoured Terraces: +0.22m, +0.44m, +0.65m)
        knoll_x, knoll_y, scale = self.world_to_canvas(85.0, -25.0, cw, ch)
        # Base terrace (+0.22m)
        c.create_oval(knoll_x - 11.0 * scale, knoll_y - 8.0 * scale, knoll_x + 11.0 * scale, knoll_y + 8.0 * scale,
                      fill="#080808", outline="#181818", width=1, dash=(3, 3))
        # Mid terrace (+0.44m)
        c.create_oval(knoll_x - 7.5 * scale, knoll_y - 5.5 * scale, knoll_x + 7.5 * scale, knoll_y + 5.5 * scale,
                      fill="#0e0e0e", outline="#202020", width=1, dash=(2, 2))
        # Crest terrace (+0.65m)
        c.create_oval(knoll_x - 4.5 * scale, knoll_y - 3.2 * scale, knoll_x + 4.5 * scale, knoll_y + 3.2 * scale,
                      fill="#141414", outline="#2a2a2a", width=1)
        c.create_text(knoll_x, knoll_y, text="KNOLL +0.65M", fill="#555555", font=("Consolas", 6, "bold"))

        # 5b. Central-East Natural Ridge (Multi-Contoured Spine: +0.30m, +0.68m)
        ridge_x, ridge_y, _ = self.world_to_canvas(42.0, 22.0, cw, ch)
        # Base ridge (+0.30m)
        c.create_oval(ridge_x - 8.0 * scale, ridge_y - 11.0 * scale, ridge_x + 8.0 * scale, ridge_y + 11.0 * scale,
                      fill="#090909", outline="#181818", width=1, dash=(3, 3))
        # Spine ridge (+0.68m)
        c.create_oval(ridge_x - 5.0 * scale, ridge_y - 7.5 * scale, ridge_x + 5.0 * scale, ridge_y + 7.5 * scale,
                      fill="#121212", outline="#242424", width=1)
        c.create_text(ridge_x, ridge_y, text="RIDGE +0.68M", fill="#555555", font=("Consolas", 6, "bold"))

        # 5c. Rocky Outcrop Formations (Monotone Rock Clusters Alpha & Beta)
        for bx, by, b_lbl in [(52.0, -12.0, "ROCKS A"), (92.0, 28.0, "ROCKS B")]:
            b_cx, b_cy, _ = self.world_to_canvas(bx, by, cw, ch)
            c.create_polygon(b_cx - 6, b_cy - 4, b_cx + 2, b_cy - 6, b_cx + 7, b_cy + 2, b_cx + 1, b_cy + 6, b_cx - 5, b_cy + 4,
                             fill="#151515", outline="#282828", width=1)
            c.create_text(b_cx, b_cy - 9, text=b_lbl, fill="#404040", font=("Consolas", 5))

        # 5d. Concrete Jersey Barriers between lanes
        barriers = [(70.0, -9.0, "BARRIER-1"), (32.0, 9.0, "BARRIER-2"), (68.0, 27.0, "BARRIER-3")]
        for ox, oy, o_tag in barriers:
            oc_x, oc_y, _ = self.world_to_canvas(ox, oy, cw, ch)
            bw = 0.8 * scale
            bh = 2.8 * scale
            c.create_rectangle(oc_x - bw, oc_y - bh, oc_x + bw, oc_y + bh,
                               fill="#181818", outline="#2e2e2e", width=1)
            c.create_text(oc_x, oc_y, text=o_tag, fill="#505050", font=("Consolas", 5))

        # 5e. Industrial ISO 20ft Shipping Containers (West & East Zones)
        containers = [(78.0, -18.0, "ISO 20FT [WEST]"), (38.0, 18.0, "ISO 20FT [EAST]")]
        for cx_m, cy_m, c_tag in containers:
            c_cx, c_cy, scale = self.world_to_canvas(cx_m, cy_m, cw, ch)
            cw_px = 1.22 * scale  # half-width = 1.22m
            cl_px = 3.03 * scale  # half-length = 3.03m
            c.create_rectangle(c_cx - cw_px, c_cy - cl_px, c_cx + cw_px, c_cy + cl_px,
                               fill="#161916", outline="#2f3b2f", width=1.5)
            # Corrugated roof ribs
            for cor_i in [-2, -1, 0, 1, 2]:
                c.create_line(c_cx - cw_px + 2, c_cy + cor_i * (cl_px / 3.0),
                              c_cx + cw_px - 2, c_cy + cor_i * (cl_px / 3.0),
                              fill="#222822", width=1)
            c.create_text(c_cx, c_cy, text=c_tag, fill="#6b806b", font=("Consolas", 5, "bold"))

        # 5f. Disaster Rubble & Masonry Collapse Zones
        rubble_zones = [(60.0, 0.0, "COLLAPSE ZONE 1"), (25.0, -22.0, "COLLAPSE ZONE 2")]
        for rx_m, ry_m, r_tag in rubble_zones:
            r_cx, r_cy, scale = self.world_to_canvas(rx_m, ry_m, cw, ch)
            rw_px = 3.2 * scale
            c.create_oval(r_cx - rw_px, r_cy - rw_px * 0.7, r_cx + rw_px, r_cy + rw_px * 0.7,
                          fill="#141312", outline="#2b2825", width=1, dash=(2, 2))
            c.create_text(r_cx, r_cy, text=r_tag, fill="#5a5652", font=("Consolas", 5, "bold"))

        # 6. 18m Survey Swath Corridors & Turnaround Waypoints
        lane_y_coords = [-36.0, -18.0, 0.0, 18.0, 36.0]
        cur_lane_idx = (self.mission.current_lane - 1) if self.mission else 0

        for idx, ly in enumerate(lane_y_coords):
            sw_l_x0, sw_l_y0, _ = self.world_to_canvas(6.0, ly - 9.0, cw, ch)
            sw_r_x1, sw_r_y1, _ = self.world_to_canvas(114.0, ly + 9.0, cw, ch)

            if idx == cur_lane_idx and self.mission and "SURVEY" in self.mission.state.value:
                # Active Swath Corridor: subtle dark crimson illumination
                c.create_rectangle(sw_l_x0, sw_r_y1, sw_r_x1, sw_l_y0,
                                   fill="#120609", outline="#2e0f17", width=1)
            else:
                # Inactive Swath Corridor: faint neutral boundaries
                c.create_rectangle(sw_l_x0, sw_r_y1, sw_r_x1, sw_l_y0,
                                   fill="", outline="#121212", width=1, dash=(1, 4))

        # 7. Planned Survey Centerlines & Turnaround Waypoints
        if self.mission is not None:
            for idx, lane in enumerate(self.mission.lanes):
                sx, sy, _ = self.world_to_canvas(lane["start"][0], lane["start"][1], cw, ch)
                ex, ey, _ = self.world_to_canvas(lane["end"][0], lane["end"][1], cw, ch)
                is_active_lane = (idx == cur_lane_idx and "SURVEY" in self.mission.state.value)
                line_col = "#7f1d1d" if is_active_lane else "#252525"
                c.create_line(sx, sy, ex, ey, fill=line_col, dash=(4, 4), width=1.2 if is_active_lane else 1.0)
                c.create_text((sx + ex) / 2 + 10, (sy + ey) / 2, text=f"L{idx+1}",
                              fill=self.c_red_bright if is_active_lane else "#555555",
                              font=("Consolas", 8, "bold" if is_active_lane else "normal"))

                # Waypoint pips at entry/exit
                c.create_oval(sx - 2, sy - 2, sx + 2, sy + 2, fill="#404040", outline="")
                c.create_oval(ex - 2, ey - 2, ex + 2, ey + 2, fill="#404040", outline="")

            # Continuous Dubins Turn Arcs (R = 9.0m)
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
                c.create_line(*pts, fill="#252525", dash=(2, 3), width=1)

        # 8. Dual-Tone Flown Breadcrumb Flight Trail (Deep Crimson to Vivid Red Vector)
        if len(self.trail) > 1:
            recent_split = max(0, len(self.trail) - 35)
            # Older historic trail
            if recent_split > 1:
                hist_pts = []
                for tpt in self.trail[:recent_split + 1]:
                    tx_c, ty_c, _ = self.world_to_canvas(tpt[0], tpt[1], cw, ch)
                    hist_pts.extend([tx_c, ty_c])
                c.create_line(*hist_pts, fill="#3d1219", width=1.5)

            # Recent active flight vector
            active_pts = []
            for tpt in self.trail[recent_split:]:
                tx_c, ty_c, _ = self.world_to_canvas(tpt[0], tpt[1], cw, ch)
                active_pts.extend([tx_c, ty_c])
            c.create_line(*active_pts, fill=self.c_red_bright, width=2.0)

        # 9. ICAO Aviation Pylon Towers at 4 Corners
        corners = [(0.0, -45.0, "SW"), (120.0, -45.0, "NW"), (120.0, 45.0, "NE"), (0.0, 45.0, "SE")]
        for px_c, py_c, c_tag in corners:
            tc_x, tc_y, _ = self.world_to_canvas(px_c, py_c, cw, ch)
            # Monotone perimeter pylon beacon marker
            c.create_rectangle(tc_x - 3.5, tc_y - 3.5, tc_x + 3.5, tc_y + 3.5, outline="#333333", fill="#111111", width=1)
            c.create_oval(tc_x - 1.5, tc_y - 1.5, tc_x + 1.5, tc_y + 1.5, fill=self.c_red_bright, outline="")
            # Tactical corner bracket
            dx_b = 6 if py_c > 0 else -6
            dy_b = -6 if px_c > 60 else 6
            c.create_line(tc_x, tc_y, tc_x + dx_b, tc_y, fill=self.c_red_bright, width=1.5)
            c.create_line(tc_x, tc_y, tc_x, tc_y + dy_b, fill=self.c_red_bright, width=1.5)

        # 10. Operational Helipad Launch Complex at (0, 0)
        h_x, h_y, _ = self.world_to_canvas(0, 0, cw, ch)
        c.create_oval(h_x - 10, h_y - 10, h_x + 10, h_y + 10, outline="#242424", width=1.5)
        c.create_oval(h_x - 6, h_y - 6, h_x + 6, h_y + 6, outline="#383838", width=1, dash=(2, 2))
        c.create_text(h_x, h_y, text="H", fill=self.c_text_high, font=("Consolas", 8, "bold"))
        c.create_text(h_x, h_y + 13, text="LAUNCH/RTL", fill="#555555", font=("Consolas", 6))

        # 11. Confirmed Contacts with Triage Priority Badges & Covariance Uncertainty
        if self.mission is not None:
            for t in self.mission.detector.get_all_targets():
                tx_c, ty_c, scale = self.world_to_canvas(t.x, t.y, cw, ch)
                r_cov = max(7, t.pos_std_dev * scale)

                pri_str = getattr(t, 'priority', 'P2 - HIGH')
                pri_tok = pri_str.split(" ")[0] if " " in pri_str else "P2"
                t_name = getattr(t, 'triage_name', t.color_name)

                if "P1" in pri_tok:
                    cov_outline = "#881322"
                    glyph_fill = "#4c0519"
                    glyph_outline = "#ff2b4b"
                    badge_border = "#991b1b"
                elif "P2" in pri_tok:
                    cov_outline = "#66101d"
                    glyph_fill = self.c_red_dark
                    glyph_outline = self.c_red_bright
                    badge_border = "#2b0a0a"
                else:
                    cov_outline = "#262626"
                    glyph_fill = "#171717"
                    glyph_outline = "#525252"
                    badge_border = "#262626"

                # Covariance Uncertainty Circle
                c.create_oval(tx_c - r_cov, ty_c - r_cov, tx_c + r_cov, ty_c + r_cov,
                              outline=cov_outline, width=1)

                # Target Geometry Glyphs
                c.create_rectangle(tx_c - 4, ty_c - 4, tx_c + 4, ty_c + 4,
                                   fill=glyph_fill, outline=glyph_outline, width=1.5)

                tag_str = f"#{t.target_id:02d} [{pri_tok}] {t_name}"

                # Boundary edge guard: flip label anchor if close to right margin
                if tx_c > (cw - 130):
                    tag_x = tx_c - 8
                    tag_anchor = "e"
                    box_x0 = tag_x - len(tag_str) * 6.0 - 4
                    box_x1 = tag_x + 2
                else:
                    tag_x = tx_c + 8
                    tag_anchor = "w"
                    box_x0 = tag_x - 2
                    box_x1 = tag_x + len(tag_str) * 6.0 + 4

                # Crisp High-Contrast Backing Badge
                c.create_rectangle(box_x0, ty_c - 7, box_x1, ty_c + 7,
                                   fill="#050505", outline=badge_border, width=1)
                c.create_text(tag_x, ty_c, text=tag_str,
                              anchor=tag_anchor, fill=self.c_text_high, font=("Consolas", 7, "bold"))

        # 12. Drone Quadcopter Silhouette & Optical Nadir Ground Projection
        dp_x, dp_y, scale = self.world_to_canvas(px, py, cw, ch)
        yaw_rad = math.radians(yaw_deg)
        alt = max(1.0, -pz)

        # 45-degree Forward Pitch Optical Nadir Ground Cone
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

        # Accumulate photogrammetric ground coverage patch when airborne
        now = time.time()
        if alt >= 3.0 and (now - self.last_coverage_time) >= 0.25:
            self.last_coverage_time = now
            self.coverage_patches.append((
                p_near_l_x, p_near_l_y,
                p_far_l_x, p_far_l_y,
                p_far_r_x, p_far_r_y,
                p_near_r_x, p_near_r_y
            ))
            if len(self.coverage_patches) > 1500:
                self.coverage_patches = self.coverage_patches[-1500:]

        c_fl_x, c_fl_y, _ = self.world_to_canvas(p_far_l_x, p_far_l_y, cw, ch)
        c_fr_x, c_fr_y, _ = self.world_to_canvas(p_far_r_x, p_far_r_y, cw, ch)
        c_nr_x, c_nr_y, _ = self.world_to_canvas(p_near_r_x, p_near_r_y, cw, ch)
        c_nl_x, c_nl_y, _ = self.world_to_canvas(p_near_l_x, p_near_l_y, cw, ch)

        c.create_polygon(dp_x, dp_y, c_fl_x, c_fl_y, c_fr_x, c_fr_y, fill="#0d0407", outline="#25090f", width=1)
        c.create_polygon(c_nl_x, c_nl_y, c_fl_x, c_fl_y, c_fr_x, c_fr_y, c_nr_x, c_nr_y,
                         fill="#14060a", outline="#3b0f17", width=1)

        # Drone Quadcopter Frame (Neutral Silver Cross)
        arm_len = 8.0
        c.create_line(dp_x - arm_len, dp_y - arm_len, dp_x + arm_len, dp_y + arm_len, fill="#cccccc", width=1.5)
        c.create_line(dp_x - arm_len, dp_y + arm_len, dp_x + arm_len, dp_y - arm_len, fill="#cccccc", width=1.5)

        # Rotors (Dark Red Accents)
        for rx in [-arm_len, arm_len]:
            for ry in [-arm_len, arm_len]:
                c.create_oval(dp_x + rx - 2.5, dp_y + ry - 2.5, dp_x + rx + 2.5, dp_y + ry + 2.5,
                              fill=self.c_red, outline=self.c_red_bright)

        # Center Hub
        c.create_oval(dp_x - 2, dp_y - 2, dp_x + 2, dp_y + 2, fill="#f0f0f0", outline="")

        # Forward Heading Vector Arrow (Sharp Crimson Vector)
        hdg_len = 15.0
        hx = dp_x + hdg_len * math.sin(yaw_rad)
        hy = dp_y - hdg_len * math.cos(yaw_rad)
        c.create_line(dp_x, dp_y, hx, hy, fill=self.c_red_bright, width=2, arrow=tk.LAST)

        # 13. Cardinal Tactical Compass Rose (Top-Right)
        comp_x = cw - 22.0
        comp_y = 24.0
        c.create_polygon(comp_x, comp_y - 12, comp_x - 4, comp_y - 2, comp_x + 4, comp_y - 2,
                         fill=self.c_red_bright, outline=self.c_red_bright)
        c.create_polygon(comp_x, comp_y + 8, comp_x - 4, comp_y - 2, comp_x + 4, comp_y - 2,
                         fill="#222222", outline="#222222")
        c.create_oval(comp_x - 2, comp_y - 4, comp_x + 2, comp_y, fill="#000000", outline="#444444")
        c.create_text(comp_x, comp_y + 16, text="N", fill=self.c_text_high, font=("Consolas", 8, "bold"))

        # 14. Tactical Map Scale Bar (20m reference in bottom-left)
        bar_len_m = 20.0
        bar_px = bar_len_m * scale
        sb_x = 24.0
        sb_y = ch - 16.0
        c.create_line(sb_x, sb_y, sb_x + bar_px, sb_y, fill="#383838", width=1.5)
        c.create_line(sb_x, sb_y - 3, sb_x, sb_y + 3, fill="#383838", width=1.5)
        c.create_line(sb_x + bar_px / 2.0, sb_y - 2, sb_x + bar_px / 2.0, sb_y + 2, fill="#383838", width=1.0)
        c.create_line(sb_x + bar_px, sb_y - 3, sb_x + bar_px, sb_y + 3, fill="#383838", width=1.5)
        c.create_text(sb_x + bar_px / 2.0, sb_y - 8, text=f"{int(bar_len_m)} m", fill="#707070", font=("Consolas", 7))

        # 15. Zoom Level Indicator (Top-Left of Map)
        if abs(self.zoom_factor - 1.0) > 0.05:
            c.create_text(30.0, 14.0, text=f"ZOOM: {self.zoom_factor:.1f}x", fill="#707070", font=("Consolas", 7, "bold"))
