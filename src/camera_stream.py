#!/usr/bin/env python3
"""
Camera Stream Bridge for PX4 Autonomous Survey Simulation.
Connects directly to PX4's high-speed video stream (RTP H.264 on port 5600 via stream.sdp)
with automatic fallback to gz-transport13 for maximum reliability.
"""

import os
import sys
import time
import threading
import numpy as np
import cv2

# Required for FFmpeg RTP protocol whitelist
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "protocol_whitelist;file,crypto,data,udp,rtp"
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"


class GazeboCameraStream:
    def __init__(self, topic: str = "/world/box_survey/model/x500_mono_cam_45_0/link/camera_link/sensor/camera/image",
                 sdp_file: str = "/home/devi/px4_drone_survey/stream.sdp"):
        self.primary_topic = topic
        self.sdp_file = sdp_file
        self.latest_frame = None
        self.frame_count = 0
        self.lock = threading.Lock()
        self.running = False
        self.fps = 0.0
        self.last_fps_time = time.time()
        self.fps_frame_count = 0
        self.subscribed_topics = set()
        self._first_frame_logged = False
        self.source_mode = "NONE"

        self.gz_node = None
        self.gz_failed = False
        self.worker = None

    def _udp_stream_worker(self):
        """Worker thread that continuously decodes frames from the RTP stream."""
        print(f"[GazeboCameraStream] Connecting to video stream via {self.sdp_file}...", flush=True)
        
        # Ensure SDP file exists
        if not os.path.exists(self.sdp_file):
            with open(self.sdp_file, "w") as f:
                f.write("c=IN IP4 127.0.0.1\nm=video 5600 RTP/AVP 96\na=rtpmap:96 H264/90000\n")

        cap = None
        fail_count = 0

        while self.running:
            if cap is None or not cap.isOpened():
                cap = cv2.VideoCapture(self.sdp_file, cv2.CAP_FFMPEG)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    self.subscribed_topics.add("udp://127.0.0.1:5600")
                    print("[GazeboCameraStream] -> Connected to PX4 H.264 video stream on UDP port 5600!", flush=True)
                else:
                    time.sleep(0.5)
                    continue

            ret, frame = cap.read()
            if not self.running:
                break
            if not ret or frame is None:
                fail_count += 1
                if fail_count > 50:
                    # Connection lost or stalled, force reopen
                    cap.release()
                    cap = None
                    fail_count = 0
                time.sleep(0.01)
                continue

            fail_count = 0
            with self.lock:
                self.latest_frame = frame
                self.frame_count += 1
                self.fps_frame_count += 1
                self.source_mode = "RTP_UDP_5600"

                if not self._first_frame_logged:
                    print(f"[GazeboCameraStream] -> First frame decoded: {frame.shape[1]}x{frame.shape[0]} at 45-deg pitch!", flush=True)
                    self._first_frame_logged = True

                now = time.time()
                elapsed = now - self.last_fps_time
                if elapsed >= 1.0:
                    self.fps = self.fps_frame_count / elapsed
                    self.fps_frame_count = 0
                    self.last_fps_time = now

        if cap is not None:
            cap.release()

    def _start_gz_fallback(self):
        """Fallback subscriber using gz.transport13."""
        try:
            from gz.transport13 import Node
            from gz.msgs10.image_pb2 import Image
            self.gz_node = Node()

            def cb(msg: Image):
                try:
                    w, h, fmt = msg.width, msg.height, msg.pixel_format_type
                    if fmt == 3:
                        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                    else:
                        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, -1))
                        bgr = cv2.cvtColor(arr[:, :, :3], cv2.COLOR_RGB2BGR)

                    with self.lock:
                        # Only take gz-transport frames if UDP hasn't taken over
                        if self.source_mode != "RTP_UDP_5600":
                            self.latest_frame = bgr
                            self.frame_count += 1
                            self.fps_frame_count += 1
                            self.source_mode = "GZ_TRANSPORT"
                            if not self._first_frame_logged:
                                print(f"[GazeboCameraStream] -> First gz-transport frame: {w}x{h}!", flush=True)
                                self._first_frame_logged = True
                            now = time.time()
                            elapsed = now - self.last_fps_time
                            if elapsed >= 1.0:
                                self.fps = self.fps_frame_count / elapsed
                                self.fps_frame_count = 0
                                self.last_fps_time = now
                except Exception:
                    pass

            self.gz_node.subscribe(Image, self.primary_topic, cb)
            self.gz_node.subscribe(Image, "/camera", cb)
            self.subscribed_topics.add(self.primary_topic)
        except Exception as e:
            self.gz_failed = True
            print(f"[GazeboCameraStream] gz-transport notice: {e}", flush=True)

    def start(self):
        self.running = True
        self._start_gz_fallback()
        self.worker = threading.Thread(target=self._udp_stream_worker, daemon=True)
        self.worker.start()
        return True

    def stop(self):
        self.running = False
        if self.worker is not None and self.worker.is_alive():
            if threading.current_thread() != self.worker:
                self.worker.join(timeout=2.0)

    def get_frame(self):
        with self.lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def get_stats(self):
        with self.lock:
            return {
                "frame_count": self.frame_count,
                "fps": self.fps,
                "has_frame": self.latest_frame is not None,
                "subscribed": list(self.subscribed_topics),
                "mode": self.source_mode
            }


if __name__ == "__main__":
    print("Testing GazeboCameraStream...")
    stream = GazeboCameraStream()
    stream.start()
    t_end = time.time() + 10.0
    while time.time() < t_end:
        stats = stream.get_stats()
        print(f"Frames: {stats['frame_count']} | FPS: {stats['fps']:.1f} | Mode: {stats['mode']}")
        time.sleep(1.0)
    stream.stop()
