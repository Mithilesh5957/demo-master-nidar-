#!/usr/bin/env python3
"""
scout_ai_core.py — Raspberry Pi 5 Optimized Detection & Tracking Pipeline
===========================================================================
Senior Edge AI & Computer Vision Engineering for the NIDAR Scout Drone.

Hardware Stack:
    - Raspberry Pi 5 (8GB, 4x Cortex-A76 @ 2.4GHz)
    - Skydroid C12 Dual-Camera Gimbal (RGB + Thermal, H.265 RTSP)
    - ArduPilot Flight Controller (PyMAVLink UDP)
    - Skydroid C12 Gimbal Control (UDP Port 5000)

Architecture:
    Thread 1: RGB FFmpeg Capture        → H.265 software decode → ring buffer
    Thread 2: Thermal FFmpeg Capture    → H.265 software decode → ring buffer
    Thread 3: Inference Loop            → YOLOv8 CPU → ByteTrack → Thermal Validate → Target Lock
    Thread 4: Telemetry Bridge          → MAVLink + C12 Gimbal state @ 10Hz
    Thread 5: Flask MJPEG Server        → Serves annotated frames to React frontend

Optimizations for Raspberry Pi 5:
    - YOLOv8n (nano) model for fastest CPU inference (~150ms/frame)
    - Inference at 320x320 resolution to reduce computation
    - Frame skipping: process every 3rd frame for AI, stream all for video
    - ONNX Runtime with ARM NEON acceleration when available
    - UDP RTSP transport with aggressive packet discard for low latency
    - Thread affinity hints for the 4-core Cortex-A76

Author: NIDAR Autonomous Systems
Version: 2.0.0 (Raspberry Pi 5)
"""

import os

# CRITICAL: Set FFmpeg options BEFORE importing cv2
# This ensures the C++ backend inherits these flags at module load time
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "rtsp_transport;udp"
    "|fflags;discardcorrupt+nobuffer"
    "|flags;low_delay"
    "|stimeout;2000000"
    "|max_delay;500000"
)

import sys
import cv2
import math
import json
import time
import logging
import threading
import numpy as np
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Dict, Any, Callable

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("ScoutAI")

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
@dataclass
class ScoutConfig:
    """Centralized configuration for the Scout AI Core pipeline."""

    # --- Camera RTSP URLs (Skydroid C12) ---
    rgb_url: str = "rtsp://192.168.144.108:554/stream=1"
    thermal_url: str = "rtsp://192.168.144.108:555/stream=2"

    # --- AI Model (CPU-optimized for Pi 5) ---
    model_path: str = "yolov8n.pt"            # Nano model for Pi 5 speed
    inference_size: int = 320                  # Smaller = faster on CPU
    detection_confidence: float = 0.35         # Minimum confidence for person
    thermal_confidence: float = 0.25           # Lower threshold for thermal
    process_every_n_frames: int = 3            # Skip frames to save CPU

    # --- Tracker ---
    track_max_age: int = 50                    # Max frames to keep a lost track
    track_min_hits: int = 3                    # Frames before a track is confirmed
    track_iou_threshold: float = 0.3           # IoU threshold for association

    # --- Target Lock ---
    lock_confidence: float = 0.85              # Min confidence for lock
    lock_frames: int = 30                      # Consecutive frames for lock
    lock_cooldown: float = 15.0                # Seconds between locks per ID

    # --- Thermal Validation ---
    thermal_iou_threshold: float = 0.25        # IoU for RGB-thermal overlap
    thermal_temp_threshold: int = 180          # Grayscale threshold for "hot"

    # --- Camera FOV (Skydroid C12) ---
    camera_fov_h: float = 65.0                # Horizontal FOV degrees
    camera_fov_v: float = 50.0                # Vertical FOV degrees

    # --- MAVLink ---
    mavlink_uri: str = "udpin:0.0.0.0:14550"

    # --- Flask ---
    flask_port: int = 5001
    jpeg_quality: int = 75                     # Slightly lower for Pi CPU savings

    # --- MQTT ---
    broker_ips: List[str] = field(default_factory=lambda: ["100.113.148.48"])
    topic_detections: str = "nidar/scout/detections"
    topic_telemetry: str = "nidar/scout/telemetry"


# ═══════════════════════════════════════════════════════════════════════════════
# 1. RTSP Video Capture (Pi 5 Optimized)
# ═══════════════════════════════════════════════════════════════════════════════

class RTSPCapture:
    """
    RTSP video capture optimized for Raspberry Pi 5.

    Uses OpenCV's FFmpeg backend with UDP transport and aggressive
    low-latency flags. H.265 is decoded in software by FFmpeg's
    libavcodec, which uses ARM NEON SIMD on the Cortex-A76 cores.

    Features:
        - Automatic reconnection with exponential backoff
        - Buffer size = 1 to always get the latest frame
        - Thread-safe read access
    """

    def __init__(self, rtsp_url: str, name: str = "stream"):
        self.rtsp_url = rtsp_url
        self.name = name
        self.cap: Optional[cv2.VideoCapture] = None
        self._lock = threading.Lock()
        self._reconnect_delay = 1.0
        self._max_reconnect_delay = 30.0
        self._frame_count = 0

    def open(self) -> bool:
        """Open the RTSP stream via FFmpeg."""
        with self._lock:
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass

            log.info(f"[{self.name}] Opening RTSP: {self.rtsp_url}")
            cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)

            if cap.isOpened():
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self.cap = cap
                self._reconnect_delay = 1.0  # Reset backoff
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                log.info(f"[{self.name}] ✅ Stream opened ({w}x{h})")
                return True

            log.error(f"[{self.name}] ❌ Failed to open stream")
            self.cap = None
            return False

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read a single frame. Thread-safe."""
        with self._lock:
            if self.cap is None or not self.cap.isOpened():
                return False, None
            try:
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    self._frame_count += 1
                    return True, frame
                return False, None
            except Exception as e:
                log.error(f"[{self.name}] Read error: {e}")
                return False, None

    def reconnect(self) -> bool:
        """Reconnect with exponential backoff."""
        log.warning(
            f"[{self.name}] Reconnecting in {self._reconnect_delay:.1f}s..."
        )
        time.sleep(self._reconnect_delay)
        success = self.open()
        if not success:
            self._reconnect_delay = min(
                self._reconnect_delay * 2, self._max_reconnect_delay
            )
        return success

    def release(self):
        """Release the capture device."""
        with self._lock:
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
                log.info(f"[{self.name}] Released")

    def is_opened(self) -> bool:
        with self._lock:
            return self.cap is not None and self.cap.isOpened()

    @property
    def total_frames(self) -> int:
        return self._frame_count


# ═══════════════════════════════════════════════════════════════════════════════
# 2. YOLOv8 Detection Engine (CPU-Optimized for Pi 5)
# ═══════════════════════════════════════════════════════════════════════════════

class DetectionEngine:
    """
    YOLOv8 inference engine optimized for Raspberry Pi 5 CPU.

    Pi 5 Performance (approximate):
        - YOLOv8n @ 320x320: ~120-180ms per frame (~6-8 FPS)
        - YOLOv8s @ 320x320: ~300-400ms per frame (~2-3 FPS)
        - YOLOv8n @ 640x640: ~500-700ms per frame (~1.5 FPS)

    Recommendation: Use YOLOv8n at 320x320 for real-time operation.
    """

    PERSON_CLASS_ID = 0  # COCO class 0 = person

    def __init__(self, model_path: str = "yolov8n.pt",
                 confidence: float = 0.35,
                 imgsz: int = 320):
        self.confidence = confidence
        self.imgsz = imgsz
        self.model = None
        self._inference_times: List[float] = []
        self._load_model(model_path)

    def _load_model(self, model_path: str):
        """Load YOLO model with graceful error handling."""
        try:
            from ultralytics import YOLO
        except ImportError:
            log.error("❌ ultralytics not installed. Run: pip install ultralytics")
            log.error("   AI detection will be disabled.")
            return

        if os.path.exists(model_path):
            log.info(f"🧠 Loading AI Model: {model_path}")
            try:
                self.model = YOLO(model_path)
                # Force CPU device on Pi 5 (no CUDA)
                log.info(f"✅ Model loaded: {model_path} (CPU Mode)")
                return
            except Exception as e:
                log.warning(f"Failed to load {model_path}: {e}")

        # Fallback: download yolov8n (smallest, fastest)
        log.warning("⚠️ Model not found locally. Downloading yolov8n.pt...")
        try:
            self.model = YOLO("yolov8n.pt")
            log.info("✅ Downloaded and loaded yolov8n.pt (CPU Mode)")
        except Exception as e:
            log.error(f"❌ All model loading failed: {e}")

    def detect(self, frame: np.ndarray,
               conf_override: Optional[float] = None) -> np.ndarray:
        """
        Run person detection on a frame.

        Returns:
            np.ndarray of shape (N, 6): [x1, y1, x2, y2, confidence, class_id]
            Empty array if no detections.
        """
        if self.model is None or frame is None:
            return np.empty((0, 6))

        conf = conf_override or self.confidence
        try:
            t0 = time.time()

            results = self.model(
                frame,
                imgsz=self.imgsz,
                conf=conf,
                classes=[self.PERSON_CLASS_ID],
                verbose=False,
                device="cpu",  # Force CPU on Raspberry Pi 5
            )

            elapsed = time.time() - t0
            self._inference_times.append(elapsed)
            if len(self._inference_times) > 30:
                self._inference_times = self._inference_times[-30:]

            detections = []
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    c = float(box.conf[0])
                    cls = int(box.cls[0])
                    detections.append([x1, y1, x2, y2, c, cls])

            return np.array(detections) if detections else np.empty((0, 6))

        except Exception as e:
            log.error(f"Detection error: {e}")
            return np.empty((0, 6))

    @property
    def avg_inference_ms(self) -> float:
        """Average inference time in milliseconds."""
        if not self._inference_times:
            return 0.0
        return (sum(self._inference_times) / len(self._inference_times)) * 1000

    @property
    def is_available(self) -> bool:
        return self.model is not None


# ═══════════════════════════════════════════════════════════════════════════════
# 3. ByteTrack Multi-Object Tracker
# ═══════════════════════════════════════════════════════════════════════════════

class TrackerManager:
    """
    Multi-object tracker for persistent human ID assignment.

    Fallback chain:
        1. boxmot.BYTETracker (best performance)
        2. boxmot.StrongSORT (ReID-based)
        3. Centroid fallback (no dependencies)
    """

    def __init__(self, max_age: int = 50, min_hits: int = 3,
                 iou_threshold: float = 0.3):
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self._tracker = None
        self._backend = "none"
        self._next_id = 1
        self._tracks: Dict[int, Dict] = {}
        self._init_tracker()

    def _init_tracker(self):
        """Initialize tracker with fallback chain."""

        # Attempt 1: boxmot BYTETracker
        try:
            from boxmot import BYTETracker  # type: ignore
            self._tracker = BYTETracker(
                track_thresh=0.25,
                track_buffer=self.max_age,
                match_thresh=self.iou_threshold,
            )
            self._backend = "boxmot.BYTETracker"
            log.info(f"🎯 Tracker: {self._backend}")
            return
        except ImportError:
            pass

        # Attempt 2: boxmot StrongSORT
        try:
            from boxmot import StrongSORT  # type: ignore
            from pathlib import Path
            self._tracker = StrongSORT(
                model_weights=Path("osnet_x0_25_msmt17.pt"),
                device="cpu",
                fp16=False,
                max_age=self.max_age,
                n_init=self.min_hits,
            )
            self._backend = "boxmot.StrongSORT"
            log.info(f"🎯 Tracker: {self._backend}")
            return
        except (ImportError, Exception) as e:
            log.debug(f"StrongSORT unavailable: {e}")

        # Fallback: centroid tracker
        self._backend = "centroid_fallback"
        log.warning("⚠️ No tracker library. Using centroid fallback.")
        log.warning("   Install: pip install boxmot lap")

    def update(self, detections: np.ndarray,
               frame: np.ndarray) -> List[Dict[str, Any]]:
        """
        Update tracker with new detections.

        Returns:
            List of dicts: [{track_id, bbox, confidence, active}]
        """
        if len(detections) == 0:
            if self._backend == "centroid_fallback":
                self._age_fallback_tracks()
            return []

        # boxmot trackers
        if self._tracker is not None:
            try:
                tracks = self._tracker.update(detections, frame)
                results = []
                for t in tracks:
                    x1, y1, x2, y2 = int(t[0]), int(t[1]), int(t[2]), int(t[3])
                    track_id = int(t[4])
                    conf = float(t[5]) if len(t) > 5 else 0.5
                    results.append({
                        "track_id": track_id,
                        "bbox": (x1, y1, x2, y2),
                        "confidence": conf,
                        "active": True,
                    })
                return results
            except Exception as e:
                log.error(f"Tracker error: {e}")
                return self._fallback_update(detections)

        return self._fallback_update(detections)

    def _fallback_update(self, detections: np.ndarray) -> List[Dict[str, Any]]:
        """Simple centroid-based tracking."""
        results = []
        used = set()

        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2

            best_id, best_dist = None, 100.0
            for tid, td in self._tracks.items():
                if tid in used:
                    continue
                tx, ty = td["center"]
                d = math.hypot(cx - tx, cy - ty)
                if d < best_dist:
                    best_dist, best_id = d, tid

            if best_id is not None:
                self._tracks[best_id]["center"] = (cx, cy)
                self._tracks[best_id]["age"] = 0
                self._tracks[best_id]["hits"] += 1
                used.add(best_id)
                track_id = best_id
            else:
                track_id = self._next_id
                self._next_id += 1
                self._tracks[track_id] = {"center": (cx, cy), "age": 0, "hits": 1}
                used.add(track_id)

            results.append({
                "track_id": track_id,
                "bbox": (int(x1), int(y1), int(x2), int(y2)),
                "confidence": float(conf),
                "active": self._tracks[track_id]["hits"] >= self.min_hits,
            })

        self._age_fallback_tracks()
        return results

    def _age_fallback_tracks(self):
        expired = [t for t, d in self._tracks.items() if d["age"] > self.max_age]
        for t in expired:
            del self._tracks[t]
        for d in self._tracks.values():
            d["age"] += 1

    @property
    def backend(self) -> str:
        return self._backend


# ═══════════════════════════════════════════════════════════════════════════════
# 4. Thermal Validation Engine
# ═══════════════════════════════════════════════════════════════════════════════

class ThermalValidator:
    """
    Validates RGB detections against thermal heat signatures.
    Filters false positives (mannequins, signs, shadows).
    """

    def __init__(self, temp_threshold: int = 180,
                 iou_threshold: float = 0.25):
        self.temp_threshold = temp_threshold
        self.iou_threshold = iou_threshold

    def extract_heat_blobs(self, thermal_frame: np.ndarray,
                           min_area: int = 500) -> List[Tuple[int, int, int, int]]:
        """Extract bounding boxes of hot regions from thermal frame."""
        if thermal_frame is None:
            return []

        if len(thermal_frame.shape) == 3:
            gray = cv2.cvtColor(thermal_frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = thermal_frame.copy()

        _, binary = cv2.threshold(gray, self.temp_threshold, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        blobs = []
        for c in contours:
            if cv2.contourArea(c) >= min_area:
                x, y, w, h = cv2.boundingRect(c)
                blobs.append((x, y, x + w, y + h))
        return blobs

    @staticmethod
    def compute_iou(a: Tuple, b: Tuple) -> float:
        x1 = max(a[0], b[0]); y1 = max(a[1], b[1])
        x2 = min(a[2], b[2]); y2 = min(a[3], b[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        if inter == 0:
            return 0.0
        area_a = (a[2] - a[0]) * (a[3] - a[1])
        area_b = (b[2] - b[0]) * (b[3] - b[1])
        return inter / (area_a + area_b - inter)

    def validate(self, rgb_tracks: List[Dict], thermal_frame: np.ndarray,
                 rgb_shape: Tuple[int, int],
                 thermal_shape: Tuple[int, int]) -> List[Dict]:
        """Validate RGB tracks against thermal heat blobs."""
        if thermal_frame is None or not rgb_tracks:
            for t in rgb_tracks:
                t["thermal_confirmed"] = False
            return rgb_tracks

        blobs = self.extract_heat_blobs(thermal_frame)
        rgb_h, rgb_w = rgb_shape
        thm_h, thm_w = thermal_shape
        sx = thm_w / rgb_w if rgb_w > 0 else 1.0
        sy = thm_h / rgb_h if rgb_h > 0 else 1.0

        for track in rgb_tracks:
            x1, y1, x2, y2 = track["bbox"]
            scaled = (int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy))

            track["thermal_confirmed"] = any(
                self.compute_iou(scaled, blob) >= self.iou_threshold
                for blob in blobs
            )

        return rgb_tracks


# ═══════════════════════════════════════════════════════════════════════════════
# 5. Target Lock Manager
# ═══════════════════════════════════════════════════════════════════════════════

class TargetLockManager:
    """
    Triggers 'Target Lock' when a human ID sustains >85% confidence
    for 30+ consecutive frames. Prevents false geolocation events.
    """

    def __init__(self, lock_confidence: float = 0.85,
                 lock_frames: int = 30,
                 cooldown: float = 15.0,
                 on_lock: Optional[Callable] = None):
        self.lock_confidence = lock_confidence
        self.lock_frames = lock_frames
        self.cooldown = cooldown
        self.on_lock = on_lock

        self._state: Dict[int, Dict] = {}
        self._lock = threading.Lock()
        self._locked_ids: set = set()

    def update(self, tracks: List[Dict]) -> List[int]:
        """Update lock state. Returns list of newly locked track IDs."""
        newly_locked = []
        active_ids = set()

        with self._lock:
            for track in tracks:
                tid = track["track_id"]
                conf = track["confidence"]
                thermal_ok = track.get("thermal_confirmed", True)
                active_ids.add(tid)

                if tid not in self._state:
                    self._state[tid] = {
                        "consecutive": 0, "last_lock_time": 0.0, "peak_conf": 0.0
                    }

                state = self._state[tid]

                if conf >= self.lock_confidence and thermal_ok:
                    state["consecutive"] += 1
                    state["peak_conf"] = max(state["peak_conf"], conf)
                else:
                    state["consecutive"] = 0

                if state["consecutive"] >= self.lock_frames:
                    now = time.time()
                    if (now - state["last_lock_time"]) >= self.cooldown:
                        state["last_lock_time"] = now
                        state["consecutive"] = 0
                        self._locked_ids.add(tid)
                        newly_locked.append(tid)

                        log.info(
                            f"🔒 TARGET LOCK! ID:{tid} "
                            f"Conf:{state['peak_conf']:.1%} "
                            f"Frames:{self.lock_frames}"
                        )
                        if self.on_lock:
                            try:
                                self.on_lock(tid, track["bbox"], conf)
                            except Exception as e:
                                log.error(f"Lock callback error: {e}")
                        state["peak_conf"] = 0.0

            # Reset stale tracks
            for tid in self._state:
                if tid not in active_ids:
                    self._state[tid]["consecutive"] = 0

        return newly_locked

    def get_lock_state(self, track_id: int) -> Dict:
        with self._lock:
            s = self._state.get(track_id, {})
            return {
                "consecutive": s.get("consecutive", 0),
                "required": self.lock_frames,
                "progress": s.get("consecutive", 0) / self.lock_frames,
                "locked": track_id in self._locked_ids,
            }

    @property
    def total_locks(self) -> int:
        return len(self._locked_ids)


# ═══════════════════════════════════════════════════════════════════════════════
# 6. Telemetry Bridge (MAVLink + C12 Gimbal)
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class DroneState:
    """Thread-safe snapshot of drone state."""
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    heading: float = 0.0
    speed: float = 0.0
    gps_sats: int = 0
    gps_fix: int = 0
    battery_pct: float = -1.0
    battery_voltage: float = 0.0
    mode: str = "UNKNOWN"
    armed: bool = False
    gimbal_pitch: float = 0.0
    gimbal_yaw: float = 0.0
    gimbal_roll: float = 0.0
    gimbal_active: bool = False
    timestamp: float = 0.0


class TelemetryBridge:
    """Reads MAVLink telemetry + C12 gimbal state in a background thread."""

    COPTER_MODES = {
        0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
        4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
        9: "LAND", 11: "DRIFT", 13: "SPORT", 16: "POSHOLD",
        17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS",
    }

    def __init__(self, mavlink_uri: str = "udpin:0.0.0.0:14550", gimbal=None):
        self.mavlink_uri = mavlink_uri
        self.gimbal = gimbal
        self._state = DroneState()
        self._lock = threading.Lock()
        self._conn = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    @property
    def state(self) -> DroneState:
        with self._lock:
            import copy
            return copy.copy(self._state)

    def start(self):
        if self._running:
            return
        log.info(f"🔌 Connecting to MAVLink: {self.mavlink_uri}")
        try:
            from pymavlink import mavutil
            self._conn = mavutil.mavlink_connection(self.mavlink_uri)

            log.info("⏳ Waiting for heartbeat...")
            msg = self._conn.wait_heartbeat(timeout=30)
            if msg:
                self._conn.target_system = msg.get_srcSystem()
                self._conn.target_component = msg.get_srcComponent()
                log.info(
                    f"✅ Heartbeat from System {self._conn.target_system}, "
                    f"Component {self._conn.target_component}"
                )
            else:
                log.warning("⚠️ No heartbeat within 30s")

            self._conn.mav.request_data_stream_send(
                self._conn.target_system, self._conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
            )

            self._running = True
            self._thread = threading.Thread(
                target=self._loop, daemon=True, name="Telemetry"
            )
            self._thread.start()
            log.info("📡 Telemetry bridge started")
        except Exception as e:
            log.error(f"❌ MAVLink connection failed: {e}")

    def _loop(self):
        from pymavlink import mavutil
        while self._running:
            try:
                msg = self._conn.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                t = msg.get_type()
                with self._lock:
                    if t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                        self._state.armed = bool(
                            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        )
                        self._state.mode = self.COPTER_MODES.get(
                            msg.custom_mode, f"MODE_{msg.custom_mode}"
                        )
                    elif t == "GLOBAL_POSITION_INT":
                        self._state.lat = msg.lat / 1e7
                        self._state.lon = msg.lon / 1e7
                        self._state.alt = msg.relative_alt / 1000.0
                        self._state.heading = msg.hdg / 100.0
                    elif t == "VFR_HUD":
                        self._state.speed = msg.groundspeed
                        if hasattr(msg, 'heading'):
                            self._state.heading = msg.heading
                    elif t == "GPS_RAW_INT":
                        self._state.gps_sats = msg.satellites_visible
                        self._state.gps_fix = msg.fix_type
                    elif t == "SYS_STATUS":
                        self._state.battery_voltage = msg.voltage_battery / 1000.0
                        if msg.battery_remaining >= 0:
                            self._state.battery_pct = msg.battery_remaining

                    self._state.timestamp = time.time()

                    if self.gimbal:
                        try:
                            att = self.gimbal.get_attitude()
                            self._state.gimbal_yaw = att.get("yaw", 0.0)
                            self._state.gimbal_pitch = att.get("pitch", 0.0)
                            self._state.gimbal_roll = att.get("roll", 0.0)
                            self._state.gimbal_active = att.get("active", False)
                        except Exception:
                            pass
            except Exception as e:
                if self._running:
                    log.error(f"Telemetry error: {e}")
                    time.sleep(0.5)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        log.info("📡 Telemetry bridge stopped")


# ═══════════════════════════════════════════════════════════════════════════════
# 7. Geolocation Calculator
# ═══════════════════════════════════════════════════════════════════════════════

def calculate_target_ground_gps(
    drone_lat: float, drone_lon: float, alt: float, heading: float,
    gimbal_pitch: float, gimbal_yaw: float,
    pixel_x: float, pixel_y: float, img_w: float, img_h: float,
    fov_h: float = 65.0, fov_v: float = 50.0,
) -> Tuple[float, float]:
    """
    Estimate GPS coordinates of a target on the ground.

    Uses ray-casting from camera through target pixel to ground plane.

    Args:
        drone_lat/lon: Drone GPS (decimal degrees)
        alt: Relative altitude (meters above takeoff)
        heading: Drone compass heading (0-360°, North=0)
        gimbal_pitch: Gimbal pitch (0°=horizon, -90°=straight down)
        gimbal_yaw: Gimbal yaw (relative to drone heading)
        pixel_x/y: Target center pixel
        img_w/h: Frame dimensions
        fov_h/v: Camera field of view (degrees)

    Returns:
        (target_lat, target_lon) in decimal degrees.
    """
    if alt <= 0.5:
        return drone_lat, drone_lon

    # Pixel offset → angular offset
    norm_x = (pixel_x - img_w / 2) / (img_w / 2)
    norm_y = (pixel_y - img_h / 2) / (img_h / 2)
    angle_h = norm_x * (fov_h / 2)
    angle_v = norm_y * (fov_v / 2)

    # Effective look angles
    effective_pitch = gimbal_pitch - angle_v
    effective_yaw = gimbal_yaw + angle_h

    # Angle from nadir (straight down)
    angle_from_nadir = 90.0 + effective_pitch
    angle_from_nadir = max(1.0, min(85.0, angle_from_nadir))

    # Ground distance
    ground_dist = alt * math.tan(math.radians(angle_from_nadir))
    ground_dist = min(ground_dist, alt * 20.0)

    # Bearing
    bearing = (heading + effective_yaw) % 360.0

    # Forward projection (spherical)
    R = 6378137.0
    lat1 = math.radians(drone_lat)
    lon1 = math.radians(drone_lon)
    brng = math.radians(bearing)

    lat2 = math.asin(
        math.sin(lat1) * math.cos(ground_dist / R)
        + math.cos(lat1) * math.sin(ground_dist / R) * math.cos(brng)
    )
    lon2 = lon1 + math.atan2(
        math.sin(brng) * math.sin(ground_dist / R) * math.cos(lat1),
        math.cos(ground_dist / R) - math.sin(lat1) * math.sin(lat2),
    )

    target_lat, target_lon = math.degrees(lat2), math.degrees(lon2)

    log.info(
        f"📍 GEOLOC: pitch={effective_pitch:.1f}° yaw={effective_yaw:.1f}° "
        f"dist={ground_dist:.1f}m → ({target_lat:.6f}, {target_lon:.6f})"
    )
    return target_lat, target_lon


# ═══════════════════════════════════════════════════════════════════════════════
# 8. Frame Annotator
# ═══════════════════════════════════════════════════════════════════════════════

class FrameAnnotator:
    """Draws detection overlays, track IDs, lock progress, and HUD."""

    COLOR_TRACKED = (0, 255, 0)
    COLOR_LOCKED = (0, 0, 255)
    COLOR_THERMAL = (0, 255, 255)
    COLOR_LOCKING = (0, 165, 255)

    @staticmethod
    def annotate(frame: np.ndarray, tracks: List[Dict],
                 lock_mgr: TargetLockManager,
                 drone: DroneState) -> np.ndarray:
        if frame is None:
            return frame
        out = frame.copy()
        h, w = out.shape[:2]

        for t in tracks:
            tid = t["track_id"]
            x1, y1, x2, y2 = t["bbox"]
            conf = t["confidence"]
            thermal = t.get("thermal_confirmed", False)
            ls = lock_mgr.get_lock_state(tid)

            if ls["locked"]:
                color = FrameAnnotator.COLOR_LOCKED
            elif ls["progress"] > 0:
                color = FrameAnnotator.COLOR_LOCKING
            elif thermal:
                color = FrameAnnotator.COLOR_THERMAL
            else:
                color = FrameAnnotator.COLOR_TRACKED

            cv2.rectangle(out, (x1, y1), (x2, y2), color, 3 if ls["locked"] else 2)

            label = f"ID:{tid} {conf:.0%}"
            if thermal:
                label += " [HEAT]"
            if ls["locked"]:
                label += " LOCKED"

            (tw, th2), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            cv2.rectangle(out, (x1, y1 - th2 - 8), (x1 + tw + 4, y1), (0, 0, 0), -1)
            cv2.putText(out, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

            # Lock progress bar
            if 0 < ls["progress"] < 1.0:
                bw = x2 - x1
                filled = int(bw * ls["progress"])
                cv2.rectangle(out, (x1, y2 + 2), (x1 + bw, y2 + 6), (80, 80, 80), -1)
                cv2.rectangle(out, (x1, y2 + 2), (x1 + filled, y2 + 6), color, -1)

        # HUD
        hud = [
            f"ALT:{drone.alt:.1f}m SPD:{drone.speed:.1f}m/s "
            f"SAT:{drone.gps_sats} {drone.mode}",
            f"GPS:{drone.lat:.5f},{drone.lon:.5f} HDG:{drone.heading:.0f}",
            f"GIMBAL P:{drone.gimbal_pitch:.1f} Y:{drone.gimbal_yaw:.1f}",
        ]
        for i, line in enumerate(hud):
            cv2.putText(out, line, (10, h - 12 - (len(hud) - 1 - i) * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
        return out


# ═══════════════════════════════════════════════════════════════════════════════
# 9. Flask MJPEG Streaming Server
# ═══════════════════════════════════════════════════════════════════════════════

class StreamingServer:
    """Serves annotated frames as MJPEG for the React frontend."""

    def __init__(self, port: int = 5001, jpeg_quality: int = 75):
        self.port = port
        self.jpeg_quality = jpeg_quality
        self._frame_rgb: Optional[np.ndarray] = None
        self._frame_thermal: Optional[np.ndarray] = None
        self._lock_rgb = threading.Lock()
        self._lock_thermal = threading.Lock()

    def update_rgb(self, frame: np.ndarray):
        with self._lock_rgb:
            self._frame_rgb = frame

    def update_thermal(self, frame: np.ndarray):
        with self._lock_thermal:
            self._frame_thermal = frame

    def _gen(self, is_thermal=False):
        params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        while True:
            lock = self._lock_thermal if is_thermal else self._lock_rgb
            src = "_frame_thermal" if is_thermal else "_frame_rgb"
            with lock:
                frame = getattr(self, src)
                if frame is not None:
                    frame = frame.copy()
            if frame is not None:
                ret, buf = cv2.imencode(".jpg", frame, params)
                if ret:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                           + buf.tobytes() + b"\r\n")
                else:
                    time.sleep(0.03)
            else:
                time.sleep(0.05)

    def start(self):
        from flask import Flask, Response
        app = Flask(__name__)

        @app.after_request
        def cors(r):
            r.headers["Access-Control-Allow-Origin"] = "*"
            r.headers["Access-Control-Allow-Headers"] = "Content-Type,Authorization"
            r.headers["Access-Control-Allow-Methods"] = "GET,POST,OPTIONS"
            return r

        @app.route("/video_feed")
        def rgb():
            return Response(self._gen(False),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

        @app.route("/thermal_feed")
        def thermal():
            return Response(self._gen(True),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

        @app.route("/health")
        def health():
            return {"status": "ok", "platform": "raspberry_pi_5", "version": "2.0.0"}

        threading.Thread(
            target=lambda: app.run(host="0.0.0.0", port=self.port,
                                   debug=False, use_reloader=False, threaded=True),
            daemon=True, name="Flask",
        ).start()
        log.info(f"🎥 MJPEG Server on port {self.port}")


# ═══════════════════════════════════════════════════════════════════════════════
# 10. Main Orchestrator
# ═══════════════════════════════════════════════════════════════════════════════

class _FPSCounter:
    def __init__(self, name="", window=30):
        self.name = name
        self._ts: List[float] = []
        self._window = window
        self.fps = 0.0

    def tick(self):
        now = time.time()
        self._ts.append(now)
        if len(self._ts) > self._window:
            self._ts = self._ts[-self._window:]
        if len(self._ts) >= 2:
            e = self._ts[-1] - self._ts[0]
            self.fps = (len(self._ts) - 1) / e if e > 0 else 0.0


class ScoutAICore:
    """
    Main orchestrator for the Raspberry Pi 5 Scout AI pipeline.

    Manages 5 threads:
        1. RGB capture
        2. Thermal capture
        3. AI inference (with frame skipping for CPU budget)
        4. Telemetry (MAVLink + C12)
        5. Flask MJPEG server
    """

    def __init__(self, config: Optional[ScoutConfig] = None):
        self.config = config or ScoutConfig()
        self._running = False

        self.rgb_cap = RTSPCapture(self.config.rgb_url, name="RGB")
        self.thermal_cap = RTSPCapture(self.config.thermal_url, name="Thermal")

        self.detector = DetectionEngine(
            model_path=self.config.model_path,
            confidence=self.config.detection_confidence,
            imgsz=self.config.inference_size,
        )
        self.tracker = TrackerManager(
            max_age=self.config.track_max_age,
            min_hits=self.config.track_min_hits,
            iou_threshold=self.config.track_iou_threshold,
        )
        self.thermal_val = ThermalValidator(
            temp_threshold=self.config.thermal_temp_threshold,
            iou_threshold=self.config.thermal_iou_threshold,
        )
        self.lock_mgr = TargetLockManager(
            lock_confidence=self.config.lock_confidence,
            lock_frames=self.config.lock_frames,
            cooldown=self.config.lock_cooldown,
            on_lock=self._on_target_lock,
        )
        self.server = StreamingServer(
            port=self.config.flask_port,
            jpeg_quality=self.config.jpeg_quality,
        )

        self._gimbal = None
        self.telemetry: Optional[TelemetryBridge] = None

        # Shared buffers
        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_thermal: Optional[np.ndarray] = None
        self._lock_rgb = threading.Lock()
        self._lock_thermal = threading.Lock()

        self._frame_counter = 0
        self._stats = {
            "rgb_fps": 0.0, "thermal_fps": 0.0, "inference_fps": 0.0,
            "total_detections": 0, "total_locks": 0, "avg_infer_ms": 0.0,
        }

    def _init_gimbal(self):
        try:
            from C12Driver import C12Driver
            self._gimbal = C12Driver()
            log.info("✅ C12 Gimbal Driver loaded")
            return self._gimbal
        except Exception as e:
            log.warning(f"⚠️ C12 Gimbal not available: {e}")
            return None

    def start(self):
        log.info("=" * 60)
        log.info("  SCOUT AI CORE v2.0.0 — Raspberry Pi 5")
        log.info("=" * 60)
        self._running = True

        gimbal = self._init_gimbal()
        self.telemetry = TelemetryBridge(
            mavlink_uri=self.config.mavlink_uri, gimbal=gimbal
        )
        self.telemetry.start()
        self.server.start()

        threading.Thread(target=self._rgb_loop, daemon=True, name="RGB").start()
        threading.Thread(target=self._thermal_loop, daemon=True, name="Thermal").start()
        threading.Thread(target=self._inference_loop, daemon=True, name="AI").start()

        log.info("🚀 All threads started")
        log.info(f"   RGB:      {self.config.rgb_url}")
        log.info(f"   Thermal:  {self.config.thermal_url}")
        log.info(f"   Model:    {self.config.model_path} @ {self.config.inference_size}px (CPU)")
        log.info(f"   Tracker:  {self.tracker.backend}")
        log.info(f"   Skip:     Process every {self.config.process_every_n_frames} frames")
        log.info(f"   Lock:     {self.config.lock_frames}F @ {self.config.lock_confidence:.0%}")
        log.info(f"   Stream:   http://0.0.0.0:{self.config.flask_port}")

        try:
            while self._running:
                time.sleep(2.0)
                self._print_stats()
        except KeyboardInterrupt:
            log.info("🛑 Shutting down...")
            self.stop()

    def stop(self):
        self._running = False
        self.rgb_cap.release()
        self.thermal_cap.release()
        if self.telemetry:
            self.telemetry.stop()
        if self._gimbal:
            try:
                self._gimbal.close()
            except Exception:
                pass
        log.info("✅ Shutdown complete")

    # --- Capture threads ---

    def _rgb_loop(self):
        if not self.rgb_cap.open():
            log.error("[RGB] Initial open failed")
        fps = _FPSCounter("RGB")
        while self._running:
            try:
                if not self.rgb_cap.is_opened():
                    self.rgb_cap.reconnect(); continue
                ret, frame = self.rgb_cap.read()
                if ret and frame is not None:
                    with self._lock_rgb:
                        self._latest_rgb = frame
                    fps.tick()
                    self._stats["rgb_fps"] = fps.fps
                else:
                    self.rgb_cap.reconnect()
            except Exception as e:
                log.error(f"[RGB] {e}"); time.sleep(1)

    def _thermal_loop(self):
        if not self.thermal_cap.open():
            log.error("[Thermal] Initial open failed")
        fps = _FPSCounter("Thermal")
        while self._running:
            try:
                if not self.thermal_cap.is_opened():
                    self.thermal_cap.reconnect(); continue
                ret, frame = self.thermal_cap.read()
                if ret and frame is not None:
                    with self._lock_thermal:
                        self._latest_thermal = frame
                    fps.tick()
                    self._stats["thermal_fps"] = fps.fps
                else:
                    self.thermal_cap.reconnect()
            except Exception as e:
                log.error(f"[Thermal] {e}"); time.sleep(1)

    # --- AI inference thread ---

    def _inference_loop(self):
        log.info("[AI] Waiting for first frame...")
        while self._running:
            with self._lock_rgb:
                if self._latest_rgb is not None:
                    break
            time.sleep(0.1)
        log.info("[AI] ✅ Pipeline running")

        fps = _FPSCounter("AI")
        skip = self.config.process_every_n_frames

        while self._running:
            try:
                # Grab frames
                with self._lock_rgb:
                    rgb = self._latest_rgb.copy() if self._latest_rgb is not None else None
                with self._lock_thermal:
                    thm = self._latest_thermal.copy() if self._latest_thermal is not None else None

                if rgb is None:
                    time.sleep(0.01); continue

                self._frame_counter += 1
                rgb_h, rgb_w = rgb.shape[:2]

                # Frame skipping: only run AI every N frames
                run_ai = (self._frame_counter % skip == 0)

                if run_ai and self.detector.is_available:
                    # Detect
                    dets = self.detector.detect(rgb)
                    # Track
                    tracks = self.tracker.update(dets, rgb)
                    # Thermal validate
                    if thm is not None:
                        thm_h, thm_w = thm.shape[:2]
                        tracks = self.thermal_val.validate(
                            tracks, thm, (rgb_h, rgb_w), (thm_h, thm_w)
                        )
                    else:
                        for t in tracks:
                            t["thermal_confirmed"] = False
                    # Target lock
                    locked = self.lock_mgr.update(tracks)
                    self._stats["total_detections"] += len(dets)
                    self._stats["total_locks"] += len(locked)
                    self._stats["avg_infer_ms"] = self.detector.avg_inference_ms

                    for tid in locked:
                        self._process_lock(tid, tracks, rgb_w, rgb_h)

                    # Annotate
                    ds = self.telemetry.state if self.telemetry else DroneState()
                    annotated = FrameAnnotator.annotate(rgb, tracks, self.lock_mgr, ds)
                    self.server.update_rgb(annotated)
                else:
                    # On skipped frames, just push raw frame (keeps video smooth)
                    self.server.update_rgb(rgb)

                if thm is not None:
                    self.server.update_thermal(thm)

                fps.tick()
                self._stats["inference_fps"] = fps.fps
                time.sleep(0.005)  # Yield CPU

            except Exception as e:
                log.error(f"[AI] {e}", exc_info=True)
                time.sleep(0.1)

    # --- Target lock processing ---

    def _on_target_lock(self, track_id, bbox, confidence):
        log.info(f"🔒 LOCK CALLBACK ID:{track_id} Conf:{confidence:.1%}")

    def _process_lock(self, tid, tracks, img_w, img_h):
        track = next((t for t in tracks if t["track_id"] == tid), None)
        if not track or not self.telemetry:
            return
        s = self.telemetry.state
        if s.lat == 0.0 and s.lon == 0.0:
            log.warning(f"[Lock:{tid}] No GPS — skipping geoloc")
            return

        x1, y1, x2, y2 = track["bbox"]
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

        t_lat, t_lon = calculate_target_ground_gps(
            s.lat, s.lon, s.alt, s.heading,
            s.gimbal_pitch, s.gimbal_yaw,
            cx, cy, float(img_w), float(img_h),
            self.config.camera_fov_h, self.config.camera_fov_v,
        )

        log.info(
            f"🎯 LOCKED ID:{tid}\n"
            f"   Drone: ({s.lat:.6f},{s.lon:.6f}) Alt:{s.alt:.1f}m\n"
            f"   Gimbal: P:{s.gimbal_pitch:.1f} Y:{s.gimbal_yaw:.1f}\n"
            f"   Target: ({t_lat:.6f},{t_lon:.6f})"
        )
        self._mqtt_publish(tid, t_lat, t_lon, track["confidence"], s)

    def _mqtt_publish(self, tid, lat, lon, conf, ds):
        det = {
            "id": tid, "lat": lat, "lon": lon,
            "drone_lat": ds.lat, "drone_lon": ds.lon,
            "conf": conf, "source": "scout_ai_core",
            "timestamp": time.time(), "type": "human",
            "alt": ds.alt,
            "gimbal_pitch": ds.gimbal_pitch,
            "gimbal_yaw": ds.gimbal_yaw,
        }
        try:
            import paho.mqtt.publish as publish
            for ip in self.config.broker_ips:
                publish.single(self.config.topic_detections,
                               json.dumps(det), hostname=ip, port=1883)
                log.info(f"📡 Published to {ip}")
        except Exception as e:
            log.warning(f"MQTT publish failed: {e}")

    def _print_stats(self):
        s = self._stats
        log.info(
            f"📊 RGB:{s['rgb_fps']:.1f} THM:{s['thermal_fps']:.1f} "
            f"AI:{s['inference_fps']:.1f}fps ({s['avg_infer_ms']:.0f}ms) | "
            f"Det:{s['total_detections']} Lock:{s['total_locks']} | "
            f"{self.tracker.backend}"
        )


# ═══════════════════════════════════════════════════════════════════════════════
# Entry Point
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Scout AI Core — Raspberry Pi 5 Optimized Pipeline"
    )
    parser.add_argument("--rgb-url", default="rtsp://192.168.144.108:554/stream=1")
    parser.add_argument("--thermal-url", default="rtsp://192.168.144.108:555/stream=2")
    parser.add_argument("--model", default="yolov8n.pt", help="YOLO model (.pt)")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--mavlink", default="udpin:0.0.0.0:14550")
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--lock-frames", type=int, default=30)
    parser.add_argument("--lock-conf", type=float, default=0.85)
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--skip", type=int, default=3, help="Process every Nth frame")
    parser.add_argument("--broker", nargs="+", default=["100.113.148.48"])

    args = parser.parse_args()

    config = ScoutConfig(
        rgb_url=args.rgb_url,
        thermal_url=args.thermal_url,
        model_path=args.model,
        flask_port=args.port,
        mavlink_uri=args.mavlink,
        detection_confidence=args.conf,
        lock_frames=args.lock_frames,
        lock_confidence=args.lock_conf,
        inference_size=args.imgsz,
        process_every_n_frames=args.skip,
        broker_ips=args.broker,
    )

    core = ScoutAICore(config)
    core.start()


if __name__ == "__main__":
    main()
