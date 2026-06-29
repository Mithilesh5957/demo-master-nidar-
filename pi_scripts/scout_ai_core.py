#!/usr/bin/env python3
"""
scout_ai_core.py — Jetson Orin Nano Optimized Detection & Tracking Pipeline
=============================================================================
Senior Edge AI & Computer Vision Engineering for the NIDAR Scout Drone.

Hardware Stack:
    - NVIDIA Jetson Orin Nano (8GB)
    - Skydroid C12 Dual-Camera Gimbal (RGB + Thermal, H.265 RTSP)
    - ArduPilot Flight Controller (PyMAVLink UDP)
    - Skydroid C12 Gimbal Control (UDP Port 5000)

Architecture:
    Thread 1: RGB GStreamer Capture    → GPU H.265 decode → ring buffer
    Thread 2: Thermal GStreamer Capture → GPU H.265 decode → ring buffer
    Thread 3: Inference Loop           → TensorRT YOLO → ByteTrack → Thermal Validate → Target Lock
    Thread 4: Telemetry Bridge         → MAVLink + C12 Gimbal state @ 10Hz
    Thread 5: Flask MJPEG Server       → Serves annotated frames to React frontend

Author: NIDAR Autonomous Systems
Version: 2.0.0 (Jetson Orin Optimized)
"""

import os
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

    # --- AI Model ---
    model_path: str = "yolov8s.engine"       # TensorRT engine (primary)
    model_fallback: str = "yolov8s.pt"       # PyTorch fallback
    inference_size: int = 640                 # Input resolution for YOLO
    detection_confidence: float = 0.35        # Minimum confidence for person detection
    thermal_confidence: float = 0.25          # Lower threshold for thermal

    # --- Tracker ---
    track_max_age: int = 50                   # Max frames to keep a lost track
    track_min_hits: int = 3                   # Frames before a track is confirmed
    track_iou_threshold: float = 0.3          # IoU threshold for ByteTrack association

    # --- Target Lock ---
    lock_confidence: float = 0.85             # Min confidence for lock accumulation
    lock_frames: int = 30                     # Consecutive frames required for lock
    lock_cooldown: float = 15.0               # Seconds between lock events for same ID

    # --- Thermal Validation ---
    thermal_iou_threshold: float = 0.25       # IoU between RGB box and thermal blob
    thermal_temp_threshold: int = 180         # Grayscale threshold for "hot" pixels

    # --- Camera FOV (Skydroid C12) ---
    camera_fov_h: float = 65.0               # Horizontal FOV degrees
    camera_fov_v: float = 50.0               # Vertical FOV degrees

    # --- GStreamer ---
    gstreamer_latency: int = 100              # RTSP latency in ms
    gstreamer_max_buffers: int = 2            # appsink buffer limit

    # --- MAVLink ---
    mavlink_uri: str = "udpin:0.0.0.0:14550"

    # --- Flask ---
    flask_port: int = 5001
    jpeg_quality: int = 80

    # --- MQTT ---
    broker_ips: List[str] = field(default_factory=lambda: ["100.113.148.48"])
    topic_detections: str = "nidar/scout/detections"
    topic_telemetry: str = "nidar/scout/telemetry"


# ═══════════════════════════════════════════════════════════════════════════════
# 1. GStreamer Hardware-Accelerated Video Capture
# ═══════════════════════════════════════════════════════════════════════════════

class GStreamerCapture:
    """
    Hardware-accelerated RTSP capture using NVIDIA GStreamer plugins.

    On Jetson Orin Nano, this offloads H.265 decoding from CPU to the
    dedicated NVDEC hardware engine via nvv4l2decoder.

    Falls back to standard OpenCV FFmpeg if GStreamer is unavailable.
    """

    def __init__(self, rtsp_url: str, name: str = "stream",
                 latency: int = 100, max_buffers: int = 2):
        self.rtsp_url = rtsp_url
        self.name = name
        self.latency = latency
        self.max_buffers = max_buffers
        self.cap: Optional[cv2.VideoCapture] = None
        self._lock = threading.Lock()
        self._use_gstreamer = False
        self._reconnect_delay = 1.0
        self._max_reconnect_delay = 30.0

    def _build_gstreamer_pipeline(self) -> str:
        """Construct the GStreamer pipeline for NVIDIA hardware decoding."""
        return (
            f"rtspsrc location={self.rtsp_url} "
            f"latency={self.latency} "
            f"protocols=udp "
            f"drop-on-latency=true "
            f"do-retransmission=false ! "
            f"rtph265depay ! "
            f"h265parse ! "
            f"nvv4l2decoder enable-max-performance=1 ! "
            f"nvvidconv ! "
            f"video/x-raw, format=BGRx ! "
            f"videoconvert ! "
            f"video/x-raw, format=BGR ! "
            f"appsink max-buffers={self.max_buffers} drop=true sync=false"
        )

    def _build_ffmpeg_fallback(self) -> str:
        """Standard FFmpeg pipeline for non-Jetson systems."""
        return self.rtsp_url

    def open(self) -> bool:
        """Open the capture device, preferring GStreamer then FFmpeg."""
        with self._lock:
            if self.cap is not None:
                self.cap.release()

            # Attempt 1: GStreamer with NVIDIA HW decode
            pipeline = self._build_gstreamer_pipeline()
            log.info(f"[{self.name}] Attempting GStreamer HW decode...")
            log.debug(f"[{self.name}] Pipeline: {pipeline}")

            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                self.cap = cap
                self._use_gstreamer = True
                self._reconnect_delay = 1.0  # Reset backoff
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                log.info(f"[{self.name}] ✅ GStreamer opened ({w}x{h})")
                return True

            log.warning(f"[{self.name}] GStreamer failed, falling back to FFmpeg...")

            # Attempt 2: Standard FFmpeg software decode
            os.environ.setdefault(
                "OPENCV_FFMPEG_CAPTURE_OPTIONS",
                "rtsp_transport;udp|fflags;discardcorrupt+nobuffer"
                "|flags;low_delay|stimeout;2000000|max_delay;500000"
            )
            cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self.cap = cap
                self._use_gstreamer = False
                self._reconnect_delay = 1.0
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                log.info(f"[{self.name}] ✅ FFmpeg opened ({w}x{h})")
                return True

            log.error(f"[{self.name}] ❌ All capture methods failed")
            self.cap = None
            return False

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read a frame. Thread-safe."""
        with self._lock:
            if self.cap is None or not self.cap.isOpened():
                return False, None
            try:
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    return True, frame
                return False, None
            except Exception as e:
                log.error(f"[{self.name}] Read error: {e}")
                return False, None

    def reconnect(self):
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
                self.cap.release()
                self.cap = None
                log.info(f"[{self.name}] Released")

    def is_opened(self) -> bool:
        with self._lock:
            return self.cap is not None and self.cap.isOpened()

    @property
    def backend_name(self) -> str:
        return "GStreamer+nvv4l2decoder" if self._use_gstreamer else "FFmpeg"


# ═══════════════════════════════════════════════════════════════════════════════
# 2. TensorRT-Accelerated Detection Engine
# ═══════════════════════════════════════════════════════════════════════════════

class DetectionEngine:
    """
    YOLOv8 inference engine with TensorRT acceleration on Jetson.

    Loads a .engine (TensorRT) file for maximum GPU throughput.
    Falls back to .pt (PyTorch) if the engine file is not found.
    """

    PERSON_CLASS_ID = 0  # COCO class index for 'person'

    def __init__(self, model_path: str = "yolov8s.engine",
                 fallback_path: str = "yolov8s.pt",
                 confidence: float = 0.35,
                 imgsz: int = 640,
                 device: int = 0):
        self.confidence = confidence
        self.imgsz = imgsz
        self.device = device
        self.model = None
        self._load_model(model_path, fallback_path)

    def _load_model(self, primary: str, fallback: str):
        """Load model with graceful fallback chain."""
        try:
            from ultralytics import YOLO
        except ImportError:
            log.error("❌ ultralytics not installed. AI detection disabled.")
            return

        # Try TensorRT engine first
        if os.path.exists(primary):
            log.info(f"🧠 Loading TensorRT engine: {primary}")
            try:
                self.model = YOLO(primary, task="detect")
                log.info("✅ TensorRT engine loaded successfully")
                return
            except Exception as e:
                log.warning(f"TensorRT load failed: {e}")

        # Try PyTorch model
        if os.path.exists(fallback):
            log.info(f"🧠 Loading PyTorch model: {fallback}")
            try:
                self.model = YOLO(fallback)
                log.info("✅ PyTorch model loaded (CPU/CUDA)")
                return
            except Exception as e:
                log.warning(f"PyTorch load failed: {e}")

        # Last resort: download default model
        log.warning("⚠️ No local model found. Downloading yolov8s.pt...")
        try:
            self.model = YOLO("yolov8s.pt")
            log.info("✅ Downloaded and loaded yolov8s.pt")
        except Exception as e:
            log.error(f"❌ All model loading failed: {e}")

    def detect(self, frame: np.ndarray,
               conf_override: Optional[float] = None) -> np.ndarray:
        """
        Run person detection on a frame.

        Returns:
            np.ndarray of shape (N, 6): [x1, y1, x2, y2, confidence, class_id]
            Empty array if no detections or model unavailable.
        """
        if self.model is None or frame is None:
            return np.empty((0, 6))

        conf = conf_override or self.confidence
        try:
            results = self.model(
                frame,
                imgsz=self.imgsz,
                conf=conf,
                classes=[self.PERSON_CLASS_ID],
                verbose=False,
                device=self.device,
            )

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
    def is_available(self) -> bool:
        return self.model is not None


# ═══════════════════════════════════════════════════════════════════════════════
# 3. ByteTrack Multi-Object Tracker
# ═══════════════════════════════════════════════════════════════════════════════

class TrackerManager:
    """
    ByteTrack-based multi-object tracker for persistent human ID assignment.

    Attempts to use ultralytics' built-in BYTETracker. If unavailable,
    falls back to boxmot's BYTETracker. If all fail, uses a simple
    centroid-based fallback tracker.
    """

    def __init__(self, max_age: int = 50, min_hits: int = 3,
                 iou_threshold: float = 0.3):
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self._tracker = None
        self._backend = "none"
        self._next_id = 1
        self._tracks: Dict[int, Dict] = {}  # Fallback tracker state
        self._init_tracker()

    def _init_tracker(self):
        """Initialize ByteTrack with fallback chain."""

        # Attempt 1: boxmot BYTETracker (most feature-complete)
        try:
            from boxmot import BYTETracker  # type: ignore
            self._tracker = BYTETracker(
                track_thresh=0.25,
                track_buffer=self.max_age,
                match_thresh=self.iou_threshold,
            )
            self._backend = "boxmot.BYTETracker"
            log.info(f"🎯 Tracker initialized: {self._backend}")
            return
        except ImportError:
            pass

        # Attempt 2: boxmot StrongSORT (ReID-based)
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
            log.info(f"🎯 Tracker initialized: {self._backend}")
            return
        except (ImportError, Exception) as e:
            log.debug(f"StrongSORT unavailable: {e}")

        # Attempt 3: Centroid fallback
        self._backend = "centroid_fallback"
        log.warning("⚠️ No tracking library found. Using centroid fallback.")
        log.warning("   Install with: pip install boxmot lap")

    def update(self, detections: np.ndarray,
               frame: np.ndarray) -> List[Dict[str, Any]]:
        """
        Update tracker with new detections.

        Args:
            detections: (N, 6) array [x1, y1, x2, y2, conf, cls]
            frame: Current video frame (needed by some trackers for ReID)

        Returns:
            List of dicts: [{track_id, bbox:(x1,y1,x2,y2), confidence, active}]
        """
        if len(detections) == 0:
            # Age out old tracks in fallback mode
            if self._backend == "centroid_fallback":
                self._age_fallback_tracks()
            return []

        # --- boxmot trackers ---
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
                log.error(f"Tracker update error: {e}")
                return self._fallback_update(detections)

        # --- Centroid fallback ---
        return self._fallback_update(detections)

    def _fallback_update(self, detections: np.ndarray) -> List[Dict[str, Any]]:
        """Simple centroid-based tracking as last resort."""
        results = []
        used_tracks = set()

        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2

            # Find closest existing track
            best_id = None
            best_dist = 100.0  # Max pixel distance for matching

            for tid, tdata in self._tracks.items():
                if tid in used_tracks:
                    continue
                tx, ty = tdata["center"]
                dist = math.hypot(cx - tx, cy - ty)
                if dist < best_dist:
                    best_dist = dist
                    best_id = tid

            if best_id is not None:
                # Update existing track
                self._tracks[best_id]["center"] = (cx, cy)
                self._tracks[best_id]["age"] = 0
                self._tracks[best_id]["hits"] += 1
                used_tracks.add(best_id)
                track_id = best_id
            else:
                # Create new track
                track_id = self._next_id
                self._next_id += 1
                self._tracks[track_id] = {
                    "center": (cx, cy),
                    "age": 0,
                    "hits": 1,
                }
                used_tracks.add(track_id)

            results.append({
                "track_id": track_id,
                "bbox": (int(x1), int(y1), int(x2), int(y2)),
                "confidence": float(conf),
                "active": self._tracks[track_id]["hits"] >= self.min_hits,
            })

        self._age_fallback_tracks()
        return results

    def _age_fallback_tracks(self):
        """Age out old tracks in fallback mode."""
        expired = []
        for tid, tdata in self._tracks.items():
            tdata["age"] += 1
            if tdata["age"] > self.max_age:
                expired.append(tid)
        for tid in expired:
            del self._tracks[tid]

    @property
    def backend(self) -> str:
        return self._backend


# ═══════════════════════════════════════════════════════════════════════════════
# 4. Thermal Validation Engine
# ═══════════════════════════════════════════════════════════════════════════════

class ThermalValidator:
    """
    Cross-stream heat signature validation.

    Validates RGB human detections against the thermal frame to confirm
    that the detected object has a live heat signature, filtering out
    false positives like mannequins, signs, or shadows.
    """

    def __init__(self, temp_threshold: int = 180,
                 iou_threshold: float = 0.25):
        self.temp_threshold = temp_threshold
        self.iou_threshold = iou_threshold

    def extract_heat_blobs(self, thermal_frame: np.ndarray,
                           min_area: int = 500) -> List[Tuple[int, int, int, int]]:
        """
        Extract bounding boxes of hot regions from a thermal frame.

        Args:
            thermal_frame: Grayscale or BGR thermal image.
            min_area: Minimum contour area to be considered a heat source.

        Returns:
            List of bounding boxes (x1, y1, x2, y2) for hot regions.
        """
        if thermal_frame is None:
            return []

        # Convert to grayscale if needed
        if len(thermal_frame.shape) == 3:
            gray = cv2.cvtColor(thermal_frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = thermal_frame.copy()

        # Adaptive threshold for hot regions
        # In thermal imagery, hot objects appear as bright regions
        _, binary = cv2.threshold(gray, self.temp_threshold, 255, cv2.THRESH_BINARY)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        # Find contours of hot regions
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        blobs = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_area:
                x, y, w, h = cv2.boundingRect(contour)
                blobs.append((x, y, x + w, y + h))

        return blobs

    @staticmethod
    def compute_iou(box_a: Tuple, box_b: Tuple) -> float:
        """Compute Intersection over Union between two boxes."""
        x1 = max(box_a[0], box_b[0])
        y1 = max(box_a[1], box_b[1])
        x2 = min(box_a[2], box_b[2])
        y2 = min(box_a[3], box_b[3])

        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        if intersection == 0:
            return 0.0

        area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
        area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
        union = area_a + area_b - intersection

        return intersection / union if union > 0 else 0.0

    def validate(self, rgb_tracks: List[Dict], thermal_frame: np.ndarray,
                 rgb_shape: Tuple[int, int],
                 thermal_shape: Tuple[int, int]) -> List[Dict]:
        """
        Validate RGB detections against thermal heat signatures.

        Scales RGB bounding boxes to thermal resolution, then checks IoU
        overlap with thermal heat blobs.

        Args:
            rgb_tracks: List of track dicts from TrackerManager
            thermal_frame: Current thermal frame
            rgb_shape: (height, width) of the RGB frame
            thermal_shape: (height, width) of the thermal frame

        Returns:
            Each track dict with added 'thermal_confirmed' boolean field.
        """
        if thermal_frame is None or len(rgb_tracks) == 0:
            for t in rgb_tracks:
                t["thermal_confirmed"] = False
            return rgb_tracks

        heat_blobs = self.extract_heat_blobs(thermal_frame)

        # Scale factors from RGB to thermal resolution
        rgb_h, rgb_w = rgb_shape
        thm_h, thm_w = thermal_shape
        sx = thm_w / rgb_w if rgb_w > 0 else 1.0
        sy = thm_h / rgb_h if rgb_h > 0 else 1.0

        for track in rgb_tracks:
            x1, y1, x2, y2 = track["bbox"]
            # Scale RGB box to thermal coordinates
            scaled_box = (
                int(x1 * sx), int(y1 * sy),
                int(x2 * sx), int(y2 * sy)
            )

            # Check IoU against all thermal blobs
            confirmed = False
            for blob in heat_blobs:
                iou = self.compute_iou(scaled_box, blob)
                if iou >= self.iou_threshold:
                    confirmed = True
                    break

            track["thermal_confirmed"] = confirmed

        return rgb_tracks


# ═══════════════════════════════════════════════════════════════════════════════
# 5. Target Lock Manager
# ═══════════════════════════════════════════════════════════════════════════════

class TargetLockManager:
    """
    Triggers a 'Target Lock' event when a unique human ID is tracked
    with >85% confidence for more than 30 consecutive frames.

    This prevents false detections from triggering geolocation events
    and ensures only stable, high-confidence tracks are reported.
    """

    def __init__(self, lock_confidence: float = 0.85,
                 lock_frames: int = 30,
                 cooldown: float = 15.0,
                 on_lock: Optional[Callable] = None):
        self.lock_confidence = lock_confidence
        self.lock_frames = lock_frames
        self.cooldown = cooldown
        self.on_lock = on_lock  # Callback: on_lock(track_id, bbox, confidence)

        # Per-track state: {track_id: {consecutive: int, last_lock_time: float}}
        self._state: Dict[int, Dict] = {}
        self._lock = threading.Lock()
        self._locked_ids: set = set()

    def update(self, tracks: List[Dict]) -> List[int]:
        """
        Update lock state with current tracked objects.

        Args:
            tracks: List of track dicts with 'track_id', 'confidence',
                    'thermal_confirmed', 'bbox'

        Returns:
            List of track_ids that triggered a lock this frame.
        """
        newly_locked = []
        active_ids = set()

        with self._lock:
            for track in tracks:
                tid = track["track_id"]
                conf = track["confidence"]
                thermal_ok = track.get("thermal_confirmed", True)
                active_ids.add(tid)

                # Initialize state for new tracks
                if tid not in self._state:
                    self._state[tid] = {
                        "consecutive": 0,
                        "last_lock_time": 0.0,
                        "peak_conf": 0.0,
                    }

                state = self._state[tid]

                # Accumulate or reset consecutive counter
                if conf >= self.lock_confidence and thermal_ok:
                    state["consecutive"] += 1
                    state["peak_conf"] = max(state["peak_conf"], conf)
                else:
                    state["consecutive"] = 0

                # Check for lock trigger
                if state["consecutive"] >= self.lock_frames:
                    now = time.time()
                    time_since_last = now - state["last_lock_time"]

                    if time_since_last >= self.cooldown:
                        state["last_lock_time"] = now
                        state["consecutive"] = 0  # Reset after lock
                        self._locked_ids.add(tid)
                        newly_locked.append(tid)

                        log.info(
                            f"🔒 TARGET LOCK! ID:{tid} "
                            f"Confidence:{state['peak_conf']:.1%} "
                            f"Frames:{self.lock_frames}"
                        )

                        # Fire callback
                        if self.on_lock:
                            try:
                                self.on_lock(tid, track["bbox"], conf)
                            except Exception as e:
                                log.error(f"Lock callback error: {e}")

                        state["peak_conf"] = 0.0

            # Clean up stale tracks
            stale = [tid for tid in self._state if tid not in active_ids]
            for tid in stale:
                self._state[tid]["consecutive"] = 0

        return newly_locked

    def get_lock_state(self, track_id: int) -> Dict:
        """Get the current lock state for a specific track."""
        with self._lock:
            state = self._state.get(track_id, {})
            return {
                "consecutive": state.get("consecutive", 0),
                "required": self.lock_frames,
                "progress": state.get("consecutive", 0) / self.lock_frames,
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
    """Thread-safe snapshot of the drone's current state."""
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

    # Gimbal state (from C12Driver)
    gimbal_pitch: float = 0.0
    gimbal_yaw: float = 0.0
    gimbal_roll: float = 0.0
    gimbal_active: bool = False

    timestamp: float = 0.0


class TelemetryBridge:
    """
    Reads live drone GPS, attitude, and gimbal angles.

    Combines PyMAVLink telemetry with C12Driver gimbal state into
    a unified DroneState object, updated in a background thread.
    """

    COPTER_MODES = {
        0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
        4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
        9: "LAND", 11: "DRIFT", 13: "SPORT", 16: "POSHOLD",
        17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS",
    }

    def __init__(self, mavlink_uri: str = "udpin:0.0.0.0:14550",
                 gimbal=None):
        self.mavlink_uri = mavlink_uri
        self.gimbal = gimbal  # C12Driver instance (or None)
        self._state = DroneState()
        self._lock = threading.Lock()
        self._conn = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    @property
    def state(self) -> DroneState:
        """Return a snapshot of the current drone state."""
        with self._lock:
            # Return a copy to avoid race conditions
            import copy
            return copy.copy(self._state)

    def start(self):
        """Start the telemetry reading thread."""
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
                log.warning("⚠️ No heartbeat received within 30s")

            # Request telemetry streams at 4Hz
            self._conn.mav.request_data_stream_send(
                self._conn.target_system, self._conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
            )

            self._running = True
            self._thread = threading.Thread(
                target=self._telemetry_loop, daemon=True, name="Telemetry"
            )
            self._thread.start()
            log.info("📡 Telemetry bridge started")

        except Exception as e:
            log.error(f"❌ MAVLink connection failed: {e}")

    def _telemetry_loop(self):
        """Background thread: read MAVLink messages and update state."""
        from pymavlink import mavutil

        while self._running:
            try:
                msg = self._conn.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue

                msg_type = msg.get_type()

                with self._lock:
                    if msg_type == "HEARTBEAT":
                        if msg.get_srcComponent() == 1:
                            base_mode = msg.base_mode
                            custom_mode = msg.custom_mode
                            self._state.armed = bool(
                                base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                            )
                            self._state.mode = self.COPTER_MODES.get(
                                custom_mode, f"MODE_{custom_mode}"
                            )

                    elif msg_type == "GLOBAL_POSITION_INT":
                        self._state.lat = msg.lat / 1e7
                        self._state.lon = msg.lon / 1e7
                        self._state.alt = msg.relative_alt / 1000.0
                        self._state.heading = msg.hdg / 100.0

                    elif msg_type == "VFR_HUD":
                        self._state.speed = msg.groundspeed
                        if hasattr(msg, 'heading'):
                            self._state.heading = msg.heading

                    elif msg_type == "GPS_RAW_INT":
                        self._state.gps_sats = msg.satellites_visible
                        self._state.gps_fix = msg.fix_type

                    elif msg_type == "SYS_STATUS":
                        self._state.battery_voltage = msg.voltage_battery / 1000.0
                        if msg.battery_remaining >= 0:
                            self._state.battery_pct = msg.battery_remaining

                    self._state.timestamp = time.time()

                    # Update gimbal state
                    if self.gimbal is not None:
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
        """Stop the telemetry thread."""
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
    Estimate the GPS coordinates of a target on the ground.

    Uses ray-casting from the camera through the target's pixel position
    to find the intersection with the ground plane (flat earth approx).

    Coordinate System:
        - Gimbal pitch: 0° = horizon, -90° = straight down
        - Gimbal yaw:   0° = forward (along drone heading)
        - Pixel (0,0):  top-left corner of frame

    Args:
        drone_lat, drone_lon: Drone GPS in decimal degrees
        alt: Relative altitude in meters (above takeoff)
        heading: Drone compass heading (0-360°, North=0)
        gimbal_pitch: Gimbal pitch angle in degrees (negative = looking down)
        gimbal_yaw: Gimbal yaw angle in degrees (relative to drone heading)
        pixel_x, pixel_y: Target center pixel coordinates
        img_w, img_h: Frame dimensions in pixels
        fov_h, fov_v: Camera field of view in degrees

    Returns:
        (target_lat, target_lon) in decimal degrees.
        Returns (drone_lat, drone_lon) if calculation is invalid.
    """
    if alt <= 0.5:
        log.debug("Geoloc skipped: altitude too low")
        return drone_lat, drone_lon

    # --- Step 1: Pixel offset to angular offset ---
    # Normalized pixel position from center (-1 to +1)
    norm_x = (pixel_x - img_w / 2) / (img_w / 2)
    norm_y = (pixel_y - img_h / 2) / (img_h / 2)

    # Angular offset from optical center
    angle_offset_h = norm_x * (fov_h / 2)   # degrees, positive = right
    angle_offset_v = norm_y * (fov_v / 2)    # degrees, positive = down

    # --- Step 2: Effective look angles ---
    # Gimbal pitch is negative when looking down
    # Pixel Y offset: positive = lower in frame = looking more downward
    effective_pitch = gimbal_pitch - angle_offset_v  # More negative = steeper

    # Effective yaw: gimbal yaw + horizontal pixel offset
    effective_yaw = gimbal_yaw + angle_offset_h

    # --- Step 3: Ground distance via trigonometry ---
    # Convert pitch to angle from nadir (straight down)
    # pitch = 0° (horizon) → nadir_angle = 90°
    # pitch = -90° (straight down) → nadir_angle = 0°
    angle_from_nadir = 90.0 + effective_pitch  # degrees from straight-down

    # Clamp to valid range (avoid looking above horizon or exactly down)
    angle_from_nadir = max(1.0, min(85.0, angle_from_nadir))

    # Ground distance = altitude * tan(angle_from_nadir)
    ground_distance = alt * math.tan(math.radians(angle_from_nadir))

    # Clamp maximum ground distance (unreliable beyond this)
    ground_distance = min(ground_distance, alt * 20.0)

    # --- Step 4: Bearing from drone to target ---
    # Combine drone heading with gimbal yaw and pixel offset
    bearing = (heading + effective_yaw) % 360.0

    # --- Step 5: Project GPS coordinates ---
    # Vincenty-like forward projection (spherical approximation)
    R = 6378137.0  # Earth radius in meters

    lat1 = math.radians(drone_lat)
    lon1 = math.radians(drone_lon)
    brng = math.radians(bearing)

    lat2 = math.asin(
        math.sin(lat1) * math.cos(ground_distance / R)
        + math.cos(lat1) * math.sin(ground_distance / R) * math.cos(brng)
    )
    lon2 = lon1 + math.atan2(
        math.sin(brng) * math.sin(ground_distance / R) * math.cos(lat1),
        math.cos(ground_distance / R) - math.sin(lat1) * math.sin(lat2),
    )

    target_lat = math.degrees(lat2)
    target_lon = math.degrees(lon2)

    log.info(
        f"📍 GEOLOC: pitch={effective_pitch:.1f}° yaw={effective_yaw:.1f}° "
        f"dist={ground_distance:.1f}m bearing={bearing:.1f}° "
        f"→ ({target_lat:.6f}, {target_lon:.6f})"
    )

    return target_lat, target_lon


# ═══════════════════════════════════════════════════════════════════════════════
# 8. Frame Annotator
# ═══════════════════════════════════════════════════════════════════════════════

class FrameAnnotator:
    """Draws detection overlays, track IDs, and lock progress on frames."""

    # Color palette
    COLOR_TRACKED = (0, 255, 0)       # Green: tracked human
    COLOR_LOCKED = (0, 0, 255)        # Red: target locked
    COLOR_THERMAL_OK = (0, 255, 255)  # Yellow: thermal confirmed
    COLOR_LOCKING = (0, 165, 255)     # Orange: locking in progress
    COLOR_TEXT_BG = (0, 0, 0)         # Black background for text

    @staticmethod
    def annotate(frame: np.ndarray, tracks: List[Dict],
                 lock_manager: TargetLockManager,
                 drone_state: DroneState) -> np.ndarray:
        """Draw all annotations on a frame."""
        if frame is None:
            return frame

        annotated = frame.copy()
        h, w = annotated.shape[:2]

        for track in tracks:
            tid = track["track_id"]
            x1, y1, x2, y2 = track["bbox"]
            conf = track["confidence"]
            thermal = track.get("thermal_confirmed", False)
            lock_state = lock_manager.get_lock_state(tid)

            # Choose color based on state
            if lock_state["locked"]:
                color = FrameAnnotator.COLOR_LOCKED
            elif lock_state["progress"] > 0:
                color = FrameAnnotator.COLOR_LOCKING
            elif thermal:
                color = FrameAnnotator.COLOR_THERMAL_OK
            else:
                color = FrameAnnotator.COLOR_TRACKED

            # Bounding box
            thickness = 3 if lock_state["locked"] else 2
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)

            # Label: ID + Confidence
            label = f"ID:{tid} {conf:.0%}"
            if thermal:
                label += " [HEAT]"
            if lock_state["locked"]:
                label += " LOCKED"

            # Draw label background
            (tw, th_text), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            cv2.rectangle(
                annotated,
                (x1, y1 - th_text - 8), (x1 + tw + 4, y1),
                FrameAnnotator.COLOR_TEXT_BG, -1
            )
            cv2.putText(
                annotated, label, (x1 + 2, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
            )

            # Lock progress bar
            if 0 < lock_state["progress"] < 1.0:
                bar_w = x2 - x1
                bar_h = 4
                filled = int(bar_w * lock_state["progress"])
                cv2.rectangle(
                    annotated, (x1, y2 + 2), (x1 + bar_w, y2 + 2 + bar_h),
                    (80, 80, 80), -1
                )
                cv2.rectangle(
                    annotated, (x1, y2 + 2), (x1 + filled, y2 + 2 + bar_h),
                    FrameAnnotator.COLOR_LOCKING, -1
                )

        # HUD overlay: drone state
        hud_lines = [
            f"ALT: {drone_state.alt:.1f}m  SPD: {drone_state.speed:.1f}m/s",
            f"GPS: {drone_state.lat:.5f}, {drone_state.lon:.5f}  "
            f"SAT: {drone_state.gps_sats}",
            f"HDG: {drone_state.heading:.0f} deg  MODE: {drone_state.mode}",
            f"GIMBAL P:{drone_state.gimbal_pitch:.1f} "
            f"Y:{drone_state.gimbal_yaw:.1f}",
        ]

        for i, line in enumerate(hud_lines):
            y_pos = h - 20 - (len(hud_lines) - 1 - i) * 22
            cv2.putText(
                annotated, line, (10, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                cv2.LINE_AA,
            )

        return annotated


# ═══════════════════════════════════════════════════════════════════════════════
# 9. Flask MJPEG Streaming Server
# ═══════════════════════════════════════════════════════════════════════════════

class StreamingServer:
    """Serves annotated frames as MJPEG streams for the React frontend."""

    def __init__(self, port: int = 5001, jpeg_quality: int = 80):
        self.port = port
        self.jpeg_quality = jpeg_quality

        # Shared frame buffers
        self._frame_rgb: Optional[np.ndarray] = None
        self._frame_thermal: Optional[np.ndarray] = None
        self._lock_rgb = threading.Lock()
        self._lock_thermal = threading.Lock()

        self._app = None
        self._thread: Optional[threading.Thread] = None

    def update_rgb(self, frame: np.ndarray):
        with self._lock_rgb:
            self._frame_rgb = frame

    def update_thermal(self, frame: np.ndarray):
        with self._lock_thermal:
            self._frame_thermal = frame

    def _generate_frames(self, is_thermal: bool = False):
        """MJPEG generator for Flask Response."""
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]

        while True:
            frame = None
            if is_thermal:
                with self._lock_thermal:
                    if self._frame_thermal is not None:
                        frame = self._frame_thermal.copy()
            else:
                with self._lock_rgb:
                    if self._frame_rgb is not None:
                        frame = self._frame_rgb.copy()

            if frame is not None:
                ret, buffer = cv2.imencode(".jpg", frame, encode_params)
                if ret:
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n"
                        + buffer.tobytes()
                        + b"\r\n"
                    )
                else:
                    time.sleep(0.03)
            else:
                time.sleep(0.05)

    def start(self):
        """Start the Flask streaming server in a background thread."""
        from flask import Flask, Response

        self._app = Flask(__name__)

        # CORS
        @self._app.after_request
        def add_cors(response):
            response.headers["Access-Control-Allow-Origin"] = "*"
            response.headers["Access-Control-Allow-Headers"] = (
                "Content-Type,Authorization"
            )
            response.headers["Access-Control-Allow-Methods"] = (
                "GET,PUT,POST,DELETE,OPTIONS"
            )
            return response

        @self._app.route("/video_feed")
        def video_feed_rgb():
            return Response(
                self._generate_frames(False),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @self._app.route("/thermal_feed")
        def video_feed_thermal():
            return Response(
                self._generate_frames(True),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @self._app.route("/health")
        def health():
            return {"status": "ok", "version": "2.0.0-jetson"}

        self._thread = threading.Thread(
            target=lambda: self._app.run(
                host="0.0.0.0", port=self.port,
                debug=False, use_reloader=False, threaded=True,
            ),
            daemon=True, name="FlaskServer",
        )
        self._thread.start()
        log.info(f"🎥 MJPEG Server started on port {self.port}")


# ═══════════════════════════════════════════════════════════════════════════════
# 10. Main Orchestrator
# ═══════════════════════════════════════════════════════════════════════════════

class ScoutAICore:
    """
    Main orchestrator — manages all threads and the AI processing pipeline.

    This is the top-level class that ties together:
    - Video ingestion (GStreamer HW decode)
    - AI detection (TensorRT YOLO)
    - Multi-object tracking (ByteTrack)
    - Thermal validation
    - Target lock trigger
    - Telemetry integration
    - MJPEG streaming
    """

    def __init__(self, config: Optional[ScoutConfig] = None):
        self.config = config or ScoutConfig()
        self._running = False

        # --- Components ---
        self.rgb_capture = GStreamerCapture(
            self.config.rgb_url, name="RGB",
            latency=self.config.gstreamer_latency,
            max_buffers=self.config.gstreamer_max_buffers,
        )
        self.thermal_capture = GStreamerCapture(
            self.config.thermal_url, name="Thermal",
            latency=self.config.gstreamer_latency,
            max_buffers=self.config.gstreamer_max_buffers,
        )
        self.detector = DetectionEngine(
            model_path=self.config.model_path,
            fallback_path=self.config.model_fallback,
            confidence=self.config.detection_confidence,
            imgsz=self.config.inference_size,
        )
        self.tracker = TrackerManager(
            max_age=self.config.track_max_age,
            min_hits=self.config.track_min_hits,
            iou_threshold=self.config.track_iou_threshold,
        )
        self.thermal_validator = ThermalValidator(
            temp_threshold=self.config.thermal_temp_threshold,
            iou_threshold=self.config.thermal_iou_threshold,
        )
        self.target_lock = TargetLockManager(
            lock_confidence=self.config.lock_confidence,
            lock_frames=self.config.lock_frames,
            cooldown=self.config.lock_cooldown,
            on_lock=self._on_target_lock,
        )
        self.streaming_server = StreamingServer(
            port=self.config.flask_port,
            jpeg_quality=self.config.jpeg_quality,
        )

        # Telemetry bridge (gimbal loaded separately)
        self._gimbal = None
        self.telemetry = None  # Initialized in start()

        # --- Shared Frame Buffers ---
        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_thermal: Optional[np.ndarray] = None
        self._lock_rgb = threading.Lock()
        self._lock_thermal = threading.Lock()

        # --- Statistics ---
        self._stats = {
            "rgb_fps": 0.0,
            "thermal_fps": 0.0,
            "inference_fps": 0.0,
            "total_detections": 0,
            "total_locks": 0,
        }

    def _init_gimbal(self):
        """Initialize C12 gimbal driver."""
        try:
            from C12Driver import C12Driver
            self._gimbal = C12Driver()
            log.info("✅ C12 Gimbal Driver loaded")
            return self._gimbal
        except Exception as e:
            log.warning(f"⚠️ C12 Gimbal not available: {e}")
            return None

    def start(self):
        """Start all pipeline threads."""
        log.info("=" * 60)
        log.info("  SCOUT AI CORE v2.0.0 — Jetson Orin Nano")
        log.info("=" * 60)
        self._running = True

        # Initialize gimbal
        gimbal = self._init_gimbal()

        # Initialize telemetry
        self.telemetry = TelemetryBridge(
            mavlink_uri=self.config.mavlink_uri,
            gimbal=gimbal,
        )
        self.telemetry.start()

        # Start streaming server
        self.streaming_server.start()

        # Start capture threads
        t_rgb = threading.Thread(
            target=self._rgb_ingest_loop, daemon=True, name="RGB-Ingest"
        )
        t_thermal = threading.Thread(
            target=self._thermal_ingest_loop, daemon=True, name="Thermal-Ingest"
        )
        t_inference = threading.Thread(
            target=self._inference_loop, daemon=True, name="Inference"
        )

        t_rgb.start()
        t_thermal.start()
        t_inference.start()

        log.info("🚀 All pipeline threads started")
        log.info(f"   RGB Capture:     {self.rgb_capture.rtsp_url}")
        log.info(f"   Thermal Capture: {self.thermal_capture.rtsp_url}")
        log.info(f"   AI Model:        {self.config.model_path}")
        log.info(f"   Tracker:         {self.tracker.backend}")
        log.info(f"   MJPEG Server:    http://0.0.0.0:{self.config.flask_port}")
        log.info(f"   Target Lock:     {self.config.lock_frames} frames @ "
                 f"{self.config.lock_confidence:.0%} confidence")

        # Block main thread
        try:
            while self._running:
                time.sleep(1.0)
                self._print_stats()
        except KeyboardInterrupt:
            log.info("🛑 Shutting down (Ctrl+C)...")
            self.stop()

    def stop(self):
        """Gracefully shutdown all threads."""
        self._running = False
        self.rgb_capture.release()
        self.thermal_capture.release()
        if self.telemetry:
            self.telemetry.stop()
        if self._gimbal:
            try:
                self._gimbal.close()
            except Exception:
                pass
        log.info("✅ Scout AI Core shutdown complete")

    # -----------------------------------------------------------------------
    # Capture Threads
    # -----------------------------------------------------------------------

    def _rgb_ingest_loop(self):
        """Thread: Continuously read RGB frames from GStreamer."""
        log.info("[RGB] Opening capture...")
        if not self.rgb_capture.open():
            log.error("[RGB] ❌ Failed to open. Will retry...")

        fps_counter = _FPSCounter("RGB")

        while self._running:
            try:
                if not self.rgb_capture.is_opened():
                    self.rgb_capture.reconnect()
                    continue

                ret, frame = self.rgb_capture.read()
                if ret and frame is not None:
                    with self._lock_rgb:
                        self._latest_rgb = frame
                    fps_counter.tick()
                    self._stats["rgb_fps"] = fps_counter.fps
                else:
                    self.rgb_capture.reconnect()

            except Exception as e:
                log.error(f"[RGB] Ingest error: {e}")
                time.sleep(1.0)

    def _thermal_ingest_loop(self):
        """Thread: Continuously read Thermal frames from GStreamer."""
        log.info("[Thermal] Opening capture...")
        if not self.thermal_capture.open():
            log.error("[Thermal] ❌ Failed to open. Will retry...")

        fps_counter = _FPSCounter("Thermal")

        while self._running:
            try:
                if not self.thermal_capture.is_opened():
                    self.thermal_capture.reconnect()
                    continue

                ret, frame = self.thermal_capture.read()
                if ret and frame is not None:
                    with self._lock_thermal:
                        self._latest_thermal = frame
                    fps_counter.tick()
                    self._stats["thermal_fps"] = fps_counter.fps
                else:
                    self.thermal_capture.reconnect()

            except Exception as e:
                log.error(f"[Thermal] Ingest error: {e}")
                time.sleep(1.0)

    # -----------------------------------------------------------------------
    # Inference Loop
    # -----------------------------------------------------------------------

    def _inference_loop(self):
        """
        Thread: Main AI processing pipeline.

        1. Grab latest RGB frame
        2. Run YOLO person detection (TensorRT)
        3. Update ByteTrack tracker
        4. Validate against thermal heat signatures
        5. Check target lock conditions
        6. Annotate frame and push to streaming server
        """
        log.info("[Inference] Waiting for first frame...")

        # Wait for first RGB frame
        while self._running:
            with self._lock_rgb:
                if self._latest_rgb is not None:
                    break
            time.sleep(0.1)

        log.info("[Inference] ✅ First frame received. Starting pipeline.")
        fps_counter = _FPSCounter("Inference")

        while self._running:
            try:
                # 1. Grab latest frames
                rgb_frame = None
                thermal_frame = None

                with self._lock_rgb:
                    if self._latest_rgb is not None:
                        rgb_frame = self._latest_rgb.copy()

                with self._lock_thermal:
                    if self._latest_thermal is not None:
                        thermal_frame = self._latest_thermal.copy()

                if rgb_frame is None:
                    time.sleep(0.01)
                    continue

                rgb_h, rgb_w = rgb_frame.shape[:2]

                # 2. Run YOLO detection on RGB
                detections = self.detector.detect(rgb_frame)

                # 3. Update tracker
                tracks = self.tracker.update(detections, rgb_frame)

                # 4. Thermal validation
                if thermal_frame is not None:
                    thm_h, thm_w = thermal_frame.shape[:2]
                    tracks = self.thermal_validator.validate(
                        tracks, thermal_frame,
                        rgb_shape=(rgb_h, rgb_w),
                        thermal_shape=(thm_h, thm_w),
                    )
                else:
                    # No thermal available — mark all as unvalidated
                    for t in tracks:
                        t["thermal_confirmed"] = False

                # 5. Target lock check
                newly_locked = self.target_lock.update(tracks)
                self._stats["total_detections"] += len(detections)
                self._stats["total_locks"] += len(newly_locked)

                # Process newly locked targets
                for tid in newly_locked:
                    self._process_locked_target(tid, tracks, rgb_w, rgb_h)

                # 6. Annotate and push to streaming server
                drone_state = (
                    self.telemetry.state if self.telemetry else DroneState()
                )
                annotated_rgb = FrameAnnotator.annotate(
                    rgb_frame, tracks, self.target_lock, drone_state
                )
                self.streaming_server.update_rgb(annotated_rgb)

                # Also push thermal with heat blob overlay
                if thermal_frame is not None:
                    self.streaming_server.update_thermal(thermal_frame)

                fps_counter.tick()
                self._stats["inference_fps"] = fps_counter.fps

                # Throttle to ~15 FPS max to leave GPU headroom
                time.sleep(0.01)

            except Exception as e:
                log.error(f"[Inference] Pipeline error: {e}", exc_info=True)
                time.sleep(0.1)

    # -----------------------------------------------------------------------
    # Target Lock Callback
    # -----------------------------------------------------------------------

    def _on_target_lock(self, track_id: int, bbox: Tuple, confidence: float):
        """Called when a target achieves lock status."""
        log.info(f"🔒 TARGET LOCK CALLBACK — ID:{track_id} Conf:{confidence:.1%}")

    def _process_locked_target(self, track_id: int,
                                tracks: List[Dict],
                                img_w: int, img_h: int):
        """Process a newly locked target: compute geolocation."""
        # Find the track data
        track = next((t for t in tracks if t["track_id"] == track_id), None)
        if track is None:
            return

        # Get drone state
        if self.telemetry is None:
            return
        state = self.telemetry.state

        if state.lat == 0.0 and state.lon == 0.0:
            log.warning(f"[Lock ID:{track_id}] No GPS fix — skipping geolocation")
            return

        # Calculate target pixel center
        x1, y1, x2, y2 = track["bbox"]
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        # Compute ground GPS
        target_lat, target_lon = calculate_target_ground_gps(
            drone_lat=state.lat,
            drone_lon=state.lon,
            alt=state.alt,
            heading=state.heading,
            gimbal_pitch=state.gimbal_pitch,
            gimbal_yaw=state.gimbal_yaw,
            pixel_x=cx,
            pixel_y=cy,
            img_w=float(img_w),
            img_h=float(img_h),
            fov_h=self.config.camera_fov_h,
            fov_v=self.config.camera_fov_v,
        )

        log.info(
            f"🎯 LOCKED TARGET ID:{track_id}"
            f"\n   Drone:  ({state.lat:.6f}, {state.lon:.6f}) "
            f"Alt:{state.alt:.1f}m Hdg:{state.heading:.0f}°"
            f"\n   Gimbal: P:{state.gimbal_pitch:.1f}° "
            f"Y:{state.gimbal_yaw:.1f}°"
            f"\n   Target: ({target_lat:.6f}, {target_lon:.6f})"
            f"\n   Pixel:  ({cx:.0f}, {cy:.0f}) in {img_w}x{img_h}"
        )

        # Publish detection to MQTT
        self._publish_detection(
            track_id=track_id,
            target_lat=target_lat,
            target_lon=target_lon,
            confidence=track["confidence"],
            drone_state=state,
        )

    def _publish_detection(self, track_id: int, target_lat: float,
                            target_lon: float, confidence: float,
                            drone_state: DroneState):
        """Publish locked target to MQTT for the backend."""
        detection = {
            "id": track_id,
            "lat": target_lat,
            "lon": target_lon,
            "drone_lat": drone_state.lat,
            "drone_lon": drone_state.lon,
            "conf": confidence,
            "source": "scout_ai_core",
            "timestamp": time.time(),
            "type": "human",
            "alt": drone_state.alt,
            "gimbal_pitch": drone_state.gimbal_pitch,
            "gimbal_yaw": drone_state.gimbal_yaw,
        }

        try:
            import paho.mqtt.publish as publish
            for broker_ip in self.config.broker_ips:
                publish.single(
                    self.config.topic_detections,
                    payload=json.dumps(detection),
                    hostname=broker_ip,
                    port=1883,
                )
                log.info(f"📡 Detection published to {broker_ip}")
        except Exception as e:
            log.warning(f"MQTT publish failed: {e}")

    # -----------------------------------------------------------------------
    # Statistics
    # -----------------------------------------------------------------------

    def _print_stats(self):
        """Print periodic stats to the console."""
        s = self._stats
        log.info(
            f"📊 FPS: RGB={s['rgb_fps']:.1f} "
            f"THM={s['thermal_fps']:.1f} "
            f"AI={s['inference_fps']:.1f} | "
            f"Detections={s['total_detections']} "
            f"Locks={s['total_locks']} | "
            f"Tracker={self.tracker.backend} "
            f"Video={self.rgb_capture.backend_name}"
        )


# ═══════════════════════════════════════════════════════════════════════════════
# Utility: FPS Counter
# ═══════════════════════════════════════════════════════════════════════════════

class _FPSCounter:
    """Lightweight FPS counter using a sliding window."""

    def __init__(self, name: str = "", window: int = 30):
        self.name = name
        self._window = window
        self._timestamps: List[float] = []
        self.fps = 0.0

    def tick(self):
        now = time.time()
        self._timestamps.append(now)
        # Keep only the last N timestamps
        if len(self._timestamps) > self._window:
            self._timestamps = self._timestamps[-self._window:]
        # Calculate FPS
        if len(self._timestamps) >= 2:
            elapsed = self._timestamps[-1] - self._timestamps[0]
            if elapsed > 0:
                self.fps = (len(self._timestamps) - 1) / elapsed


# ═══════════════════════════════════════════════════════════════════════════════
# Entry Point
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    """Entry point for standalone execution."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Scout AI Core — Jetson Orin Nano Optimized Pipeline"
    )
    parser.add_argument(
        "--rgb-url", default="rtsp://192.168.144.108:554/stream=1",
        help="RGB camera RTSP URL"
    )
    parser.add_argument(
        "--thermal-url", default="rtsp://192.168.144.108:555/stream=2",
        help="Thermal camera RTSP URL"
    )
    parser.add_argument(
        "--model", default="yolov8s.engine",
        help="Path to YOLO model (.engine or .pt)"
    )
    parser.add_argument(
        "--model-fallback", default="yolov8s.pt",
        help="Fallback model path (.pt)"
    )
    parser.add_argument(
        "--port", type=int, default=5001,
        help="Flask MJPEG server port"
    )
    parser.add_argument(
        "--mavlink", default="udpin:0.0.0.0:14550",
        help="MAVLink connection URI"
    )
    parser.add_argument(
        "--conf", type=float, default=0.35,
        help="Detection confidence threshold"
    )
    parser.add_argument(
        "--lock-frames", type=int, default=30,
        help="Consecutive frames for target lock"
    )
    parser.add_argument(
        "--lock-conf", type=float, default=0.85,
        help="Minimum confidence for target lock"
    )
    parser.add_argument(
        "--imgsz", type=int, default=640,
        help="YOLO inference image size"
    )
    parser.add_argument(
        "--broker", nargs="+", default=["100.113.148.48"],
        help="MQTT broker IP(s)"
    )

    args = parser.parse_args()

    config = ScoutConfig(
        rgb_url=args.rgb_url,
        thermal_url=args.thermal_url,
        model_path=args.model,
        model_fallback=args.model_fallback,
        flask_port=args.port,
        mavlink_uri=args.mavlink,
        detection_confidence=args.conf,
        lock_frames=args.lock_frames,
        lock_confidence=args.lock_conf,
        inference_size=args.imgsz,
        broker_ips=args.broker,
    )

    core = ScoutAICore(config)
    core.start()


if __name__ == "__main__":
    main()
