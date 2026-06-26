from pymavlink import mavutil
import paho.mqtt.client as mqtt
import json
import time
import threading
import cv2
import numpy as np
from datetime import datetime
import math
from flask import Flask, Response
import os
import struct
import socket
import subprocess
from pathlib import Path
from ultralytics import YOLO # AI Model
YOLO_AVAILABLE = True

# Enforce strict low-latency flags for OpenCV FFmpeg to solve the "high latency" issue
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|fflags;nobuffer|analyzeduration;10000|probesize;32000"
VERSION = "1.2 (Outdoor Mode Enabled)"
print(f"🚀 SCOUT SCRIPT VERSION: {VERSION} Started at {datetime.now()}")

# 🔴 CONFIGURATION - YOUR TAILSCALE IP
BROKER_IPS = ["100.113.148.48"] # Your laptop's Tailscale IP (mithilesh-m)

TOPIC_MISSION = "nidar/scout/mission"
TOPIC_TELEM = "nidar/scout/telemetry"
TOPIC_SURVIVOR = "nidar/delivery/target"
TOPIC_DETECTIONS = "nidar/scout/detections"  # Human detection topic

# CAMERA & DETECTION SETTINGS
# Camera IDs
CAM_ID_WEBCAM = 0  # USB Webcam
# C12 Skydroid Gimbal Camera RTSP URLs (Updated with correct streams)
CAM_ID_RGB = "rtsp://192.168.144.108:554/stream=1"       # C12 RGB main stream
CAM_ID_THERMAL = "rtsp://192.168.144.108:555/stream=2"   # C12 Thermal stream

CAMERA_ENABLED = True
FLASK_PORT = 5001

# CAMERA FLAGS (Enable/Disable streams here)
ENABLE_RGB = True       # C12 RGB Camera (Enabled)
ENABLE_THERMAL = True   # C12 Thermal Camera (Enabled)
ENABLE_WEBCAM = False   # USB Camera (Disabled - not connected)

# DETECTION CONFIG
DETECTION_FPS = 10.0 # Limit detection checks
DETECTION_CONFIDENCE = 0.35
DETECTION_COOLDOWN = 15 # Seconds
DETECTION_MIN_DISTANCE = 2 # Meters - minimum distance between unique detections (was 10)

# STRONGSORT TRACKING CONFIG (Scout Plan)
TRACK_CONFIRMATION_FRAMES = 5  # Frames before confirming a track
TRACK_MAX_AGE = 30  # Max frames to keep lost track
HELMET_DETECTION_ENABLED = True  # Enable yellow helmet detection
FUSION_MODE = True  # RGB + Thermal fusion (thermal primary, RGB confirmation)

# Yellow Helmet HSV Range (Adjust based on lighting)
HELMET_HSV_LOW = (20, 100, 100)   # Lower bound (H, S, V)
HELMET_HSV_HIGH = (35, 255, 255)  # Upper bound
HELMET_MIN_PIXELS = 300  # Minimum yellow pixels to confirm helmet

# FAILSAFE CONFIGURATION
RTL_SPEED_MPS = 8.0     # Assumed average speed during RTL
FAILSAFE_AMPS = 26.0    # Conservative current draw estimate

# GEOLOCATION CONFIGURATION
CAMERA_FOV_H = 62.2  # Horizontal Field of View (degrees) - Standard for RPi Cam V2
CAMERA_FOV_V = 48.8  # Vertical Field of View (degrees)
CAMERA_PITCH_DEG = -45.0  # Camera pitch angle (Fixed 45 deg down) 
# Note: Ensure this matches your physical mount angle!

COPTER_MODES = {
    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED', 5: 'LOITER', 
    6: 'RTL', 7: 'CIRCLE', 9: 'LAND', 11: 'DRIFT', 13: 'SPORT', 16: 'POSHOLD', 
    17: 'BRAKE', 18: 'THROW', 19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS'
}

# --- SKYBROID GIMBAL CONTROL ---
# --- GIMBAL CONTROL (DISABLED) ---
# C12 Code removed as per request
class DummyGimbal:
    def center(self): pass
    def look_down_full(self): pass
    def set_search_angle(self): pass
    def rotate_pitch(self, speed): pass

skybroid_gimbal = DummyGimbal()

# --- MAVLINK CONNECTION ---

print("🔌 Connecting to Mavlink Router (UDP)...")
# Use 0.0.0.0 to listen on all interfaces (Localhost + Tailscale/LAN)
drone = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

print("⏳ Waiting for heartbeat...")
while True:
    msg = drone.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg:
        drone.target_system = msg.get_srcSystem()
        drone.target_component = msg.get_srcComponent()
        print(f"✅ Heartbeat from System {drone.target_system}, Component {drone.target_component}")
        break
    print("⚠️ No heartbeat yet... check Mavlink Router is running and sending to port 14550")

print("✅ Scout Online")

# Request telemetry streams from the flight controller
print("📡 Requesting telemetry streams...")
drone.mav.request_data_stream_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    4, 1)  # 4Hz rate, enable=1

def get_drone_ip():
    """Detect drone IP (prefer Tailscale)."""
    try:
        return subprocess.check_output(["tailscale", "ip", "-4"], text=True).strip()
    except:
        try:
            # Fallback to first non-loopback IP
            cmd = "hostname -I | awk '{print $1}'"
            return subprocess.check_output(cmd, shell=True, text=True).strip()
        except:
            return "127.0.0.1"

# --- STATE ---
DRONE_DATA = {
    "lat": 0, "lon": 0, "alt": 0, 
    "bat": 0,           # Battery percentage (0-100)
    "bat_voltage": 0,   # Voltage in volts
    "bat_current": 0,   # Current draw in amps
    "bat_time_min": 0,  # Estimated minutes remaining
    "req_rtl_batt": 0,  # REQUIRED battery % for RTL
    "gps_sats": 0,      # Number of GPS satellites
    "gps_fix": 0,
    "status": "DISARMED", "speed": 0, "mode": "UNKNOWN",
    "detections_count": 0,  # Total humans detected
    "home_lat": 0, "home_lon": 0, # Home location (Set on Arm)
    "heading": 0,
    "dist_to_home": 0,
    "gimbal_pitch": -45.0, # Approximate
    "ip": get_drone_ip()    # Auto-detected IP
}
print(f"🌍 DRONE IP DETECTED: {DRONE_DATA['ip']}")
BATTERY_CAPACITY_MAH = 5000  # Default battery capacity, adjust for your battery
LAST_HEARTBEAT = 0
MISSION_UPLOAD_IN_PROGRESS = False  # Track mission upload state
AUTO_START_MISSION = False  # Flag to auto-start mission after upload
MISSION_POINTS = [] # Store mission to respond to FC requests
MISSION_FILE = "scout_mission.json"

def save_mission_state():
    try:
        with open(MISSION_FILE, 'w') as f:
            json.dump({"points": MISSION_POINTS, "current_wp": DRONE_DATA.get("current_wp", -1)}, f)
    except: pass

def load_mission_state():
    global MISSION_POINTS
    try:
        if os.path.exists(MISSION_FILE):
            with open(MISSION_FILE, 'r') as f:
                state = json.load(f)
                MISSION_POINTS = state.get("points", [])
                return state.get("current_wp", 0)
    except: pass
    return -1

# DETECTION STATE

# DETECTION STATE
DETECTIONS = []  # List of detected humans: [{"lat": X, "lon": Y, "timestamp": T, "id": unique_id}]
LAST_DETECTION_TIME = 0
DETECTION_ID_COUNTER = 0  # Unique ID for each detection
INDOOR_MODE = False

# VIDEO STREAMING STATE
frame_rgb = None
frame_thermal = None
lock_rgb = threading.Lock()
lock_thermal = threading.Lock()
lock_webcam = threading.Lock()
frame_webcam = None
# lock_webcam already defined above
# GLOBAL AI MODEL
model = None 
mqtt_clients = []
app = Flask(__name__)

# Enable CORS for all domains/IPs (Allows video feed on all laptops)
@app.after_request
def add_cors(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
    return response

# Human-readable mapping for MAV_CMD_ACK results
MAV_RESULT_NAMES = {
    mavutil.mavlink.MAV_RESULT_ACCEPTED: "Accepted",
    mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: "Temp Rejected",
    mavutil.mavlink.MAV_RESULT_DENIED: "Denied",
    mavutil.mavlink.MAV_RESULT_UNSUPPORTED: "Unsupported",
    mavutil.mavlink.MAV_RESULT_FAILED: "Failed",
    mavutil.mavlink.MAV_RESULT_IN_PROGRESS: "In Progress",
    getattr(mavutil.mavlink, 'MAV_RESULT_CANCELLED', 6): "Cancelled"
}

def set_flight_mode(mode):
    """Robust mode setting using MAVLink command."""
    if mode not in COPTER_MODES.values():
        print(f"⚠️ Mode {mode} not in COPTER_MODES")
        return
    
    # Get mode ID
    mode_id = next(k for k, v in COPTER_MODES.items() if v == mode)
    print(f"🔄 Switching to {mode} (ID: {mode_id})...")
    
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0)

# --- COMMAND HELPERS ---

# Duplicate get_target_gps removed (See "GEOLOCATION MATH" section below)

def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate distance in meters between two GPS coordinates using Haversine formula."""
    R = 6371000  # Earth radius in meters
    
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)
    
    a = math.sin(delta_lat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

def voltage_to_percent(v):
    """Fallback calculation for battery % when FC doesn't report it."""
    if v < 5: return -1 # No voltage detected
    
    # Robust cell detection (max 4.25V per cell)
    cells = math.ceil(v / 4.25)
    if cells < 1: return -1
    
    # LiPo Range: 3.5V (empty) to 4.2V (full) per cell
    min_v = cells * 3.5
    max_v = cells * 4.2
    
    pct = ((v - min_v) / (max_v - min_v)) * 100
    return max(0, min(100, round(pct, 1)))

def calculate_smart_rtl():
    """Calculate required battery % for RTL based on distance and power profile."""
    # 1. Check if we have a valid Home Location
    home_lat = DRONE_DATA.get('home_lat', 0)
    home_lon = DRONE_DATA.get('home_lon', 0)
    curr_lat = DRONE_DATA.get('lat', 0)
    curr_lon = DRONE_DATA.get('lon', 0)
    
    if home_lat == 0 or curr_lat == 0:
        DRONE_DATA['req_rtl_batt'] = 0
        return

    # 2. Calculate Distance to Home
    dist_home = calculate_distance(curr_lat, curr_lon, home_lat, home_lon)
    DRONE_DATA['dist_to_home'] = dist_home
    
    # 3. Calculate Time to Return (in hours)
    # Time = Distance / Speed
    time_hours = dist_home / (RTL_SPEED_MPS * 3600.0) # Convert seconds to hours
    
    # 4. Calculate Capacity Consumed (Ah)
    # Capacity = Current (Amps) * Time (Hours)
    capacity_consumed_ah = FAILSAFE_AMPS * time_hours
    
    # 5. Convert to Percentage of Total Capacity
    # We use BATTERY_CAPACITY_MAH (which is in mAh, so divide by 1000 for Ah)
    total_capacity_ah = BATTERY_CAPACITY_MAH / 1000.0
    req_pct = (capacity_consumed_ah / total_capacity_ah) * 100.0
    
    # 6. Add Safety Margin (e.g., +5%)
    req_pct += 5.0
    
    DRONE_DATA['req_rtl_batt'] = round(req_pct, 1)

def send_arm():
    
    print(f"⚠️ Sending ARM Command (Current Mode: {DRONE_DATA.get('mode')})...")
    # Set Disarm Delay to 2 minutes (120s) instead of 0 to ensure it's disabled or very long
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'DISARM_DELAY', 120, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(0.5)
    
    try:
        drone.mav.command_long_send(
            drone.target_system, drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        
        # OPTIMISTIC UPDATE: Force status immediately to prevent UI flicker
        DRONE_DATA['status'] = "ARMED" 
        print("✅ Optimistic Status: ARMED")
    except Exception as e:
        print(f"❌ ARM Params Failed: {e}")

def send_disarm(force=True):
    """Disarm the drone. Uses force=True by default to bypass safety checks."""
    print("⚠️ Sending DISARM Command...")
    # ArduPilot magic number 21196 forces disarm bypassing safety checks
    magic_override = 21196 if force else 0
    
    # If not forcing, switch to LAND mode first
    if not force and DRONE_DATA.get('mode') != 'LAND':
        print("⚠️ Switching to LAND mode before DISARM...")
        set_flight_mode('LAND')
        time.sleep(2)  # Wait for landing to initiate
    
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, magic_override, 0, 0, 0, 0, 0)
    
    # OPTIMISTIC UPDATE: Force status immediately
    DRONE_DATA['status'] = "DISARMED"
    print("✅ Optimistic Status: DISARMED")

def set_indoor_mode():
    global INDOOR_MODE
    INDOOR_MODE = True
    print("⚠️ SETTING INDOOR MODE (Safety Checks Disabled + GPS Bypass)")
    # Disable Arming Checks
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    # Disable Radio Failsafe
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'FS_THR_ENABLE', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    # Ensure motor spin
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'MOT_SPIN_ARM', 0.10, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    
    # Set BATT_LOW_VOLT to 20.0V (Account for ~2V sag under load)
    # Param Index -1 means strictly by name
    drone.mav.param_set_send(
        drone.target_system, drone.target_component,
        b'BATT_LOW_VOLT',
        20.0,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    # Disable Battery Failsafe
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'BATT_FS_LOW_ACT', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'BATT_FS_CRT_ACT', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    # Disable Battery Monitor entirely (Bypass -1% check)
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'BATT_MONITOR', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    # Disable Auto-Disarm (Prevents flickering from ARMED to DISARMED on ground)
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'DISARM_DELAY', 120, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    
    start_rc_override()

RC_OVERRIDE_ACTIVE = False
# Channel Map: 1:Roll, 2:Pitch, 3:Throttle, 4:Yaw, 5:SwA, 6:SwB, 7:SwC, 8:SwD
RC_OVERRIDE_CHANNELS = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000] 

def rc_override_thread():
    global RC_OVERRIDE_ACTIVE
    print("🕹️ RC Override Thread Started (Daemon)")
    while True:
        if RC_OVERRIDE_ACTIVE:
            # Send the current state of channels continuously
            # This heartbeat satisfies FS_THR_ENABLE=1 without needing Real RC
            drone.mav.rc_channels_override_send(
                drone.target_system, drone.target_component,
                *RC_OVERRIDE_CHANNELS
            )
        time.sleep(0.1) # 10Hz update rate

def start_rc_override():
    global RC_OVERRIDE_ACTIVE, RC_OVERRIDE_CHANNELS
    if not RC_OVERRIDE_ACTIVE:
        print("⚠️ Enabling RC Override (Sending Heartbeat)...")
        # Reset to safe defaults (Throttle down)
        RC_OVERRIDE_CHANNELS = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        RC_OVERRIDE_ACTIVE = True

def stop_rc_override():
    global RC_OVERRIDE_ACTIVE
    print("🕹️ Disabling RC Override (Releasing Control)...")
    RC_OVERRIDE_ACTIVE = False
    # Send 0 to release control back to real RC
    pwm = [0] * 8
    drone.mav.rc_channels_override_send(drone.target_system, drone.target_component, *pwm)

# Start the background thread once
threading.Thread(target=rc_override_thread, daemon=True).start()

def ensure_battery_monitor():
    """Enforce Analog Battery Monitor (4) on startup."""
    print("🔋 Enforcing Battery Monitor Settings...")
    # BATT_MONITOR = 4 (Analog Voltage and Current)
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'BATT_MONITOR', 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(0.2)
    # BATT_VOLT_PIN = 2 (Pixhawk Standard) - Might vary by FC, but standard is 2 or 0
    # BATT_CURR_PIN = 3 
    print("✅ Battery Monitor Command Sent")

def override_rc(channels):
    """Update defaults for the override thread."""
    global RC_OVERRIDE_CHANNELS
    # Update global state, let the thread send it
    for i, val in enumerate(channels):
        if i < 8: RC_OVERRIDE_CHANNELS[i] = int(val)

def upload_mission_to_fc(waypoints, alt=20, speed=None, auto_start=True, is_resume=False):
    global MISSION_POINTS, MISSION_UPLOAD_IN_PROGRESS, AUTO_START_MISSION
    print(f"📤 Uploading {len(waypoints)} points (Alt: {alt}m, Speed: {speed}, Resume: {is_resume})...")
    
    mission_items = []
    
    if is_resume:
        # Restore EXACTLY what was saved
        mission_items = waypoints
    else:
        # Construct Mission List from list of dicts
        # If speed is set, add DO_CHANGE_SPEED command first
        if speed and speed > 0:
            mission_items.append({
                'type': 'SPEED',
                'speed': speed
            })
            
        # Add waypoints
        for wp in waypoints:
            try:
                mission_items.append({
                    'type': 'WAYPOINT',
                    'lat': float(wp['lat']),
                    'lng': float(wp['lng']),
                    'alt': float(alt)
                })
            except: pass
        
    MISSION_POINTS = mission_items # Store for Protocol
    save_mission_state() # PERSIST MISSION
    
    MISSION_UPLOAD_IN_PROGRESS = True
    AUTO_START_MISSION = auto_start  # Store whether to auto-start after upload
    
    print(f"   Target System: {drone.target_system}, Component: {drone.target_component}")
    print("   Clearing existing mission...")
    drone.mav.mission_clear_all_send(drone.target_system, drone.target_component)
    time.sleep(0.5)  # Small delay for clear to process
    # Send Count - FC will then request each Wpoint
    drone.mav.mission_count_send(drone.target_system, drone.target_component, len(mission_items))

def send_mission_item(seq):
    if seq < len(MISSION_POINTS):
        item = MISSION_POINTS[seq]
        print(f"   Sending Item #{seq}: {item.get('type')}")
        
        if item['type'] == 'SPEED':
            print(f"   sending SPEED change: {item['speed']} m/s")
            drone.mav.mission_item_int_send(
                drone.target_system, drone.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0, 1, # current, autocontinue
                0, item['speed'], -1, 0, # param1=0(groundspeed), param2=speed, param3=-1(no throttle limit), param4=0
                0, 0, 0 # x,y,z unused
            )
        elif item['type'] == 'WAYPOINT':
            print(f"   sending WP {seq}: {item['lat']}, {item['lng']} Alt:{item['alt']}")
            drone.mav.mission_item_int_send(
                drone.target_system, drone.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1, # current, autocontinue
                0, 0, 0, 0, # params 1-4
                int(item['lat'] * 1e7),
                int(item['lng'] * 1e7),
                int(item['alt']) # Altitude from mission settings
            )
    else:
        print(f"⚠️ Requested item {seq} out of bounds {len(MISSION_POINTS)}")

def handle_takeoff_command(alt):
    """Handle TAKEOFF command with proper sequencing (runs in separate thread)."""
    try:
        # Step 1: Validate GPS position
        print(f"🛫 TAKEOFF sequence starting (alt={alt}m)...")
        
        # Check for GPS, but allow bypass if INDOOR_MODE is True
        for _ in range(10):
            if DRONE_DATA.get('lat', 0) != 0 and DRONE_DATA.get('lon', 0) != 0:
                break
            time.sleep(0.1)
        
        lat = float(DRONE_DATA.get('lat', 0))
        lon = float(DRONE_DATA.get('lon', 0))
        
        if (lat == 0 or lon == 0) and not INDOOR_MODE:
            print("❌ TAKEOFF aborted: No valid GPS position (lat/lon=0)")
            print("   💡 If strictly necessary, enable 'Indoor Mode' to bypass this check.")
            return
        
        if INDOOR_MODE and (lat == 0 or lon == 0):
             print("⚠️ Indoor Mode: Bypassing GPS check...")
        
        # Step 2: Ensure GUIDED mode (CRITICAL - TAKEOFF only works in GUIDED)
        current_mode = DRONE_DATA.get('mode', '')
        if current_mode != 'GUIDED':
            print(f"⚠️ Switching from {current_mode} to GUIDED mode...")
            set_flight_mode('GUIDED')
            time.sleep(1.5)  # Wait for mode change
        
        # Step 3: Ensure armed
        if DRONE_DATA.get('status') != 'ARMED':
            print("⚠️ Not armed; arming before takeoff...")
            send_arm()
            time.sleep(2)  # Wait for arming to complete
        
        # Step 4: Send TAKEOFF command
        print(f"🛫 Sending TAKEOFF to {alt}m at ({lat:.6f}, {lon:.6f})")
        drone.mav.command_long_send(
            drone.target_system, drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,              # confirmation
            0,              # param1: minimum pitch (if airspeed sensor)
            0,              # param2: empty
            0,              # param3: empty
            float('nan'),   # param4: yaw (NaN = use current heading)
            lat,            # param5: latitude
            lon,            # param6: longitude
            alt)            # param7: altitude
    except Exception as e:
        print(f"❌ TAKEOFF Error: {e}")

# --- GEOLOCATION MATH ---
def get_target_gps(drone_lat, drone_lon, alt, heading, x_pixel, y_pixel, img_w, img_h, drone_pitch=0):
    """
    Calculate the precise GPS coordinates of a target on the ground using 
    drone telemetry and pixel position.
    Assumes flat ground.
    Added Drone Pitch compensation for fixed cameras.
    """
    if alt <= 0: return drone_lat, drone_lon # Fallback if on ground
    
    # Camera Params (Approximate for SIYI A8/C12 at 1x Zoom)
    H_FOV = 65.0 # Horizontal FOV in degrees
    V_FOV = 50.0 # Vertical FOV in degrees
    
    # Calculate angular offsets from center
    x_offset_deg = ((x_pixel - (img_w / 2)) / (img_w / 2)) * (H_FOV / 2)
    y_offset_deg = ((y_pixel - (img_h / 2)) / (img_h / 2)) * (V_FOV / 2)
    
    # Gimbal Pitch (Fixed Webcam) + Drone Pitch (critical for fixed cameras)
    # If drone pitches down (negative), camera looks further down (steeper/negative)
    # We want effective pitch relative to HORIZON.
    # Camera is mounted at -45 (down).
    # Drone pitch is typically negative when flying forward.
    # Effective angle = Camera Mount + Drone Pitch
    effective_pitch = CAMERA_PITCH_DEG + drone_pitch
    
    # Target pitch = Effective Pitch - Y_offset
    # We want absolute angle from horizon (postive value)
    target_pitch_deg = abs(effective_pitch + y_offset_deg) 
    
    # Clamp pitch to avoid div by zero (infinity distance at horizon)
    if target_pitch_deg < 5: target_pitch_deg = 5
    if target_pitch_deg > 89: target_pitch_deg = 89
    
    # Ground Distance (d = h * tan(theta))
    ground_dist = alt / math.tan(math.radians(target_pitch_deg))
    
    # Target Bearing (Drone Heading + X Offset)
    target_bearing = (heading + x_offset_deg) % 360
    
    # Geodetic Calculation (Simple flat-earth approximation for short distances)
    R = 6378137.0 # Earth Radius
    
    # Convert to radians
    lat1 = math.radians(drone_lat)
    lon1 = math.radians(drone_lon)
    brng = math.radians(target_bearing)
    
    lat2 = math.asin(math.sin(lat1)*math.cos(ground_dist/R) +
                     math.cos(lat1)*math.sin(ground_dist/R)*math.cos(brng))
                     
    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(ground_dist/R)*math.cos(lat1),
                             math.cos(ground_dist/R)-math.sin(lat1)*math.sin(lat2))
    
    return math.degrees(lat2), math.degrees(lon2)

# --- DETECTION AND DUPLICATION LOGIC ---
def is_duplicate_detection(lat, lon):
    """Check if this location was recently detected - strict filtering to prevent duplicates."""
    global DETECTIONS, LAST_DETECTION_TIME
    
    # Rule 1: Global cooldown - minimum time between ANY detections
    time_since_last = time.time() - LAST_DETECTION_TIME
    if time_since_last < DETECTION_COOLDOWN:
        print(f"⏱️ Duplicate blocked by cooldown ({time_since_last:.1f}s < {DETECTION_COOLDOWN}s)")
        return True
    
    # Rule 2: Distance check - ensure new detection is far enough from ALL previous detections
    for detection in DETECTIONS:
        dist = calculate_distance(lat, lon, detection['lat'], detection['lon'])
        if dist < DETECTION_MIN_DISTANCE:
            print(f"📏 Duplicate blocked by distance ({dist:.1f}m < {DETECTION_MIN_DISTANCE}m) - Detection #{detection.get('id', '?')}")
            return True
    
    # Rule 3: Exact location check - prevent same GPS coordinate being detected twice
    for detection in DETECTIONS:
        if abs(detection['lat'] - lat) < 0.00001 and abs(detection['lon'] - lon) < 0.00001:
            print(f"🎯 Duplicate blocked by exact location match - Detection #{detection.get('id', '?')}")
            return True
    
    return False

def mark_detection(target_lat, target_lon, source="rgb", conf=0.0):
    """Record detection with current GPS coordinates."""
    global DETECTIONS, LAST_DETECTION_TIME, DETECTION_ID_COUNTER
    
    # Check for duplicates with strict filtering
    if is_duplicate_detection(target_lat, target_lon):
        print("⚠️ Detection REJECTED - Duplicate location or too soon")
        return False
    
    # Generate unique ID for this detection
    DETECTION_ID_COUNTER += 1
    detection_id = DETECTION_ID_COUNTER
    
    # Record detection with metadata
    detection = {
        "id": detection_id,
        "lat": target_lat, # Use PROJECTED lat
        "lon": target_lon, # Use PROJECTED lon
        "drone_lat": DRONE_DATA.get('lat', 0), 
        "drone_lon": DRONE_DATA.get('lon', 0),
        "conf": conf,
        "source": source,
        "timestamp": time.time(),
        "type": "human",
        "alt": DRONE_DATA.get('alt', 0)
    }
    DETECTIONS.append(detection)
    DRONE_DATA["detections_count"] = len(DETECTIONS)
    LAST_DETECTION_TIME = time.time()
    
    # Publish to MQTT
    print(f"✅ DETECTION #{detection_id} MARKED!")
    print(f"   Target: ({target_lat:.6f}, {target_lon:.6f})")
    
    payload = json.dumps(detection)
    for c in mqtt_clients:
        if c.is_connected(): c.publish(TOPIC_DETECTIONS, payload)
    
    return True

# --- YOLO SETUP ---
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    print("⚠️ Ultralytics/YOLO not found. Video Only.")
    YOLO_AVAILABLE = False

# --- STRONGSORT TRACKING SETUP (Scout Plan) ---
STRONGSORT_AVAILABLE = False
tracker_rgb = None
tracker_thermal = None

try:
    from boxmot import StrongSORT # type: ignore
    STRONGSORT_AVAILABLE = True
    print("✅ StrongSORT tracking available")
except ImportError:
    print("⚠️ boxmot not found. Falling back to simple detection.")
    print("   Install with: pip install boxmot lap")

# Track state management
CONFIRMED_TRACKS = {}  # {track_id: {"hits": N, "gps": (lat, lon), "helmet": bool, "source": str}}
THERMAL_CANDIDATES = {}  # Thermal-detected tracks waiting for RGB confirmation
RGB_CONFIRMED_IDS = set()  # Track IDs confirmed by RGB with helmet

def init_tracker(device='cpu'):
    """Initialize StrongSORT tracker."""
    if not STRONGSORT_AVAILABLE:
        return None
    try:
        tracker = StrongSORT(
            model_weights=Path('osnet_x0_25_msmt17.pt'),  # Lightweight ReID model
            device=device,
            fp16=False,  # No FP16 on CPU
            max_age=TRACK_MAX_AGE,
            n_init=3,  # Frames before track is confirmed
        )
        print(f"🎯 StrongSORT Tracker initialized on {device}")
        return tracker
    except Exception as e:
        print(f"⚠️ StrongSORT init failed: {e}")
        return None

def detect_yellow_helmet(frame, bbox):
    """
    Detect yellow helmet in the head region of a person bounding box.
    Returns True if yellow helmet detected, False otherwise.
    """
    if not HELMET_DETECTION_ENABLED:
        return False
    
    try:
        x1, y1, x2, y2 = map(int, bbox)
        
        # Ensure valid coordinates
        h, w = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        
        # Extract head region (top 30% of bounding box)
        head_height = int((y2 - y1) * 0.3)
        head_region = frame[y1:y1+head_height, x1:x2]
        
        if head_region.size == 0:
            return False
        
        # Convert to HSV and detect yellow
        hsv = cv2.cvtColor(head_region, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HELMET_HSV_LOW, HELMET_HSV_HIGH)
        yellow_pixels = cv2.countNonZero(mask)
        
        return yellow_pixels >= HELMET_MIN_PIXELS
    except Exception as e:
        return False

def is_track_confirmed(track_id):
    """Check if a track has been confirmed (seen for enough frames)."""
    if track_id not in CONFIRMED_TRACKS:
        return False
    return CONFIRMED_TRACKS[track_id]["hits"] >= TRACK_CONFIRMATION_FRAMES

def update_track_state(track_id, lat, lon, helmet, source):
    """Update or create track state."""
    global CONFIRMED_TRACKS
    
    if track_id not in CONFIRMED_TRACKS:
        CONFIRMED_TRACKS[track_id] = {
            "hits": 1,
            "gps": (lat, lon),
            "helmet": helmet,
            "source": source,
            "marked": False  # Whether GPS marker was created
        }
    else:
        CONFIRMED_TRACKS[track_id]["hits"] += 1
        CONFIRMED_TRACKS[track_id]["gps"] = (lat, lon)  # Update GPS
        if helmet:
            CONFIRMED_TRACKS[track_id]["helmet"] = True  # Keep helmet=True once detected

def load_yolo_model():
    if not YOLO_AVAILABLE: return None
    try:
        print("🧠 Loading AI Model (YOLOv8m for Jetson Orin)...")
        model = YOLO('yolov8m.pt') 
        if hasattr(model, 'overrides'): model.overrides['imgsz'] = 1280
        return model
    except Exception as e:
        print(f"❌ AI Init Error: {e}")
        return None

def process_rgb_frame(frame, model, tracker=None, width=1920, height=1080):
    """
    Process RGB frame with StrongSORT tracking and helmet detection.
    Scout Plan: Persistent IDs, Yellow Helmet Detection, Track Confirmation.
    """
    global CONFIRMED_TRACKS, RGB_CONFIRMED_IDS
    
    if model is None: 
        return frame
    
    inference_frame = frame.copy()
    det_conf = DETECTION_CONFIDENCE
    
    try:
        results = model(inference_frame, imgsz=1280, verbose=False)
        
        # Collect all person detections for tracker
        detections = []
        for result in results:
            for box in result.boxes:
                if int(box.cls[0]) == 0 and float(box.conf[0]) >= det_conf:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    detections.append([x1, y1, x2, y2, conf, 0])  # cls=0 for person
        
        # Use StrongSORT tracker if available
        if tracker is not None and len(detections) > 0:
            dets_array = np.array(detections)
            tracks = tracker.update(dets_array, inference_frame)
            
            for track in tracks:
                x1, y1, x2, y2, track_id = int(track[0]), int(track[1]), int(track[2]), int(track[3]), int(track[4])
                conf = track[5] if len(track) > 5 else 0.5
                
                # Detect yellow helmet
                has_helmet = detect_yellow_helmet(frame, (x1, y1, x2, y2))
                
                # Calculate GPS
                cx, cy = (x1+x2)/2, (y1+y2)/2
                t_lat, t_lon = get_target_gps(
                    DRONE_DATA['lat'], DRONE_DATA['lon'], DRONE_DATA['alt'], DRONE_DATA['heading'],
                    cx, cy, width, height
                )
                
                # Update track state
                update_track_state(track_id, t_lat, t_lon, has_helmet, "rgb")
                
                # Draw bounding box with track ID
                color = (0, 255, 255) if has_helmet else (0, 255, 0)  # Yellow if helmet, Green otherwise
                cv2.rectangle(inference_frame, (x1, y1), (x2, y2), color, 2)
                
                # Status text
                helmet_str = "🎓" if has_helmet else ""
                confirmed_str = "✓" if is_track_confirmed(track_id) else ""
                label = f'ID:{track_id} {helmet_str}{confirmed_str} {conf*100:.0f}%'
                cv2.putText(inference_frame, label, (x1, y1-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                # Mark detection ONLY when track is confirmed AND (has helmet OR fusion disabled)
                if DRONE_DATA['lat'] != 0 and is_track_confirmed(track_id):
                    track_data = CONFIRMED_TRACKS.get(track_id, {})
                    
                    # Fusion mode: Only mark if helmet detected OR fusion disabled
                    should_mark = False
                    if FUSION_MODE:
                        if has_helmet or track_data.get("helmet", False):
                            should_mark = True
                            RGB_CONFIRMED_IDS.add(track_id)
                    else:
                        should_mark = True
                    
                    if should_mark and not track_data.get("marked", False):
                        is_new = mark_detection(t_lat, t_lon, "rgb_tracked", conf)
                        if is_new:
                            CONFIRMED_TRACKS[track_id]["marked"] = True
                            cv2.putText(inference_frame, "NEW TARGET", (x1, y1-30), 0, 0.7, (0,255,255), 2)
                
                # Overlay GPS
                cv2.putText(inference_frame, f"{t_lat:.5f},{t_lon:.5f}", (x1, y2+15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)
        
        else:
            # Fallback: Simple detection without tracking
            for det in detections:
                x1, y1, x2, y2, conf, _ = map(int, det[:4]) + [det[4], det[5]]
                x1, y1, x2, y2 = int(det[0]), int(det[1]), int(det[2]), int(det[3])
                conf = det[4]
                
                cv2.rectangle(inference_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(inference_frame, f'Human {conf*100:.0f}%', (x1, y1-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                cx, cy = (x1+x2)/2, (y1+y2)/2
                t_lat, t_lon = get_target_gps(
                    DRONE_DATA['lat'], DRONE_DATA['lon'], DRONE_DATA['alt'], DRONE_DATA['heading'],
                    cx, cy, width, height
                )
                
                if DRONE_DATA['lat'] != 0:
                    is_new = mark_detection(t_lat, t_lon, "rgb", conf)
                    if is_new:
                        cv2.putText(inference_frame, "NEW TARGET", (x1, y1-30), 0, 0.5, (0,255,255), 2)
                
                cv2.putText(inference_frame, f"{t_lat:.5f},{t_lon:.5f}", (x1, y1+20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
        
        return inference_frame
    except Exception as e:
        print(f"RGB Infer Err: {e}")
        return frame


def process_thermal_frame(frame, model, tracker=None, width=640, height=512):
    """
    Process Thermal frame with StrongSORT tracking.
    Scout Plan: Primary detection source for heat signatures.
    """
    global THERMAL_CANDIDATES
    
    if model is None: 
        return frame
    
    # Thermal Optimization: Invert for better YOLO detection
    inference_frame = cv2.bitwise_not(frame)
    det_conf = 0.25  # Lower threshold for thermal
    display_frame = frame.copy()
    
    try:
        results = model(inference_frame, imgsz=1280, verbose=False)
        
        # Collect all person detections
        detections = []
        for result in results:
            for box in result.boxes:
                if int(box.cls[0]) == 0 and float(box.conf[0]) >= det_conf:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    detections.append([x1, y1, x2, y2, conf, 0])
        
        # Use StrongSORT tracker if available
        if tracker is not None and len(detections) > 0:
            dets_array = np.array(detections)
            tracks = tracker.update(dets_array, inference_frame)
            
            for track in tracks:
                x1, y1, x2, y2, track_id = int(track[0]), int(track[1]), int(track[2]), int(track[3]), int(track[4])
                conf = track[5] if len(track) > 5 else 0.5
                
                # Calculate GPS
                cx, cy = (x1+x2)/2, (y1+y2)/2
                t_lat, t_lon = get_target_gps(
                    DRONE_DATA['lat'], DRONE_DATA['lon'], DRONE_DATA['alt'], DRONE_DATA['heading'],
                    cx, cy, width, height
                )
                
                # Update thermal candidates (for fusion with RGB)
                THERMAL_CANDIDATES[track_id] = {
                    "gps": (t_lat, t_lon),
                    "conf": conf,
                    "timestamp": time.time()
                }
                
                # Draw on display frame (original, not inverted)
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                
                confirmed_str = "✓" if is_track_confirmed(track_id) else ""
                label = f'T-ID:{track_id} {confirmed_str} {conf*100:.0f}%'
                cv2.putText(display_frame, label, (x1, y1-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # In non-fusion mode, mark thermal detections directly
                if not FUSION_MODE and DRONE_DATA['lat'] != 0 and is_track_confirmed(track_id):
                    is_new = mark_detection(t_lat, t_lon, "thermal_tracked", conf)
                    if is_new:
                        cv2.putText(display_frame, "NEW TARGET", (x1, y1-25), 0, 0.5, (0,255,255), 2)
                
                cv2.putText(display_frame, f"{t_lat:.5f},{t_lon:.5f}", (x1, y2+12), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)
        
        else:
            # Fallback: Simple detection without tracking
            for det in detections:
                x1, y1, x2, y2 = int(det[0]), int(det[1]), int(det[2]), int(det[3])
                conf = det[4]
                
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(display_frame, f'Human {conf*100:.0f}%', (x1, y1-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                cx, cy = (x1+x2)/2, (y1+y2)/2
                t_lat, t_lon = get_target_gps(
                    DRONE_DATA['lat'], DRONE_DATA['lon'], DRONE_DATA['alt'], DRONE_DATA['heading'],
                    cx, cy, width, height
                )
                
                if DRONE_DATA['lat'] != 0:
                    is_new = mark_detection(t_lat, t_lon, "thermal", conf)
                    if is_new:
                        cv2.putText(display_frame, "NEW TARGET", (x1, y1-30), 0, 0.5, (0,255,255), 2)
                
                cv2.putText(display_frame, f"{t_lat:.5f},{t_lon:.5f}", (x1, y1+20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
        
        return display_frame
    except Exception as e:
        print(f"Thermal Infer Err: {e}")
        return frame

# --- CAMERA THREADS ---
class DummyCapture:
    def __init__(self, color=(100,100,100), w=1920, h=1080, name="NO CAM"): 
        self.color = color; self.w=w; self.h=h; self.name=name
    def isOpened(self): return True
    def read(self):
        img = np.zeros((self.h, self.w, 3), np.uint8)
        img[:] = self.color
        cv2.putText(img, self.name, (100, 100), 0, 2, (255,255,255), 3)
        time.sleep(0.1)
        return True, img
    def release(self): pass

def run_camera_rgb():
    global frame_rgb, model
    print(f"📷 Init RGB Camera ({CAM_ID_RGB})...")
    # model = load_yolo_model() # Use global model
    
    # Initialize StrongSORT tracker for RGB (Scout Plan)
    tracker = init_tracker(device='cpu')
    if tracker:
        print("🎯 RGB StrongSORT Tracker Ready")
    
    def open_cam():
        print(f"📷 Attempting to open RGB RTSP Stream: {CAM_ID_RGB}")
        # Use FFmpeg explicitly as it's more reliable for RTSP in standard OpenCV builds
        cap = cv2.VideoCapture(CAM_ID_RGB, cv2.CAP_FFMPEG)
        # Set buffer size small to reduce latency
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap
    
    cap = open_cam()
    if not cap.isOpened(): cap = DummyCapture(name="NO RGB")
    
    while True:
        ret, frame = cap.read()
        if ret:
            with lock_rgb: frame_rgb = process_rgb_frame(frame, model, tracker, 1920, 1080)
        else:
            print("⚠️ RGB Stream Lost... Reconnecting...")
            cap.release(); time.sleep(2); cap = open_cam()
            if not cap.isOpened(): time.sleep(1)

def run_camera_thermal():
    global frame_thermal, model
    print(f"📷 Init Thermal Camera ({CAM_ID_THERMAL})...")
    # model = load_yolo_model() # Use global model
    
    # Initialize StrongSORT tracker for Thermal (Scout Plan)
    tracker = init_tracker(device='cpu')
    if tracker:
        print("🎯 Thermal StrongSORT Tracker Ready")
    
    # Thermal often 640x480 or 640x512
    def open_cam():
        print(f"📷 Attempting to open Thermal RTSP Stream: {CAM_ID_THERMAL}")
        cap = cv2.VideoCapture(CAM_ID_THERMAL, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    cap = open_cam()
    if not cap.isOpened(): cap = DummyCapture(color=(0,0,100), w=640, h=480, name="NO THERMAL")
    
    while True:
        ret, frame = cap.read()
        if ret:
            h, w = frame.shape[:2]
            with lock_thermal: frame_thermal = process_thermal_frame(frame, model, tracker, w, h)
        else:
            print("⚠️ Thermal Stream Lost... Reconnecting...")
            cap.release(); time.sleep(2); cap = open_cam()
            if not cap.isOpened(): time.sleep(1)

def run_camera_webcam():
    global frame_webcam, model
    # Check if webcam device exists to avoid spamming warnings
    webcam_dev = f"/dev/video{CAM_ID_WEBCAM}" if isinstance(CAM_ID_WEBCAM, int) else CAM_ID_WEBCAM
    if not os.path.exists(webcam_dev) and isinstance(CAM_ID_WEBCAM, int):
        print(f"⚠️ USB Webcam not found at {webcam_dev}. Skipping webcam thread.")
        return

    print(f"📷 Init USB Webcam ({CAM_ID_WEBCAM})...")
    cap = cv2.VideoCapture(CAM_ID_WEBCAM)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    while True:
        ret, frame = cap.read()
        if ret:
            # Run YOLO Detection if available
            if YOLO_AVAILABLE and model is not None:
                try:
                    # Run inference
                    results = model(frame, verbose=False, conf=0.4, iou=0.5)
                    
                    # Annotate and Extract
                    for result in results:
                        frame = result.plot() # Draw boxes
                        
                        for box in result.boxes:
                            # Extract Box
                            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                            conf = float(box.conf[0])
                            cls = int(box.cls[0])
                            
                            # Only Humans (Class 0)
                            if cls == 0 and conf > 0.4:
                                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                                h, w, _ = frame.shape
                                
                                # Calculate GPS Location
                                t_lat, t_lon = get_target_gps(
                                    DRONE_DATA['lat'], DRONE_DATA['lon'], DRONE_DATA['alt'], DRONE_DATA['heading'],
                                    cx, cy, w, h, DRONE_DATA.get('pitch', 0)
                                )
                                
                                # Draw GPS on Frame
                                cv2.putText(frame, f"{t_lat:.5f}, {t_lon:.5f}", (x1, y2 + 20), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

                                # Mark detection (Save to Global List)
                                # Only if drone has valid GPS
                                if DRONE_DATA['lat'] != 0:
                                    is_new = mark_detection(t_lat, t_lon, "webcam", conf)
                                    if is_new:
                                        print(f"🎯 NEW WEBCAM TARGET: {t_lat}, {t_lon}")
                                        cv2.putText(frame, "NEW TARGET", (x1, y1-30), 0, 0.6, (0,255,0), 2)

                except Exception as e: 
                    # print(f"Webcam Detection Err: {e}") 
                    pass
            
            with lock_webcam: frame_webcam = frame
        else:
            time.sleep(0.5); cap.release(); cap = cv2.VideoCapture(CAM_ID_WEBCAM)
            cap.release(); time.sleep(2); cap = cv2.VideoCapture(CAM_ID_WEBCAM)
            if not cap.isOpened(): time.sleep(1)

# --- FLASK STREAMING ---
def generate_frames(is_thermal=False):
    global frame_rgb, frame_thermal, frame_webcam
    while True:
        ret = False
        if is_thermal == "webcam":
            with lock_webcam:
                if frame_webcam is not None:
                    ret, buffer = cv2.imencode('.jpg', frame_webcam)
                else:
                    ret = False
        elif is_thermal:
            with lock_thermal:
                if frame_thermal is not None:
                    ret, buffer = cv2.imencode('.jpg', frame_thermal)
                else:
                    ret = False
        else:
            with lock_rgb:
                if frame_rgb is not None:
                    ret, buffer = cv2.imencode('.jpg', frame_rgb)
                else:
                    ret = False
        
        if ret:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        else:
            time.sleep(0.05)

@app.route('/video_feed')
def video_feed_rgb():
    print(f"📡 Video Feed Requested")
    return Response(generate_frames(False), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/thermal_feed')
def video_feed_thermal():
    return Response(generate_frames(True), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/webcam_feed')
def video_feed_webcam():
    return Response(generate_frames("webcam"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/get_delivery_mission')
def get_delivery_mission():
    """Returns detected survivors as a Mission Plan for Delivery Drone"""
    print("📦 Generating Delivery Mission from Detections...")
    mission_items = []
    
    # Sort detections by time? Or just unique locations?
    # DETECTIONS list contains {lat, lon, id, ...}
    
    for i, det in enumerate(DETECTIONS):
        # Create a waypoint for each detection (Hover above target)
        item = {
            "seq": i,
            "lat": det['lat'],
            "lng": det['lon'],
            "alt": 15, # Delivery Hover Alt
            "cmd": "WAYPOINT"
        }
        mission_items.append(item)
    
    # Add RTL at the end
    mission_items.append({
        "seq": len(mission_items),
        "lat": 0, "lng": 0, "alt": 0, "cmd": "RTL"
    })
    
    return jsonify({"count": len(DETECTIONS), "waypoints": mission_items})

def run_flask():
    print(f"🎥 Starting Video Server on port {FLASK_PORT}...")
    # Enable threaded mode to handle multiple stream requests (dashboard + vision page)
    app.run(host='0.0.0.0', port=FLASK_PORT, debug=False, use_reloader=False, threaded=True)

# --- MQTT SETUP ---
def on_message(client, userdata, msg):
    try:
        # print(f"📩 MQTT RAW: {msg.payload}") # Silence raw spam
        data = json.loads(msg.payload.decode())
        
        if data['type'] == "COMMAND":
            act = data.get('act')
            if act != "RC_OVERRIDE": # Reduce log spam
                print(f"⚙️ EXECUTING COMMAND: {act}")
                
            if act == "ARM": 
                def arm_sequence():
                    # 1. Force RC Override Enabled
                    print("🛡️ Preparing to Arm: Enforcing Low Throttle...")
                    # Even if active, force channels to safe defaults
                    global RC_OVERRIDE_ACTIVE, RC_OVERRIDE_CHANNELS
                    RC_OVERRIDE_CHANNELS = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000] # Ch3=1000
                    RC_OVERRIDE_ACTIVE = True 
                    
                    if DRONE_DATA.get('lat', 0) != 0:
                        DRONE_DATA['home_lat'] = DRONE_DATA['lat']
                        DRONE_DATA['home_lon'] = DRONE_DATA['lon']

                    # 2. Wait a moment for overrides to stick (Crucial for 'Throttle not neutral' error)
                    time.sleep(1.0)

                    # 3. Direct Arm
                    send_arm()
                
                threading.Thread(target=arm_sequence, daemon=True).start()

            elif act == "DISARM": send_disarm()
            elif act == "LAND": set_flight_mode('LAND')
            elif act == "RTL": set_flight_mode('RTL')
            elif act == "TAKEOFF": threading.Thread(target=handle_takeoff_command, args=(float(data.get('alt', 10)),), daemon=True).start()
            elif act == "INDOOR_MODE": set_indoor_mode()
            elif act == "START_RC_OVERRIDE": start_rc_override()
            elif act == "STOP_RC_OVERRIDE": stop_rc_override()
            elif act == "RC_OVERRIDE": override_rc(data.get('channels', []))
            
            elif act == "SET_MODE": drone.set_mode(data.get('mode'))
            elif act == "REBOOT_FC": drone.reboot_autopilot()
            
            elif act == "RESUME_MISSION":
                last_wp = load_mission_state()
                if len(MISSION_POINTS) > 0 and last_wp >= 0:
                    DRONE_DATA['current_wp'] = last_wp
                    upload_mission_to_fc(MISSION_POINTS, auto_start=False, is_resume=True)
                    time.sleep(1)
                    drone.mav.mission_set_current_send(drone.target_system, drone.target_component, last_wp)
                    time.sleep(1)
                    if DRONE_DATA['status'] != 'ARMED': send_arm(); time.sleep(2)
                    drone.set_mode('AUTO')
                    print(f"🔄 MISSION RESUMED from WP {last_wp}")

        elif data['type'] == "MISSION_UPLOAD":
            print(f"📩 RX MISSION_UPLOAD")
            waypoints = data.get('waypoints', data.get('points'))
            if waypoints:
                # Map both 'speed' and 'speed_limit' keys, cast to float
                raw_speed = data.get('speed', data.get('speed_limit', 5))
                try:
                    speed = float(raw_speed)
                except:
                    speed = 5.0
                
                alt = float(data.get('altitude', data.get('alt', 20)))
                upload_mission_to_fc(waypoints, alt=alt, speed=speed)
            
    except Exception as e: print(f"MQTT Error: {e}")

def setup_mqtt():
    for ip in BROKER_IPS:
        try:
            c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            c.on_message = on_message
            def on_connect_wrapper(client, userdata, flags, rc, properties=None):
                if rc == 0: 
                    client.subscribe(TOPIC_MISSION)
                    print(f"✅ MQTT Connected & Subscribed to {ip}")
                else: print(f"❌ MQTT Connect Fail {rc}")
            c.on_connect = on_connect_wrapper
            c.connect_async(ip, 1883, 60)
            c.loop_start()
            mqtt_clients.append(c)
        except Exception as e: print(f"MQTT Init Fail {ip}: {e}")

def publish_survivor_locations():
    """Publishes all detected survivor locations to the delivery drone."""
    if not DETECTIONS:
        print("⚠️ No survivors detected to publish.")
        return

    # Filter unique locations (simple rounding deduplication)
    unique_points = []
    seen = set()
    for d in DETECTIONS:
        key = (round(d['lat'], 5), round(d['lon'], 5))
        if key not in seen:
            seen.add(key)
            unique_points.append({"lat": d['lat'], "lon": d['lon']})

    payload = {
        "type": "SURVIVOR_LOCATIONS",
        "points": unique_points,
        "altitude": 6.5
    }
    
    # Publish to all clients
    msg = json.dumps(payload)
    for c in mqtt_clients:
        if c.is_connected():
            c.publish(TOPIC_SURVIVOR, msg)
    
    print(f"✅ PUBLISHED {len(unique_points)} SURVIVOR LOCATIONS TO DELIVERY DRONE")

# --- MAIN LOOP ---
if __name__ == "__main__":
    setup_mqtt()
    
    # Send Battery Fix on Startup
    # Battery fix param removed (undefined function)

    # Load Model ONCE globally
    print("🧠 Loading AI Model (YOLOv8s for Raspberry Pi 5 - CPU)...")
    try:
        # Upgraded to yolov8s.pt (Small) instead of nano for better accuracy
        model = YOLO("yolov8s.pt")  
        print("✅ Model Loaded Successfully (CPU Mode)")
    except Exception as e:
        print(f"❌ Model Load Failed: {e}")
        model = None
    
    # Start Camera/Webserver threads
    threading.Thread(target=run_flask, daemon=True).start()
    if ENABLE_RGB: threading.Thread(target=run_camera_rgb, daemon=True).start()
    if ENABLE_THERMAL: threading.Thread(target=run_camera_thermal, daemon=True).start()
    if ENABLE_WEBCAM: threading.Thread(target=run_camera_webcam, daemon=True).start()
    
    # Init Gimbal (Only if using main cameras)
    if ENABLE_RGB or ENABLE_THERMAL:
        threading.Thread(target=skybroid_gimbal.set_search_angle, daemon=True).start()

    # Enforce Battery Monitor
    threading.Thread(target=ensure_battery_monitor, daemon=True).start()

    last_telem_send = time.time()
    
    print("✅ System Online - Entering Main Loop")
    
    while True:
        try:
            msg = drone.recv_match(blocking=False)
            if msg:
                mtype = msg.get_type()
                
                if mtype == 'HEARTBEAT':
                    # FILTER: Only accept Heartbeats from Autopilot (Comp 1)
                    # Gimbals/Cameras often send "Disarmed" heartbeats which cause flickering
                    if msg.get_srcComponent() != 1:
                        continue

                    LAST_HEARTBEAT = time.time()
                    new_status = "ARMED" if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "DISARMED"
                    if new_status != DRONE_DATA.get('status'):
                        print(f"🔄 STATUS CHANGE: {DRONE_DATA.get('status')} -> {new_status}")
                    DRONE_DATA['status'] = new_status
                    DRONE_DATA['mode'] = COPTER_MODES.get(msg.custom_mode, str(msg.custom_mode))
                    
                elif mtype == 'GLOBAL_POSITION_INT':
                    DRONE_DATA['lat'] = msg.lat / 1e7
                    DRONE_DATA['lon'] = msg.lon / 1e7
                    DRONE_DATA['alt'] = msg.relative_alt / 1000.0
                    # Set home if not set
                    if DRONE_DATA['home_lat'] == 0 and DRONE_DATA['lat'] != 0:
                         DRONE_DATA['home_lat'] = DRONE_DATA['lat']
                         DRONE_DATA['home_lon'] = DRONE_DATA['lon']
                    
                    # Run Failsafe Calc
                    calculate_smart_rtl()
                    
                elif mtype == 'SYS_STATUS':
                    v_raw = msg.voltage_battery / 1000.0
                    # Voltage Correction Logic:
                    # User feedback: "Telemetry shows correct voltage when not flying" -> 0V correction when Disarmed.
                    # "Always +2" implies need for +2V correction under load/flying condition.
                    correction = 2.0 if DRONE_DATA['status'] == 'ARMED' else 0.0
                    v = v_raw + correction
                    
                    # Clamp to ensure no negative voltage
                    v = max(0.0, v)

                    DRONE_DATA['bat_voltage'] = v
                    DRONE_DATA['bat_current'] = msg.current_battery / 100.0
                    
                    # Debug Print (Once every 5 seconds)
                    if int(time.time()) % 5 == 0 and time.time() % 1 < 0.2:
                         print(f"⚡ SYS_STATUS: Raw={v_raw:.2f}V Corrected={v:.2f}V ({correction:+.1f}V), {DRONE_DATA['bat_current']:.2f}A")

                    if msg.battery_remaining == -1:
                        DRONE_DATA['bat'] = voltage_to_percent(v)
                    else:
                        DRONE_DATA['bat'] = msg.battery_remaining
                
                elif mtype == 'BATTERY_STATUS':
                     # Alternative Battery Message (often used for Smart Batteries)
                     v = msg.voltages[0] / 1000.0 # First cell bank voltage
                     if v > 0:
                         DRONE_DATA['bat_voltage'] = v
                         DRONE_DATA['bat_current'] = msg.current_battery / 100.0
                         DRONE_DATA['bat'] = msg.battery_remaining
                         if int(time.time()) % 5 == 0 and time.time() % 1 < 0.2:
                             print(f"🔋 BATTERY_STATUS DEBUG: {v:.2f}V, {msg.current_battery}cA, {msg.battery_remaining}%")

                elif mtype == 'VFR_HUD':
                    DRONE_DATA['speed'] = round(msg.groundspeed, 1)
                    DRONE_DATA['heading'] = msg.heading
                
                elif mtype == 'GPS_RAW_INT':
                    DRONE_DATA['gps_sats'] = msg.satellites_visible
                    DRONE_DATA['gps_fix'] = msg.fix_type

                elif mtype == 'ATTITUDE':
                    DRONE_DATA['pitch'] = math.degrees(msg.pitch)
                    DRONE_DATA['roll'] = math.degrees(msg.roll)
                
                elif mtype == 'COMMAND_ACK':
                    res_text = MAV_RESULT_NAMES.get(msg.result, str(msg.result))
                    print(f"🔔 ACK Received: Cmd={msg.command} Res={msg.result} ({res_text})")
                    if msg.result != 0:
                         print(f"   ❌ Command {msg.command} failed: {res_text}")
                
                elif mtype == 'STATUSTEXT':
                    print(f"🤖 FC MSG: {msg.text}")
                    if "Failsafe" in msg.text or "Check" in msg.text:
                         print(f"   ⚠️ WARNING: Flight Controller alert might be blocking arming/mission!")

                elif mtype == 'MISSION_ITEM_REACHED':
                    print(f"📍 Waypoint {msg.seq} Reached")
                    DRONE_DATA["current_wp"] = msg.seq
                    save_mission_state() # PERSIST PROGRESS
                    
                    # Check if this is the last waypoint
                    if len(MISSION_POINTS) > 0 and msg.seq >= len(MISSION_POINTS) - 1:
                        print("✅ MISSION COMPLETE: Reached final waypoint.")
                        # Publish detected locations to Delivery Drone
                        publish_survivor_locations()
                        # Return to Launch
                        print("🏠 Returning to Launch (RTL)...")
                        set_flight_mode('RTL')

                elif mtype == 'MISSION_REQUEST':
                    # print(f"   FC Requesting WP {msg.seq}") # Verbose
                    send_mission_item(msg.seq)
                
                elif mtype == 'MISSION_ACK':
                    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        print("✅ MISSION ACCEPTED")
                        MISSION_UPLOAD_IN_PROGRESS = False
                        drone.mav.mission_set_current_send(drone.target_system, drone.target_component, 0)
                        
                        if AUTO_START_MISSION:
                             print("🚀 Auto-starting mission sequence...")
                             # Step 1: Switch to GUIDED for arming/takeoff (usually more robust)
                             set_flight_mode('GUIDED')
                             time.sleep(1)
                             
                             # Step 2: Arm
                             if DRONE_DATA['status'] != 'ARMED':
                                 send_arm()
                                 time.sleep(2)
                             
                             # Step 3: Switch to AUTO to start execution
                             print("🚀 Switching to AUTO to begin mission...")
                             drone.set_mode('AUTO')
                             AUTO_START_MISSION = False
                    else:
                        print(f"❌ MISSION FAILED: {msg.type}")
                        
            # Smart Battery Failsafe Trigger
            if (DRONE_DATA['mode'] in ['GUIDED', 'AUTO', 'LOITER']) and \
               (DRONE_DATA['bat'] < DRONE_DATA['req_rtl_batt']) and \
               (DRONE_DATA['bat'] > 0) and (DRONE_DATA['req_rtl_batt'] > 0):
                 print(f"⚠️ SMART FAILSAFE TRIGGERED: {DRONE_DATA['bat']}% < {DRONE_DATA['req_rtl_batt']}%")
                 drone.set_mode('RTL')

            # Telemetry Broadcast
            if time.time() - last_telem_send >= 0.25:
                last_telem_send = time.time()
                payload = json.dumps(DRONE_DATA)
                for c in mqtt_clients:
                     if c.is_connected(): c.publish(TOPIC_TELEM, payload)
                
                if int(time.time()) % 2 == 0 and time.time() % 1 < 0.25:
                     print(f"📡 Sats:{DRONE_DATA.get('gps_sats')} Bat:{DRONE_DATA.get('bat')}% Mode:{DRONE_DATA.get('mode')} Status:{DRONE_DATA.get('status')} Fix:{DRONE_DATA.get('gps_fix')}")

            time.sleep(0.001)

        except Exception as e:
            # print(f"Loop Error: {e}")
            time.sleep(0.1)
