import cv2
import time
import json
import threading
import argparse
import sys
import os
import numpy as np
from flask import Flask, Response
import paho.mqtt.client as mqtt
from pymavlink import mavutil
from ultralytics import YOLO

# Import our custom geolocation math
try:
    from geo_math import calculate_gps_from_pixel, calculate_distance_to_target
except ImportError:
    # Fallback if running from a different context
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from geo_math import calculate_gps_from_pixel, calculate_distance_to_target

# --- CONFIGURATION ---
MQTT_BROKER = "100.125.45.22" # Corrected Laptop IP
MQTT_PORT = 1883
TOPIC_DETECTIONS = "nidar/scout/detections"

MAVLINK_CONNECTION = "udpin:0.0.0.0:14551" # Output 2 from Mavlink Router

MODEL_PATH = "scout_human_yolo.pt"

# Camera Config 
CAMERA_RGB_ID = 0
CAMERA_THERMAL_ID = 1 

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FLASK_PORT = 5001

# Global State
frame_rgb = None
frame_thermal = None
lock_rgb = threading.Lock()
lock_thermal = threading.Lock()

drone_telemetry = {
    'lat': 0.0,
    'lon': 0.0,
    'alt': 10.0,
    'heading': 0.0,
    'connected': False
}

# --- FLASK APP ---
app = Flask(__name__)

def generate_frames(is_thermal=False):
    global frame_rgb, frame_thermal
    while True:
        frame = None
        if is_thermal:
            with lock_thermal:
                if frame_thermal is not None:
                    # Encode frame
                    ret, buffer = cv2.imencode('.jpg', frame_thermal)
                    if ret: frame = buffer.tobytes()
        else:
             with lock_rgb:
                if frame_rgb is not None:
                    # Encode frame
                    ret, buffer = cv2.imencode('.jpg', frame_rgb)
                    if ret: frame = buffer.tobytes()
        
        if frame is None:
            time.sleep(0.05)
            continue
            
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed_rgb():
    return Response(generate_frames(is_thermal=False), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/thermal_feed')
def video_feed_thermal():
    return Response(generate_frames(is_thermal=True), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    print(f"🎥 Starting Video Server on port {FLASK_PORT}")
    app.run(host='0.0.0.0', port=FLASK_PORT, debug=False, use_reloader=False)

# --- MAVLINK THREAD ---
def run_mavlink():
    global drone_telemetry
    print(f"🔌 Connecting to Flight Controller on {MAVLINK_CONNECTION}...")
    
    try:
        connection = mavutil.mavlink_connection(MAVLINK_CONNECTION)
        connection.wait_heartbeat()
        print("✅ FC Connected!")
        drone_telemetry['connected'] = True
        
        while True:
            msg = connection.recv_match(blocking=True, timeout=1.0)
            if not msg:
                continue
            
            msg_type = msg.get_type()
            
            if msg_type == 'GLOBAL_POSITION_INT':
                drone_telemetry['lat'] = msg.lat / 1e7
                drone_telemetry['lon'] = msg.lon / 1e7
                drone_telemetry['alt'] = msg.relative_alt / 1000.0 # Meters
                drone_telemetry['heading'] = msg.hdg / 100.0 # Degrees
            
            elif msg_type == 'VFR_HUD':
                 if 'heading' not in drone_telemetry:
                     drone_telemetry['heading'] = msg.heading
                     
    except Exception as e:
        print(f"❌ Mavlink Error: {e}")

# --- HELPER: Load Model ---
import torch

def load_yolo_model():
    print("🧠 Loading AI Model...")
    
    if torch.cuda.is_available():
        print(f"✅ CUDA Available: {torch.cuda.get_device_name(0)}")
    else:
        print("❌ WARNING: CUDA NOT Available. Running on CPU!")

    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    ABS_MODEL_PATH = os.path.join(SCRIPT_DIR, MODEL_PATH)
    
    try:
        model = None
        if os.path.exists(ABS_MODEL_PATH):
            model = YOLO(ABS_MODEL_PATH)
            print(f"✅ Loaded {ABS_MODEL_PATH}")
        elif os.path.exists(MODEL_PATH):
             model = YOLO(MODEL_PATH)
             print(f"✅ Loaded {MODEL_PATH} (from CWD)")
        else:
            print(f"⚠️ Model {ABS_MODEL_PATH} not found, downloading standard YOLOv8n...")
            model = YOLO("yolov8n.pt")
            
        return model
    except Exception as e:
        print(f"❌ AI Init Error: {e}")
        return None

# --- HELPER: Inference & Process ---
def process_frame(frame, model, client, source_type="rgb"):
    if model is None: return frame
    
    # Run Tracking (GPU)
    # Use distinct classes/persistence if needed, but here simple tracking
    results = model.track(frame, persist=True, device=0, classes=[0], conf=0.4, verbose=False)
    
    annotated_frame = results[0].plot()
    
    for r in results:
         boxes = r.boxes
         for box in boxes:
             track_id = int(box.id[0]) if box.id is not None else 0
             conf = float(box.conf[0])
             x1, y1, x2, y2 = box.xyxy[0]
             cx = int((x1 + x2) / 2)
             cy = int((y1 + y2) / 2)
             
             # Geolocation (Shared logic)
             if drone_telemetry['lat'] != 0:
                 intrinsics = {'width': FRAME_WIDTH, 'height': FRAME_HEIGHT, 'hfov': 80.0, 'vfov': 60.0}
                 target_lat, target_lon = calculate_gps_from_pixel(drone_telemetry, (cx, cy), intrinsics)
                 
                 # Calculate Distance
                 dist_info = calculate_distance_to_target(drone_telemetry, target_lat, target_lon)
                 dist_ground = dist_info['distance_ground']
                 
                 payload = {
                     "id": f"{source_type}_{track_id}",
                     "lat": target_lat,
                     "lon": target_lon,
                     "dist_m": dist_ground, # Added Distance
                     "conf": conf,
                     "type": "human",
                     "source": source_type
                 }
                 client.publish(TOPIC_DETECTIONS, json.dumps(payload))
                 
                 cv2.putText(annotated_frame, f"ID:{track_id} D:{dist_ground:.1f}m", 
                             (int(x1), int(y1)-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                 cv2.putText(annotated_frame, f"GPS:{target_lat:.5f},{target_lon:.5f}", 
                             (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    return annotated_frame

# --- HELPER: Dummy Camera ---
class DummyCapture:
    def __init__(self, width, height, color=(0,0,0)):
        self.width = width
        self.height = height
        self.color = color
    
    def isOpened(self):
        return True
    
    def read(self):
        # Create a dummy image (noise or solid color)
        img = np.zeros((self.height, self.width, 3), np.uint8)
        img[:] = self.color
        # Add some tracking noise
        cv2.circle(img, (int(time.time()*50 % self.width), self.height//2), 20, (255, 255, 255), -1)
        time.sleep(0.1) # Simulate 10 FPS
        return True, img
    
    def set(self, prop, val):
        pass

# --- CAMERA CAPTURE THREAD (THERMAL) ---
def run_capture_thermal():
    global frame_thermal
    
    # Init MQTT for Thermal Thread
    client = mqtt.Client()
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
    except: pass

    # Init Model for Thermal (Separate Instance for separate tracking state)
    print("🧠 Loading Thermal Model Instance...")
    model_thermal = load_yolo_model()

    print(f"📷 Opening Thermal Camera ({CAMERA_THERMAL_ID})...")
    cap = cv2.VideoCapture(CAMERA_THERMAL_ID)
    if not cap.isOpened():
        print(f"⚠️ Warning: Thermal Camera {CAMERA_THERMAL_ID} not found. Using DUMMY source.")
        cap = DummyCapture(FRAME_WIDTH, FRAME_HEIGHT, color=(0, 0, 100)) # Dark Red for Thermal
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    
    while True:
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                # Run Inference
                annotated = process_frame(frame, model_thermal, client, "thermal")
                with lock_thermal:
                    frame_thermal = annotated
            else:
                time.sleep(0.1)
        else:
            time.sleep(1)

# --- AI & MAIN LOOP (RGB) ---
def run_main_service():
    global frame_rgb
    
    # Init MQTT
    print(f"📡 Connecting to MQTT Broker {MQTT_BROKER}...")
    client = mqtt.Client()
    try:
        # Reduced timeout for faster startup if offline
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=10) 
        client.loop_start()
        print("✅ MQTT Connected")
    except Exception as e:
        print(f"⚠️ MQTT Connection Failed: {e} (Running in Offline Mode)")
    
    # Init Model for RGB
    model_rgb = load_yolo_model()

    # Open RGB Camera
    print(f"📷 Opening RGB Camera ({CAMERA_RGB_ID})...")
    cap = cv2.VideoCapture(CAMERA_RGB_ID)
    
    if not cap.isOpened():
        print(f"⚠️ Warning: RGB Camera {CAMERA_RGB_ID} not found. Using DUMMY source.")
        cap = DummyCapture(FRAME_WIDTH, FRAME_HEIGHT, color=(50, 50, 50)) # Grey for RGB
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        
    print("🚀 AI Service Running (Dual Stream Detection)...")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ RGB Frame Capture Failed")
            time.sleep(1)
            continue
            
        annotated = process_frame(frame, model_rgb, client, "rgb")
        
        with lock_rgb:
            frame_rgb = annotated

if __name__ == '__main__':
    t_flask = threading.Thread(target=run_flask, daemon=True)
    t_mav = threading.Thread(target=run_mavlink, daemon=True)
    t_therm = threading.Thread(target=run_capture_thermal, daemon=True)
    
    t_flask.start()
    t_mav.start()
    t_therm.start()
    
    run_main_service()
