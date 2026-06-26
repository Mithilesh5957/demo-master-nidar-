from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO
import paho.mqtt.client as mqtt
import json
import os
import time
# Removed eventlet to avoid conflicts with Paho MQTT threads

app = Flask(__name__)
CORS(app)
# Force threading mode for stability
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

BROKER = os.getenv('MQTT_BROKER', 'mqtt')
TOPICS = {
    "scout": "nidar/scout/mission",
    "delivery": "nidar/delivery/mission"
}
TOPIC_SURVIVOR = "nidar/delivery/target"
TOPIC_TELEM_SCOUT = "nidar/scout/telemetry"
TOPIC_TELEM_DELIVERY = "nidar/delivery/telemetry"

# --- MYSQL DATABASE CONNECTION ---
import mysql.connector
from mysql.connector import Error

MYSQL_CONFIG = {
    'host': os.getenv('MYSQL_HOST', 'mysql'),
    'user': os.getenv('MYSQL_USER', 'nidar'),
    'password': os.getenv('MYSQL_PASSWORD', 'nidar_pass'),
    'database': os.getenv('MYSQL_DATABASE', 'nidar_db')
}

db_connection = None

def get_db_connection():
    """Get database connection with retry logic."""
    global db_connection
    try:
        if db_connection is None or not db_connection.is_connected():
            db_connection = mysql.connector.connect(**MYSQL_CONFIG)
            print("✅ MySQL Connected")
        return db_connection
    except Error as e:
        print(f"❌ MySQL Error: {e}")
        return None

def init_db_connection():
    """Initialize database connection with retries."""
    retries = 10
    for i in range(retries):
        conn = get_db_connection()
        if conn:
            return conn
        print(f"⏳ Waiting for MySQL... ({i+1}/{retries})")
        time.sleep(3)
    print("❌ Failed to connect to MySQL after retries")
    return None

# --- TARGET TRACKING (Now with MySQL) ---
TARGET_DEDUP_DISTANCE = 2.0  # Meters - minimum distance between unique targets

import math

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance in meters between two GPS coordinates."""
    R = 6371000  # Earth radius in meters
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)
    
    a = math.sin(delta_lat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def get_all_targets_from_db():
    """Get all targets from MySQL database."""
    try:
        conn = get_db_connection()
        if not conn:
            return []
        cursor = conn.cursor(dictionary=True)
        cursor.execute("SELECT id, lat, lon, conf, source, timestamp, helmet_detected, track_id FROM detections ORDER BY id")
        targets = cursor.fetchall()
        cursor.close()
        # Convert Decimal to float for JSON serialization
        for t in targets:
            t['lat'] = float(t['lat'])
            t['lon'] = float(t['lon'])
            t['conf'] = float(t['conf']) if t['conf'] else 0
        return targets
    except Error as e:
        print(f"❌ DB Query Error: {e}")
        return []

def add_target_to_db(lat, lon, conf, source, helmet=False, track_id=None):
    """Add new target to MySQL database. Returns the new ID or None."""
    try:
        conn = get_db_connection()
        if not conn:
            return None
        cursor = conn.cursor()
        cursor.execute(
            "INSERT INTO detections (lat, lon, conf, source, helmet_detected, track_id) VALUES (%s, %s, %s, %s, %s, %s)",
            (lat, lon, conf, source, helmet, track_id)
        )
        conn.commit()
        new_id = cursor.lastrowid
        cursor.close()
        return new_id
    except Error as e:
        print(f"❌ DB Insert Error: {e}")
        return None

def is_duplicate_target(lat, lon):
    """Check if target already exists within deduplication distance (uses MySQL)."""
    targets = get_all_targets_from_db()
    for target in targets:
        dist = haversine_distance(lat, lon, target['lat'], target['lon'])
        if dist < TARGET_DEDUP_DISTANCE:
            return True
    return False

def nearest_neighbor_tsp(home, targets):
    """
    Solve TSP using Nearest Neighbor heuristic.
    Returns ordered list of points: home -> targets... -> home
    """
    if not targets:
        return [home, home]
    
    route = [home]
    remaining = targets.copy()
    current = home
    
    while remaining:
        # Find nearest unvisited target
        nearest = None
        nearest_dist = float('inf')
        
        for target in remaining:
            dist = haversine_distance(current[0], current[1], target[0], target[1])
            if dist < nearest_dist:
                nearest_dist = dist
                nearest = target
        
        if nearest:
            route.append(nearest)
            remaining.remove(nearest)
            current = nearest
    
    # Return to home
    route.append(home)
    return route

def get_mission_topic(data):
    target = data.get('drone', 'scout') # Default to scout if not specified
    return TOPICS.get(target, TOPICS['scout'])

def on_connect(client, userdata, flags, rc, properties=None):
    print("✅ API Connected to Broker")
    client.subscribe(TOPIC_TELEM_SCOUT)
    client.subscribe(TOPIC_TELEM_DELIVERY)
    client.subscribe("nidar/scout/detections")

def on_message(client, userdata, msg):
    global DETECTED_TARGETS
    try:
        payload_str = msg.payload.decode()
        # print(f"📩 MSG Received on {msg.topic}") # Uncomment for verbose spam
        
        data = json.loads(payload_str)
        drone_type = "scout" if "scout" in msg.topic else "delivery"
        
        # Determine if it is telemetry
        if "telemetry" in msg.topic:
            # print(f"   -> Forwarding Telemetry for {drone_type}")
            socketio.emit('telemetry_update', {"drone": drone_type, "data": data})
        
        elif "detections" in msg.topic:
            lat = data.get('lat')
            lon = data.get('lon')
            
            # Check for duplicates with 2m threshold
            if lat and lon and not is_duplicate_target(lat, lon):
                conf = data.get('conf', 0)
                source = data.get('source', 'unknown')
                helmet = data.get('helmet', False)
                track_id = data.get('track_id')
                
                # Add to MySQL database
                target_id = add_target_to_db(lat, lon, conf, source, helmet, track_id)
                
                if target_id:
                    targets = get_all_targets_from_db()
                    target = {
                        "id": target_id,
                        "lat": lat,
                        "lon": lon,
                        "conf": conf,
                        "source": source,
                        "helmet": helmet
                    }
                    print(f"🚨 NEW TARGET #{target_id}: ({lat:.6f}, {lon:.6f}) - Total: {len(targets)} [MySQL]")
                    
                    # Forward to frontend with target data
                    socketio.emit('survivor_alert', target)
            else:
                print(f"⚠️ Duplicate target ignored (within 2m of existing)")
            
    except Exception as e:
        print(f"❌ Error processing message: {e}")

@socketio.on('control_input')
def handle_control_input(data):
    """
    Handle virtual joystick input from frontend.
    Expected data: { "drone": "scout", "channels": [1500, 1500, 1000, 1500, ...] }
    """
    try:
        drone_target = data.get('drone', 'scout')
        channels = data.get('channels', [])
        
        # Construct MQTT RC Override Payload
        payload = {
            "type": "COMMAND",
            "act": "RC_OVERRIDE",
            "channels": channels
        }
        
        topic = TOPICS.get(drone_target, TOPICS['scout'])
        mqtt_client.publish(topic, json.dumps(payload))
        # print(f"🎮 Virtual Joystick: {drone_target} -> {channels[:4]}") # Verbose logging
        
    except Exception as e:
        print(f"❌ Control Input Error: {e}")

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
try:
    print(f"🔌 Connecting to Broker: {BROKER}")
    mqtt_client.connect(BROKER, 1883, 60)
    mqtt_client.loop_start()
except Exception as e:
    print(f"❌ MQTT Connection Fail: {e}")

@app.route('/upload_mission', methods=['POST'])
def upload_mission():
    data = request.json
    waypoints = data.get('waypoints')
    altitude = data.get('altitude', 20)  # Default to 20m if not provided
    speed_limit = data.get('speed_limit', 5.0)  # Default to 5 m/s
    payload = json.dumps({
        "type": "MISSION_UPLOAD", 
        "points": waypoints, 
        "altitude": altitude,
        "speed_limit": speed_limit
    })
    topic = get_mission_topic(data)
    mqtt_client.publish(topic, payload)
    return jsonify({"status": "Uploaded"})

@app.route('/resume_mission', methods=['POST'])
def resume_mission():
    data = request.json or {}
    topic = get_mission_topic(data)
    mqtt_client.publish(topic, json.dumps({"type": "COMMAND", "act": "RESUME_MISSION"}))
    return jsonify({"status": "RESUME Command Sent"})

# --- RESTORED COMMAND ROUTES ---
@app.route('/arm', methods=['POST'])
def arm_drone():
    data = request.json or {}
    topic = get_mission_topic(data)
    mqtt_client.publish(topic, json.dumps({"type": "COMMAND", "act": "ARM"}))
    return jsonify({"status": "ARM Command Sent"})

@app.route('/disarm', methods=['POST'])
def disarm_drone():
    data = request.json or {}
    topic = get_mission_topic(data)
    mqtt_client.publish(topic, json.dumps({"type": "COMMAND", "act": "DISARM"}))
    return jsonify({"status": "DISARM Command Sent"})

@app.route('/takeoff', methods=['POST'])
def takeoff_drone():
    data = request.json
    alt = data.get('alt', 10)  # Default to 10m if not specified
    topic = get_mission_topic(data)
    mqtt_client.publish(topic, json.dumps({"type": "COMMAND", "act": "TAKEOFF", "alt": alt}))
    return jsonify({"status": f"TAKEOFF Command Sent (alt={alt}m)"})

@app.route('/land', methods=['POST'])
def land_drone():
    data = request.json or {}
    topic = get_mission_topic(data)
    mqtt_client.publish(topic, json.dumps({"type": "COMMAND", "act": "LAND"}))
    return jsonify({"status": "LAND Command Sent"})

@app.route('/rtl', methods=['POST'])
def rtl_drone():
    data = request.json or {}
    topic = get_mission_topic(data)
    mqtt_client.publish(topic, json.dumps({"type": "COMMAND", "act": "RTL"}))
    return jsonify({"status": "RTL Command Sent"})



@app.route('/indoor_mode', methods=['POST'])
def indoor_mode():
    data = request.json or {}
    topic = get_mission_topic(data)
    mqtt_client.publish(topic, json.dumps({"type": "COMMAND", "act": "INDOOR_MODE"}))
    return jsonify({"status": "INDOOR MODE SET"})

@app.route('/start_rc_override', methods=['POST'])
def start_rc_override():
    data = request.json or {}
    topic = get_mission_topic(data)
    mqtt_client.publish(topic, json.dumps({"type": "COMMAND", "act": "START_RC_OVERRIDE"}))
    return jsonify({"status": "RC OVERRIDE STARTED"})

@app.route('/stop_rc_override', methods=['POST'])
def stop_rc_override():
    data = request.json or {}
    topic = get_mission_topic(data)
    mqtt_client.publish(topic, json.dumps({"type": "COMMAND", "act": "STOP_RC_OVERRIDE"}))
    return jsonify({"status": "RC OVERRIDE STOPPED"})

@app.route('/set_mode', methods=['POST'])
def set_flight_mode():
    data = request.json
    mode = data.get('mode')
    topic = get_mission_topic(data)
    mqtt_client.publish(topic, json.dumps({"type": "COMMAND", "act": "SET_MODE", "mode": mode}))
    return jsonify({"status": f"Mode {mode} Sent"})

@app.route('/rescue', methods=['POST'])
def rescue_command():
    data = request.json
    payload = {
        "type": "COMMAND", 
        "act": "RESCUE", 
        "lat": data.get('lat'), 
        "lon": data.get('lon')
    }
    # Publish to Delivery Drone Topic
    mqtt_client.publish(TOPIC_SURVIVOR, json.dumps(payload))
    return jsonify({"status": "RESCUE LAUNCHED"})

# --- TARGET MANAGEMENT ENDPOINTS ---
@app.route('/api/targets', methods=['GET'])
def get_targets():
    """Get all detected survivor targets from MySQL."""
    targets = get_all_targets_from_db()
    return jsonify({"targets": targets, "count": len(targets)})

@app.route('/api/arrange', methods=['POST'])
def arrange_route():
    """Calculate optimal delivery route using TSP."""
    data = request.json
    home = data.get('home')  # [lat, lon]
    
    if not home:
        return jsonify({"error": "Home location required"}), 400
    
    targets = get_all_targets_from_db()
    if not targets:
        return jsonify({"error": "No targets to route", "route": [home, home]}), 200
    
    # Extract target coordinates as [lat, lon] pairs
    target_coords = [[t['lat'], t['lon']] for t in targets]
    
    # Calculate optimal route using Nearest Neighbor TSP
    route = nearest_neighbor_tsp(home, target_coords)
    
    # Calculate total distance
    total_dist = 0
    for i in range(len(route) - 1):
        total_dist += haversine_distance(route[i][0], route[i][1], route[i+1][0], route[i+1][1])
    
    print(f"🛣️ ARRANGE: Calculated route with {len(route)} points, {total_dist:.0f}m total")
    
    return jsonify({
        "route": route,
        "total_distance_m": round(total_dist, 1),
        "waypoint_count": len(route) - 2  # Exclude home start and end
    })

@app.route('/api/clear_targets', methods=['POST'])
def clear_targets():
    """Clear all detected targets from MySQL database."""
    try:
        conn = get_db_connection()
        if conn:
            cursor = conn.cursor()
            cursor.execute("SELECT COUNT(*) FROM detections")
            count = cursor.fetchone()[0]
            cursor.execute("TRUNCATE TABLE detections")
            conn.commit()
            cursor.close()
            print(f"🗑️ Cleared {count} targets from MySQL")
            socketio.emit('targets_cleared', {"cleared": count})
            return jsonify({"status": "Targets cleared", "cleared": count})
        return jsonify({"error": "Database connection failed"}), 500
    except Error as e:
        print(f"❌ Clear targets error: {e}")
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    # Initialize database connection
    print("🔌 Connecting to MySQL...")
    init_db_connection()
    
    # allow_unsafe_werkzeug=True is required when using threading mode in Docker
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
