from pymavlink import mavutil
import paho.mqtt.client as mqtt
import json
import time
import threading
import os
import struct
import socket
import subprocess

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("⚠️ RPi.GPIO not found - Servo will be simulated")

# 🔴 CONFIGURATION
SERVO_PINS = [17, 27, 22, 23]  # GPIO pins for up to 4 servos
PAYLOAD_CAPACITY = 4
payloads_remaining = PAYLOAD_CAPACITY
servo_index = 0  # tracks which servo to use next
BROKER_IPS = ["100.113.148.48"] # Your laptop's Tailscale IP (mithilesh-m)
TOPIC_MISSION = "nidar/delivery/mission"
TOPIC_TELEM = "nidar/delivery/telemetry"
TOPIC_SURVIVOR = "nidar/delivery/target"

COPTER_MODES = {
    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED', 5: 'LOITER', 
    6: 'RTL', 7: 'CIRCLE', 9: 'LAND', 11: 'DRIFT', 13: 'SPORT', 16: 'POSHOLD', 
    17: 'BRAKE', 18: 'THROW', 19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS'
}

# --- GLOBAL STATE ---
DRONE_DATA = {
    "lat": 0, "lon": 0, "alt": 0, 
    "bat": 0, "bat_voltage": 0, "bat_current": 0,
    "status": "DISARMED", "mode": "UNKNOWN",
    "heading": 0, "current_wp": -1, "motors": [0,0,0,0],
    "connected": False, "dist_to_home": 0, "rtl_bat_min": 0
}

MISSION_POINTS = []
MISSION_FILE = "mission_state.json"
mqtt_clients = []

# --- MAVLINK ---
print("🔌 Connecting to Mavlink Router...")
drone = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

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

def send_arm():
    print(f"⚠️ Sending ARM Command (Current Mode: {DRONE_DATA.get('mode')})...")
    # Set Disarm Delay to 2 minutes (120s) instead of 0 to ensure it's disabled or very long
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'DISARM_DELAY', 120, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(0.5)
    
    try:
        drone.mav.command_long_send(drone.target_system, drone.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        # OPTIMISTIC UPDATE: Force status immediately
        DRONE_DATA['status'] = "ARMED"
        print("✅ Optimistic Status: ARMED")
    except Exception as e:
        print(f"❌ ARM Params Failed: {e}")

def send_disarm(force=True):
    magic = 21196 if force else 0
    drone.mav.command_long_send(drone.target_system, drone.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, magic, 0, 0, 0, 0, 0)
    # OPTIMISTIC UPDATE: Force status immediately
    DRONE_DATA['status'] = "DISARMED"
    print("✅ Optimistic Status: DISARMED")

def handle_takeoff(alt):
    try:
        set_flight_mode('GUIDED'); time.sleep(1)
        if DRONE_DATA['status'] != 'ARMED': send_arm(); time.sleep(2)
        drone.mav.command_long_send(drone.target_system, drone.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, float('nan'), DRONE_DATA['lat'], DRONE_DATA['lon'], alt)
    except Exception as e: print(f"Takeoff Error: {e}")


def drop_payload():
    global payloads_remaining, servo_index
    if payloads_remaining <= 0:
        print("⚠️ PAYLOAD EMPTY! Cannot drop.")
        return

    print(f"📦 DROPPING PAYLOAD! ({payloads_remaining-1} left)")
    
    if GPIO_AVAILABLE:
        try:
            # Choose the next servo pin from the SERVO_PINS list
            pin = SERVO_PINS[servo_index]
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, 50)  # 50Hz
            pwm.start(2.5)  # 0 deg (Locked)
            time.sleep(0.5)
            pwm.ChangeDutyCycle(7.5) # 90 deg (Open) - Adjust as needed
            time.sleep(1.0) # Wait for mekanism
            pwm.ChangeDutyCycle(2.5) # Lock again (or stay open?) assuming single drop mechanism
            # If "4 at a time" means 4 separate servos or 1 mechanism triggering 4 times?
            # Assuming 1 mechanism cycling.
            pwm.stop()
        except Exception as e:
            print(f"❌ GPIO Error: {e}")
    else:
        print("Done (Simulated Drop)")
    
    payloads_remaining -= 1
    # Advance to the next servo for the subsequent drop
    servo_index = (servo_index + 1) % len(SERVO_PINS)

def execute_rescue_mission(lat, lon):
    """Uploads a mission: Fly -> Delay (Pi drops manually during this) -> RTL"""
    print(f"🚑 Generating Rescue Mission for {lat}, {lon}")
    
    # We use a Standard MAVLink Mission. 
    # The Pi tracks 'MISSION_ITEM_REACHED'. When it reaches the Waypoint (Seq 1), it fires GPIO.
    waypoints = [
        {"type": "WAYPOINT", "lat": lat, "lon": lon, "alt": 6.5},  # Seq 1: Go to Survivor
        {"type": "DELAY", "seconds": 5},                           # Seq 2: Hover 5s (Pi drops here)
        {"type": "RTL"}                                            # Seq 3: Come Home
    ]
    upload_mission_to_fc(waypoints, auto_start=True, takeoff_alt=6.5)

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

def upload_mission_to_fc(waypoints, auto_start=True, takeoff_alt=6.5, is_resume=False):
    global MISSION_POINTS
    MISSION_POINTS = waypoints if is_resume else [{"type": "TAKEOFF", "alt": takeoff_alt}, *waypoints, {"type": "RTL"}]
    save_mission_state()
    drone.mav.mission_clear_all_send(drone.target_system, drone.target_component)
    time.sleep(0.5)
    drone.mav.mission_count_send(drone.target_system, drone.target_component, len(MISSION_POINTS))
    if auto_start:
        threading.Thread(target=auto_start_sequence).start()

def auto_start_sequence():
    time.sleep(2) # Wait for upload
    if DRONE_DATA['status'] != 'ARMED': send_arm(); time.sleep(2)
    set_flight_mode('AUTO')

    if seq < len(MISSION_POINTS):
        p = MISSION_POINTS[seq]
        cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        lat, lon, alt = int(p.get('lat',0)*1e7), int(p.get('lon',0)*1e7), 6.5
        p1, p2, p3, p4 = 0, 0, 0, 0 # Default params

        if p.get('type') == 'TAKEOFF':
             cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
             lat, lon, alt = int(DRONE_DATA['lat']*1e7), int(DRONE_DATA['lon']*1e7), int(p['alt'])
        elif p.get('type') == 'RTL':
             cmd = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
             lat, lon, alt = 0, 0, 0
        elif p.get('type') == 'SERVO':
             cmd = mavutil.mavlink.MAV_CMD_DO_SET_SERVO
             p1 = p.get('ch', 9)    # Servo Instance (Channel)
             p2 = p.get('pwm', 1500) # PWM Value
             lat, lon, alt = 0, 0, 0 # Not spatial
        elif p.get('type') == 'DELAY':
             cmd = mavutil.mavlink.MAV_CMD_CONDITION_DELAY
             p1 = p.get('seconds', 0) # Seconds
             lat, lon, alt = 0, 0, 0

        drone.mav.mission_item_int_send(drone.target_system, drone.target_component, seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd, 0, 1, p1, p2, p3, p4, lat, lon, alt)
        # Trigger payload drop when we reach a delivery waypoint and payloads remain
        if p.get('type') == 'WAYPOINT' and payloads_remaining > 0:
            drop_payload()

# --- MQTT ---
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        if data.get('type') == "COMMAND":
            act = data.get('act')
            if act != "RC_OVERRIDE": print(f"⚙️ CMD: {act}")
            if act == "ARM": 
                def arm_sequence():
                    # 1. Force RC Override Enabled
                    print("🛡️ Preparing to Arm: Enforcing Low Throttle...")
                    # Even if active, force channels to safe defaults
                    global RC_OVERRIDE_ACTIVE, RC_OVERRIDE_CHANNELS
                    RC_OVERRIDE_CHANNELS = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000] # Ch3=1000
                    RC_OVERRIDE_ACTIVE = True 

                    time.sleep(1.0)
                    
                    # Direct Arm (User controls mode)
                    send_arm()
                threading.Thread(target=arm_sequence, daemon=True).start()
            elif act == "DISARM": send_disarm()
            elif act == "LAND": set_flight_mode('LAND')
            elif act == "RTL": set_flight_mode('RTL')
            elif act == "TAKEOFF": threading.Thread(target=handle_takeoff, args=(float(data.get('alt', 6.5)),)).start()

            elif act == "SET_MODE": set_flight_mode(data.get('mode'))
            elif act == "START_RC_OVERRIDE": start_rc_override()
            elif act == "STOP_RC_OVERRIDE": stop_rc_override()
            elif act == "RC_OVERRIDE": override_rc(data.get('channels', []))
            elif act == "RESUME_MISSION":
                last_wp = load_mission_state()
                if len(MISSION_POINTS) > 0 and last_wp >= 0:
                    DRONE_DATA['current_wp'] = last_wp
                    upload_mission_to_fc(MISSION_POINTS, auto_start=False, is_resume=True)
                    time.sleep(1)
                    drone.mav.mission_set_current_send(drone.target_system, drone.target_component, last_wp)
                    time.sleep(1)
                    if DRONE_DATA['status'] != 'ARMED': send_arm(); time.sleep(2)
                    set_flight_mode('AUTO')
        
        elif data.get('type') == "SURVIVOR_FOUND":
            # Single human detection
            if payloads_remaining <= 0:
                print("⚠️ No payloads remaining – cannot execute rescue mission.")
            elif data.get('confidence', 1.0) < 0.7:
                print(f"⚠️ Detection confidence too low ({data.get('confidence')}); ignoring.")
            else:
                execute_rescue_mission(data['lat'], data['lon'])
        elif data.get('type') == "SURVIVOR_LOCATIONS":
            # Multiple human detections sent as a list of points
            points = data.get('points', [])
            if not points:
                print("⚠️ No points received for SURVIVOR_LOCATIONS.")
            elif payloads_remaining < len(points):
                print(f"⚠️ Not enough payloads ({payloads_remaining}) for {len(points)} locations.")
            else:
                # Build a mission with a waypoint for each point; drop_payload will be called at each waypoint
                waypoints = [{"type": "WAYPOINT", "lat": p['lat'], "lon": p['lon'], "alt": 6.5} for p in points]
                # Safety altitude clamp
                requested_alt = float(data.get('altitude', 6.5))
                safe_alt = min(requested_alt, 7.6)
                print(f"🔒 Backend Safety: Clamped Alt {requested_alt}m -> {safe_alt}m")
                upload_mission_to_fc(waypoints, takeoff_alt=safe_alt, auto_start=False)


    except Exception as e: print(f"MQTT Error: {e}")

def setup_mqtt():
    for ip in BROKER_IPS:
        try:
            c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            c.on_message = on_message
            c.connect_async(ip, 1883, 60)
            c.loop_start()
            c.subscribe(TOPIC_MISSION)
            c.subscribe(TOPIC_SURVIVOR)
            mqtt_clients.append(c)
            print(f"✅ MQTT Client for {ip}")
        except: pass

# --- RC OVERRIDE ---
RC_OVERRIDE_ACTIVE = False
# Channel Map: 1:Roll, 2:Pitch, 3:Throttle, 4:Yaw, 5:SwA, 6:SwB, 7:SwC, 8:SwD
RC_OVERRIDE_CHANNELS = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000] 

def rc_override_thread():
    global RC_OVERRIDE_ACTIVE
    print("🕹️ RC Override Thread Started (Daemon)")
    while True:
        if RC_OVERRIDE_ACTIVE:
            drone.mav.rc_channels_override_send(
                drone.target_system, drone.target_component,
                *RC_OVERRIDE_CHANNELS
            )
        time.sleep(0.1)

def start_rc_override():
    global RC_OVERRIDE_ACTIVE, RC_OVERRIDE_CHANNELS
    if not RC_OVERRIDE_ACTIVE:
        print("⚠️ Enabling RC Override (Sending Heartbeat)...")
        RC_OVERRIDE_CHANNELS = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        RC_OVERRIDE_ACTIVE = True

def stop_rc_override():
    global RC_OVERRIDE_ACTIVE
    print("🕹️ Disabling RC Override (Releasing Control)...")
    RC_OVERRIDE_ACTIVE = False
    pwm = [0] * 8
    drone.mav.rc_channels_override_send(drone.target_system, drone.target_component, *pwm)

threading.Thread(target=rc_override_thread, daemon=True).start()

def override_rc(channels):
    global RC_OVERRIDE_CHANNELS
    for i, val in enumerate(channels):
        if i < 8: RC_OVERRIDE_CHANNELS[i] = int(val)

def voltage_to_percent(v):
    """Fallback calculation for battery % when FC doesn't report it."""
    if v < 5: return -1 # No voltage detected
    cells = math.ceil(v / 4.25)
    if cells < 1: return -1
    min_v = cells * 3.5
    max_v = cells * 4.2
    pct = ((v - min_v) / (max_v - min_v)) * 100
    return max(0, min(100, round(pct, 1)))

def fix_battery_param():
    """Auto-fix the BATT_LOW_VOLT and BATT_MONITOR parameter."""
    print("🔋 Checking/Fixing Battery Parameters...")
    
    # Enable Analog Monitor (4)
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'BATT_MONITOR', 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(0.1)
    
    # Set BATT_LOW_VOLT to 20.0V (Account for ~2V sag under load)
    drone.mav.param_set_send(
        drone.target_system, drone.target_component,
        b'BATT_LOW_VOLT',
        20.0,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.1)
    print("✅ Battery Params Sent (Monitor=4, LowVolt=20.0V)")

# --- MAIN ---
if __name__ == "__main__":
    setup_mqtt()

    # Send Battery Fix on Startup
    threading.Thread(target=fix_battery_param, daemon=True).start()

    print("✅ Delivery System Online (Telemetry + Control Only)")

    # Request Telemetry Streams (Critical for real-time data)
    print("📡 Requesting Data Stream...")
    drone.mav.request_data_stream_send(drone.target_system, drone.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    last_telem_send = time.time()
    
    while True:
        try:
            msg = drone.recv_match(blocking=False)
            if msg:
                mtype = msg.get_type()
                if mtype == 'HEARTBEAT':
                    # FILTER: Only accept Heartbeats from Autopilot (Comp 1)
                    if msg.get_srcComponent() != 1: continue

                    DRONE_DATA["status"] = "ARMED" if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "DISARMED"
                    DRONE_DATA["mode"] = COPTER_MODES.get(msg.custom_mode, str(msg.custom_mode))
                    DRONE_DATA["connected"] = True
                elif mtype == 'GLOBAL_POSITION_INT':
                    DRONE_DATA["lat"] = msg.lat / 1e7
                    DRONE_DATA["lon"] = msg.lon / 1e7
                    DRONE_DATA["alt"] = msg.relative_alt / 1000.0
                elif mtype == 'SYS_STATUS':
                    v_raw = msg.voltage_battery / 1000.0
                    # Voltage Correction Logic:
                    # User feedback: "Telemetry shows correct voltage when not flying" -> 0V correction when Disarmed.
                    # "Always +2" implies need for +2V correction under load/flying condition.
                    correction = 2.0 if DRONE_DATA['status'] == 'ARMED' else 0.0
                    v = max(0.0, v_raw + correction)
                    
                    DRONE_DATA["bat_voltage"] = v
                    DRONE_DATA["bat_current"] = msg.current_battery / 100.0
                    
                    if int(time.time()) % 5 == 0 and time.time() % 1 < 0.2:
                         print(f"⚡ BATTERY: Raw={v_raw:.2f}V Corrected={v:.2f}V ({correction:+.1f}V), {DRONE_DATA['bat_current']:.2f}A, Rem: {msg.battery_remaining}%")

                    if msg.battery_remaining == -1:
                        DRONE_DATA["bat"] = voltage_to_percent(v)
                    else:
                        DRONE_DATA["bat"] = msg.battery_remaining
                elif mtype == 'MISSION_REQUEST':
                    send_mission_item(msg.seq)
                elif mtype == 'MISSION_ACK' and msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print("✅ Mission Uploaded")
                    drone.mav.mission_set_current_send(drone.target_system, drone.target_component, 0)
                    
                elif mtype == 'MISSION_CURRENT':
                    DRONE_DATA["current_wp"] = msg.seq
                    save_mission_state()
                
                elif mtype == 'MISSION_ITEM_REACHED':
                    print(f"📍 Reached Waypoint #{msg.seq}")
                    DRONE_DATA["current_wp"] = msg.seq
                    save_mission_state()

                elif mtype == 'VFR_HUD':
                    DRONE_DATA['speed'] = round(msg.groundspeed, 1)
                    DRONE_DATA['heading'] = msg.heading
                
                elif mtype == 'GPS_RAW_INT':
                    DRONE_DATA['gps_sats'] = msg.satellites_visible
                    DRONE_DATA['gps_fix'] = msg.fix_type

            # Smart Battery Failsafe
            if DRONE_DATA['lat'] != 0:
                 # Power Model (Delivery Drone might be heavier/different, but using Safe Defaults)
                 # Assume Flight + Electronics ~25A standard cruise
                 current_draw = DRONE_DATA.get('bat_current', 0)
                 if current_draw < 1.0: current_draw = 26.0 # Slightly higher default for delivery
                 
                 rtl_speed = 8.0
                 time_to_home_hr = (DRONE_DATA.get('dist_to_home', 0) / rtl_speed) / 3600.0
                 
                 # Capacity Check
                 capacity_needed_mah = (current_draw * 1000 * time_to_home_hr) * 1.25 # +25% overhead
                 BATTERY_CAPACITY_MAH = 22000 
                 
                 percent_needed = (capacity_needed_mah / BATTERY_CAPACITY_MAH) * 100
                 DRONE_DATA['req_rtl_batt'] = round(percent_needed + 5, 1) # +5% Landing buffer

                 # Trigger RTL if needed
                 if (DRONE_DATA['mode'] in ['GUIDED', 'AUTO', 'LOITER']) and \
                    (DRONE_DATA['bat'] < DRONE_DATA['req_rtl_batt']) and \
                    (DRONE_DATA['bat'] > 0) and (DRONE_DATA['req_rtl_batt'] > 0):
                      print(f"⚠️ SMART FAILSAFE TRIGGERED: {DRONE_DATA['bat']}% < {DRONE_DATA['req_rtl_batt']}%")
                      set_flight_mode('RTL')

        except Exception as e:
            time.sleep(0.1)

        # Telemetry Broadcast (4Hz) - MOVED OUTSIDE TRY BLOCK
        if time.time() - last_telem_send >= 0.25:
            last_telem_send = time.time()
            payload = json.dumps(DRONE_DATA)
            sent_count = 0
            for c in mqtt_clients:
                if c.is_connected(): 
                    c.publish(TOPIC_TELEM, payload)
                    sent_count += 1
            if sent_count == 0: 
                print("⚠️ MQTT Disconnected")
            # else: 
            #    print(f"📡 Telem Sent: {DRONE_DATA['bat']}% Bat | {DRONE_DATA['lat']} Lat") # Debug Print
        
        time.sleep(0.001)
