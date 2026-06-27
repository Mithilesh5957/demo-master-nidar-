import socket
import time
import threading
import struct

class C12Driver:
    """
    Re-Engineered Skydroid C12 Gimbal Driver.
    Based on decompiled rcsdk-v1.8.4.aar protocol analysis.
    
    Features:
    - Absolute positioning (goto_angles)
    - Velocity rate control at 50Hz (set_rates)
    - Live attitude telemetry parsing (yaw/pitch/roll)
    - Mode switching (follow/lock/follow_switch)
    - Dynamic checksum calculation
    - Camera actions (photo/record)
    """
    
    GIMBAL_IP = "192.168.144.108"
    GIMBAL_PORT = 5000
    GIMBAL_PORT_ALT = 12580  # Alternate port used by SDK
    
    # Telemetry subscription commands
    TELEM_SUBSCRIBE   = "#TPUG2wGAA0136"
    TELEM_UNSUBSCRIBE = "#TPUG2wGAA0035"
    
    # Mode commands
    MODE_FOLLOW        = "#TPUG2wPTZ063F"
    MODE_LOCK          = "#TPUG2wPTZ0740"
    MODE_FOLLOW_SWITCH = "#TPUG2wPTZ0841"
    
    # Camera commands
    CMD_PHOTO     = "#TPUD2wCAP013E"
    CMD_REC_START = "#TPUD2wREC0144"
    CMD_REC_STOP  = "#TPUD2wREC0043"
    
    # Gimbal presets
    CMD_CENTER = "#TPUG2wGHO006B"
    CMD_DOWN90 = "#TPUG2wGDP0073"
    
    # Rate control step unit (from SDK: speed / 0.5)
    RATE_STEP_UNIT = 0.5
    
    def __init__(self, ip=None, local_ip=None):
        self.ip = ip or self.GIMBAL_IP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to specific interface if needed (Pi with WiFi + Ethernet)
        if local_ip:
            try:
                self.sock.bind((local_ip, 0))
                print(f"[C12] Bound UDP to interface {local_ip}")
            except Exception as e:
                print(f"[C12] Warning: Could not bind to {local_ip}: {e}")
        
        # --- State ---
        self.running = True
        self.lock = threading.Lock()
        
        # Live attitude from telemetry (updated at ~50Hz)
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.telemetry_active = False
        
        # Rate control state
        self._active_rate_cmd = None  # Bytes to send at 50Hz for rate control
        
        # --- Threads ---
        # Background rate sender (50Hz)
        self._rate_thread = threading.Thread(target=self._rate_sender_loop, daemon=True)
        self._rate_thread.start()
        
        # Telemetry listener
        self._recv_sock = None
        self._listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._listen_thread.start()
        
        # Subscribe to telemetry
        time.sleep(0.1)  # Let listener bind first
        self._subscribe_telemetry()
        
        print(f"[C12] Driver initialized → {self.ip}:{self.GIMBAL_PORT}")
    
    # ========================================
    # PROTOCOL HELPERS
    # ========================================
    
    @staticmethod
    def _checksum(s):
        """
        Calculate Skydroid #TP checksum.
        Sum of ASCII values mod 256, formatted as 2-digit uppercase hex.
        """
        total = sum(ord(c) for c in s) % 256
        return f"{total:02X}"
    
    @staticmethod
    def _to_hex_short(value):
        """
        Convert a signed integer to a 4-character hex string (signed 16-bit).
        e.g. 1260 → '04EC', -9068 → 'DC94'
        """
        if value < 0:
            value = value + 0x10000  # Two's complement for 16-bit
        return f"{value & 0xFFFF:04X}"
    
    @staticmethod
    def _from_hex_short(hex_str):
        """
        Convert a 4-character hex string to a signed 16-bit integer.
        e.g. '04EC' → 1260, 'DC94' → -9068
        """
        val = int(hex_str, 16)
        if val >= 0x8000:
            val -= 0x10000
        return val
    
    @staticmethod
    def _speed_to_byte(deg_per_sec):
        """
        Convert degrees/second to signed byte [-127, 127].
        SDK divides by step unit 0.5.
        """
        raw = int(deg_per_sec / C12Driver.RATE_STEP_UNIT)
        return max(-127, min(127, raw))
    
    @staticmethod
    def _byte_to_hex(val):
        """
        Convert signed byte to 2-character hex.
        e.g. 100 → '64', -100 → '9C'
        """
        if val < 0:
            val = val + 0x100
        return f"{val & 0xFF:02X}"
    
    # ========================================
    # LOW-LEVEL COMMUNICATION
    # ========================================
    
    def _send_udp(self, data):
        """Send data to gimbal on both ports."""
        if isinstance(data, str):
            data = data.encode('utf-8')
        try:
            self.sock.sendto(data, (self.ip, self.GIMBAL_PORT))
            self.sock.sendto(data, (self.ip, self.GIMBAL_PORT_ALT))
        except Exception:
            pass
    
    def _send_cmd_with_checksum(self, cmd_body):
        """Append dynamic checksum and send."""
        checksum = self._checksum(cmd_body)
        full_cmd = cmd_body + checksum
        self._send_udp(full_cmd)
    
    # ========================================
    # TELEMETRY
    # ========================================
    
    def _subscribe_telemetry(self):
        """Send telemetry subscription command."""
        self._send_udp(self.TELEM_SUBSCRIBE)
        print("[C12] Telemetry subscription sent")
    
    def _unsubscribe_telemetry(self):
        """Send telemetry unsubscription command."""
        self._send_udp(self.TELEM_UNSUBSCRIBE)
        print("[C12] Telemetry unsubscribed")
    
    def _listen_loop(self):
        """
        Listen for incoming UDP telemetry packets.
        Packet format: #tpUGCrGAC[YAW_HEX:4][PITCH_HEX:4][ROLL_HEX:4][CHECKSUM:2]
        """
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._recv_sock.bind(("0.0.0.0", self.GIMBAL_PORT))
            self._recv_sock.settimeout(1.0)
            print(f"[C12] Telemetry listener started on port {self.GIMBAL_PORT}")
            
            while self.running:
                try:
                    data, addr = self._recv_sock.recvfrom(1024)
                    if not data:
                        continue
                    
                    decoded = data.decode('utf-8', errors='ignore').strip()
                    
                    # Parse attitude telemetry: #tpUGCrGAC + 4+4+4+2 = 24 chars
                    if decoded.startswith('#tpUGCrGAC') and len(decoded) >= 24:
                        try:
                            yaw_hex   = decoded[10:14]
                            pitch_hex = decoded[14:18]
                            roll_hex  = decoded[18:22]
                            
                            with self.lock:
                                self.current_yaw   = self._from_hex_short(yaw_hex) / 100.0
                                self.current_pitch  = self._from_hex_short(pitch_hex) / 100.0
                                self.current_roll   = self._from_hex_short(roll_hex) / 100.0
                                self.telemetry_active = True
                        except (ValueError, IndexError):
                            pass
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        time.sleep(0.5)
        finally:
            self._recv_sock.close()
    
    # ========================================
    # RATE CONTROL (Velocity — 50Hz Sender)
    # ========================================
    
    def _rate_sender_loop(self):
        """Background thread: sends active rate command at 50Hz."""
        while self.running:
            cmd = self._active_rate_cmd
            if cmd:
                self._send_udp(cmd)
            time.sleep(0.02)  # 50Hz
    
    def set_rates(self, pitch_speed=0.0, yaw_speed=0.0):
        """
        Set continuous rate movement.
        pitch_speed: degrees/second (positive = up, negative = down)
        yaw_speed: degrees/second (positive = right, negative = left)
        
        Command: #TPUG4wGSM[YAW_HEX:2][PITCH_HEX:2] + checksum
        """
        if pitch_speed == 0 and yaw_speed == 0:
            self.stop_movement()
            return
        
        yaw_byte = self._speed_to_byte(yaw_speed)
        pitch_byte = self._speed_to_byte(pitch_speed)
        
        yaw_hex = self._byte_to_hex(yaw_byte)
        pitch_hex = self._byte_to_hex(pitch_byte)
        
        cmd_body = f"#TPUG4wGSM{yaw_hex}{pitch_hex}"
        checksum = self._checksum(cmd_body)
        full_cmd = (cmd_body + checksum).encode('utf-8')
        
        self._active_rate_cmd = full_cmd
    
    def stop_movement(self):
        """Stop all gimbal movement immediately."""
        self._active_rate_cmd = None
        # Send zero-rate command a few times to ensure it stops
        cmd_body = "#TPUG4wGSM0000"
        checksum = self._checksum(cmd_body)
        full_cmd = (cmd_body + checksum).encode('utf-8')
        for _ in range(3):
            self._send_udp(full_cmd)
            time.sleep(0.02)
    
    # ========================================
    # ABSOLUTE POSITIONING (Goto)
    # ========================================
    
    def goto_angles(self, pitch=None, yaw=None):
        """
        Command gimbal to absolute angles.
        
        Combined: #TPUGCwGAM[YAW_HEX:4]10[PITCH_HEX:4]10 + checksum
        Pitch only: #TPUG6wGAP[PITCH_HEX:4]10 + checksum
        Yaw only: #TPUG6wGAY[YAW_HEX:4]10 + checksum
        """
        self.stop_movement()  # Stop any active rate first
        
        if pitch is not None and yaw is not None:
            # Combined command
            pitch_raw = int(pitch * 100)
            yaw_raw = int(yaw * 100)
            pitch_hex = self._to_hex_short(pitch_raw)
            yaw_hex = self._to_hex_short(yaw_raw)
            cmd_body = f"#TPUGCwGAM{yaw_hex}10{pitch_hex}10"
        elif pitch is not None:
            pitch_raw = int(pitch * 100)
            pitch_hex = self._to_hex_short(pitch_raw)
            cmd_body = f"#TPUG6wGAP{pitch_hex}10"
        elif yaw is not None:
            yaw_raw = int(yaw * 100)
            yaw_hex = self._to_hex_short(yaw_raw)
            cmd_body = f"#TPUG6wGAY{yaw_hex}10"
        else:
            return
        
        self._send_cmd_with_checksum(cmd_body)
        print(f"[C12] GOTO pitch={pitch}° yaw={yaw}°")
    
    # ========================================
    # PRESETS
    # ========================================
    
    def center(self):
        """Center gimbal (pitch=0, yaw=0)."""
        self.stop_movement()
        self._send_udp(self.CMD_CENTER)
        with self.lock:
            self.current_pitch = 0.0
            self.current_yaw = 0.0
        print("[C12] Center")
    
    def look_down(self):
        """Point camera straight down (-90° pitch)."""
        self.stop_movement()
        self._send_udp(self.CMD_DOWN90)
        with self.lock:
            self.current_pitch = -90.0
        print("[C12] Look Down 90°")
    
    # ========================================
    # RELATIVE MOVEMENT (Closed-Loop)
    # ========================================
    
    def move_angle_relative(self, delta_pitch=0, delta_yaw=0):
        """
        Move by a relative amount using closed-loop control.
        Reads current parsed angles → computes target → sends goto.
        """
        with self.lock:
            target_pitch = self.current_pitch + delta_pitch
            target_yaw = self.current_yaw + delta_yaw
        
        # Clamp pitch to physical limits
        target_pitch = max(-90.0, min(30.0, target_pitch))
        target_yaw = max(-180.0, min(180.0, target_yaw))
        
        self.goto_angles(pitch=target_pitch, yaw=target_yaw)
    
    # ========================================
    # MODE CONTROL
    # ========================================
    
    def set_mode(self, mode):
        """
        Set gimbal mode.
        mode: 'follow', 'lock', or 'follow_switch'
        """
        mode_map = {
            'follow': self.MODE_FOLLOW,
            'lock': self.MODE_LOCK,
            'follow_switch': self.MODE_FOLLOW_SWITCH
        }
        cmd = mode_map.get(mode.lower())
        if cmd:
            self._send_udp(cmd)
            print(f"[C12] Mode → {mode}")
        else:
            print(f"[C12] Unknown mode: {mode}")
    
    # ========================================
    # CAMERA ACTIONS
    # ========================================
    
    def take_photo(self):
        """Trigger photo capture."""
        self._send_udp(self.CMD_PHOTO)
        print("[C12] 📷 Photo")
    
    def start_recording(self):
        """Start video recording."""
        self._send_udp(self.CMD_REC_START)
        print("[C12] 🔴 Record Start")
    
    def stop_recording(self):
        """Stop video recording."""
        self._send_udp(self.CMD_REC_STOP)
        print("[C12] ⏹️ Record Stop")
    
    # ========================================
    # TRACKING HELPER
    # ========================================
    
    def update_tracking(self, error_x, error_y, deadzone=0.05, speed=30.0):
        """
        Real-time tracking helper. Call every frame.
        error_x/y: normalized [-1, 1] offset from center
        """
        yaw_rate = 0.0
        pitch_rate = 0.0
        
        if abs(error_x) > deadzone:
            yaw_rate = error_x * speed
        if abs(error_y) > deadzone:
            pitch_rate = -error_y * speed  # Inverted: positive error_y = look down
        
        if yaw_rate == 0 and pitch_rate == 0:
            self.stop_movement()
        else:
            self.set_rates(pitch_speed=pitch_rate, yaw_speed=yaw_rate)
    
    # ========================================
    # RAW ACCESS
    # ========================================
    
    def send_raw(self, hex_string):
        """Send raw hex bytes to gimbal."""
        try:
            byte_data = bytes.fromhex(hex_string.replace(" ", ""))
            self._send_udp(byte_data)
            print(f"[C12] Raw: {byte_data}")
        except Exception:
            print("[C12] Invalid hex string")
    
    # ========================================
    # GETTERS
    # ========================================
    
    def get_attitude(self):
        """Get current gimbal attitude as dict."""
        with self.lock:
            return {
                'yaw': round(self.current_yaw, 2),
                'pitch': round(self.current_pitch, 2),
                'roll': round(self.current_roll, 2),
                'active': self.telemetry_active
            }
    
    # ========================================
    # LIFECYCLE
    # ========================================
    
    def close(self):
        """Shutdown driver gracefully."""
        print("[C12] Shutting down...")
        self.running = False
        self.stop_movement()
        self._unsubscribe_telemetry()
        time.sleep(0.1)
        try:
            self.sock.close()
        except:
            pass
        print("[C12] Driver closed")


# ========================================
# TEST BENCH
# ========================================
if __name__ == "__main__":
    driver = C12Driver()
    try:
        print("--- C12 Re-Engineered Driver Test ---\n")
        
        # Wait for telemetry
        print("Waiting for telemetry...")
        time.sleep(3)
        att = driver.get_attitude()
        print(f"Current: Yaw={att['yaw']}° Pitch={att['pitch']}° Roll={att['roll']}°\n")
        
        # Test 1: Center
        print("1. Centering...")
        driver.center()
        time.sleep(2)
        
        # Test 2: Absolute goto
        print("2. Goto pitch=-45°, yaw=30°...")
        driver.goto_angles(pitch=-45, yaw=30)
        time.sleep(3)
        att = driver.get_attitude()
        print(f"   Result: Yaw={att['yaw']}° Pitch={att['pitch']}°\n")
        
        # Test 3: Rate control
        print("3. Rate: yaw right at 30°/s for 2s...")
        driver.set_rates(yaw_speed=30)
        time.sleep(2)
        driver.stop_movement()
        att = driver.get_attitude()
        print(f"   Result: Yaw={att['yaw']}° Pitch={att['pitch']}°\n")
        
        # Test 4: Return to center
        print("4. Return to center...")
        driver.center()
        time.sleep(2)
        
        print("✅ All tests complete")
        
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        driver.close()
