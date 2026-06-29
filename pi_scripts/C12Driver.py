import socket
import time
import threading

class C12Driver:
    """
    Re-engineered Python Driver for the Skydroid C12 Gimbal & Camera.
    Provides DJI Drone Camera-like capabilities including absolute angle control,
    velocity rate control, gimbal mode switching, and real-time attitude telemetry.
    """
    
    def __init__(self, ip="192.168.144.108", local_ip=None):
        self.ip = ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to ephemeral port to receive UDP telemetry back to this socket
        if local_ip:
            try:
                self.sock.bind((local_ip, 0))
                print(f"[C12] Bound outbound/inbound UDP to {local_ip}:{self.sock.getsockname()[1]}")
            except Exception as e:
                print(f"[C12] Warning: Could not bind to local IP {local_ip}, falling back: {e}")
                self.sock.bind(("0.0.0.0", 0))
        else:
            self.sock.bind(("0.0.0.0", 0))
            print(f"[C12] Bound outbound/inbound UDP to ephemeral port: {self.sock.getsockname()[1]}")
            
        self.running = True
        self.active_cmd = None  # Command to send repeatedly in background loop (None = Idle)
        self.lock = threading.Lock()
        
        # Real-time Telemetry (Yaw, Pitch, Roll in degrees)
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.last_telemetry_time = 0.0
        
        # Start background UDP sender (50Hz)
        self.sender_thread = threading.Thread(target=self._background_loop, daemon=True)
        self.sender_thread.start()
        
        # Start telemetry listener thread
        self.listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listener_thread.start()
        
        # Enable gimbal attitude push
        self.enable_telemetry(True)

    def _calculate_checksum(self, cmd_str):
        """Calculates 2-digit uppercase hex checksum."""
        cmd_bytes = cmd_str.encode('utf-8')
        total = sum(cmd_bytes) & 0xFF
        return format(total, '02X')

    def _send_udp(self, data):
        """Helper to send UDP packets to Port 5000 (Gimbal) and 12580 (Camera)."""
        try:
            self.sock.sendto(data, (self.ip, 5000))
            self.sock.sendto(data, (self.ip, 12580))
        except Exception as e:
            pass

    def _background_loop(self):
        """Runs background loop to continuously send rate/joystick commands at 50Hz."""
        while self.running:
            with self.lock:
                cmd = self.active_cmd
            if cmd:
                self._send_udp(cmd)
            time.sleep(0.02) # 50Hz

    def _listen_loop(self):
        """Listens for incoming UDP attitude packets from the Gimbal."""
        self.sock.settimeout(1.0)
        while self.running:
            try:
                data, addr = self.sock.recvfrom(2048)
                if data:
                    try:
                        decoded = data.decode('utf-8', errors='ignore').strip()
                        if decoded.startswith("#tpUGCrGAC"):
                            self._parse_telemetry(decoded)
                    except Exception as e:
                        pass
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[C12] Listener error: {e}")
                time.sleep(0.5)

    def _parse_telemetry(self, data_str):
        """
        Parses telemetry response format: #tpUGCrGAC[YAW][PITCH][ROLL][CHECKSUM]
        Yaw, Pitch, Roll are 4-character signed 16-bit hex values representing (degrees * 100).
        """
        if len(data_str) >= 24:
            try:
                # Validate checksum
                base = data_str[:22]
                rx_chk = data_str[22:24]
                if self._calculate_checksum(base) != rx_chk:
                    return
                
                # Parse signed 16-bit hex fields
                def hex_to_float(h):
                    val = int(h, 16)
                    if val >= 0x8000:
                        val -= 0x10000
                    return val / 100.0
                
                self.current_yaw = hex_to_float(data_str[10:14])
                self.current_pitch = hex_to_float(data_str[14:18])
                self.current_roll = hex_to_float(data_str[18:22])
                self.last_telemetry_time = time.time()
            except Exception:
                pass

    def enable_telemetry(self, enable=True):
        """Enables/Disables continuous attitude data streaming from the Gimbal."""
        cmd_str = "#TPUG2wGAA01" if enable else "#TPUG2wGAA00"
        chk = self._calculate_checksum(cmd_str)
        self._send_udp(f"{cmd_str}{chk}".encode('utf-8'))

    def get_current_attitude(self, timeout=1.5):
        """
        Synchronously blocks and returns the next received telemetry update.
        Returns a tuple of (yaw, pitch, roll) or None if timeout expires.
        """
        start_time = time.time()
        last_t = self.last_telemetry_time
        while time.time() - start_time < timeout:
            if self.last_telemetry_time > last_t:
                return (self.current_yaw, self.current_pitch, self.current_roll)
            time.sleep(0.01)
        return None

    def close(self):
        """Clean shutdown of threads and socket."""
        self.enable_telemetry(False)
        time.sleep(0.1)
        self.running = False
        with self.lock:
            self.active_cmd = None
        try:
            self.sock.close()
        except:
            pass

    # --- Absolute Position Control (DJI Drone Style) ---
    def goto_pitch(self, pitch_deg):
        """Commands the gimbal pitch to an absolute angle (degrees)."""
        p_val = int(max(-90.0, min(90.0, pitch_deg)) * 100)
        p_hex = format(p_val & 0xFFFF, '04X')
        base_cmd = f"#TPUG6wGAP{p_hex}10"
        chk = self._calculate_checksum(base_cmd)
        self._send_udp(f"{base_cmd}{chk}".encode('utf-8'))

    def goto_yaw(self, yaw_deg):
        """Commands the gimbal yaw to an absolute angle (degrees)."""
        y_val = int(max(-90.0, min(90.0, yaw_deg)) * 100)
        y_hex = format(y_val & 0xFFFF, '04X')
        base_cmd = f"#TPUG6wGAY{y_hex}10"
        chk = self._calculate_checksum(base_cmd)
        self._send_udp(f"{base_cmd}{chk}".encode('utf-8'))

    def goto_angles(self, pitch_deg, yaw_deg):
        """Commands both Pitch and Yaw to absolute angles simultaneously."""
        p_val = int(max(-90.0, min(90.0, pitch_deg)) * 100)
        y_val = int(max(-90.0, min(90.0, yaw_deg)) * 100)
        p_hex = format(p_val & 0xFFFF, '04X')
        y_hex = format(y_val & 0xFFFF, '04X')
        
        base_cmd = f"#TPUGCwGAM{y_hex}10{p_hex}10"
        chk = self._calculate_checksum(base_cmd)
        self._send_udp(f"{base_cmd}{chk}".encode('utf-8'))

    # --- Velocity Rate Control (DJI Drone Style) ---
    def set_rates(self, pitch_speed, yaw_speed):
        """
        Sets continuous rotation rates in degrees per second.
        Gimbal keeps moving at this rate until set_rates(0,0) or stop_movement() is called.
        """
        # Map speed to byte value range [-127, 127] (approx speed / 0.5)
        p_val = int(max(-127, min(127, pitch_speed / 0.5)))
        y_val = int(max(-127, min(127, yaw_speed / 0.5)))
        
        p_hex = format(p_val & 0xFF, '02X')
        y_hex = format(y_val & 0xFF, '02X')
        
        base_cmd = f"#TPUG4wGSM{y_hex}{p_hex}"
        chk = self._calculate_checksum(base_cmd)
        
        with self.lock:
            self.active_cmd = f"{base_cmd}{chk}".encode('utf-8')

    def stop_movement(self):
        """Stops all rate-based movements immediately."""
        with self.lock:
            self.active_cmd = None
        # Send explicit stop commands
        gsp_stop = f"#TPUG2wGSP00{self._calculate_checksum('#TPUG2wGSP00')}"
        gsy_stop = f"#TPUG2wGSY00{self._calculate_checksum('#TPUG2wGSY00')}"
        self._send_udp(gsp_stop.encode('utf-8'))
        self._send_udp(gsy_stop.encode('utf-8'))

    # --- Mode Control ---
    def set_mode(self, mode):
        """
        Sets gimbal mode:
        'follow' - Gimbal Yaw aligns with aircraft heading.
        'lock'   - Gimbal Yaw locks to absolute world compass heading.
        'follow_switch' - Hybrid follow/lock mode.
        """
        mode_cmds = {
            "follow": "#TPUG2wPTZ06",
            "lock": "#TPUG2wPTZ07",
            "follow_switch": "#TPUG2wPTZ08"
        }
        if mode in mode_cmds:
            base = mode_cmds[mode]
            chk = self._calculate_checksum(base)
            self._send_udp(f"{base}{chk}".encode('utf-8'))
            print(f"[C12] Mode set to: {mode}")

    # --- Camera Actions ---
    def take_photo(self):
        """Triggers a photo capture."""
        self._send_udp(b"#TPUD2wCAP013E")
        
    def start_recording(self):
        """Starts video recording."""
        self._send_udp(b"#TPUD2wREC0144")
        
    def stop_recording(self):
        """Stops video recording."""
        self._send_udp(b"#TPUD2wREC0043")

    def center(self):
        """Native command to center the gimbal immediately."""
        self.stop_movement()
        self._send_udp(b"#TPUG2wGHO006B")

    def look_down_90(self):
        """Native command to face the gimbal straight down (90 degrees)."""
        self.stop_movement()
        self._send_udp(b"#TPUG2wGDP0073")

    # --- Legacy Helpers ---
    def move_angle_relative(self, pitch_delta, yaw_delta):
        """Relative angular move using absolute feedback loop (much more accurate than timing)."""
        target_pitch = self.current_pitch + pitch_delta
        target_yaw = self.current_yaw + yaw_delta
        self.goto_angles(target_pitch, target_yaw)

