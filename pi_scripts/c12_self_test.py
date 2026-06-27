import os
import time

# MUST be set before importing cv2
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
import cv2

try:
    from C12Driver import C12Driver
except ImportError:
    print("❌ ERROR: C12Driver.py not found in this directory. Make sure to run this from pi_scripts.")
    exit(1)

def test_camera():
    print("\n--- 1. Testing C12 RGB Camera (RTSP TCP) ---")
    rtsp_url = "rtsp://192.168.144.108:554/stream=1"
    print(f"📡 Connecting to {rtsp_url}...")
    
    cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
    
    if not cap.isOpened():
        print("❌ FAILED: Could not open RTSP stream. Check camera IP and network connection.")
        return False
        
    print("✅ Stream opened successfully. Attempting to grab a frame...")
    
    # Try a few times to get a valid frame (FFmpeg might drop the first few due to I-frame searching)
    frame = None
    for i in range(10):
        ret, f = cap.read()
        if ret and f is not None:
            frame = f
            break
        time.sleep(0.1)
        
    cap.release()
    
    if frame is not None:
        filename = "test_frame.jpg"
        cv2.imwrite(filename, frame)
        print(f"✅ SUCCESS: Captured frame and saved to '{filename}' (Resolution: {frame.shape[1]}x{frame.shape[0]})")
        return True
    else:
        print("❌ FAILED: Connected to stream but could not decode any frames.")
        return False

def test_gimbal():
    print("\n--- 2. Testing C12 Gimbal Hardware Control ---")
    print("🔌 Initializing C12 Driver at 192.168.144.108:5000...")
    
    try:
        gimbal = C12Driver(ip="192.168.144.108")
    except Exception as e:
        print(f"❌ FAILED to initialize driver: {e}")
        return
        
    print("✅ Driver initialized. Waiting 3 seconds for telemetry connection...")
    time.sleep(3)
    
    att = gimbal.get_attitude()
    if att['active']:
        print(f"📡 Telemetry Active! Current Pitch: {att['pitch']}°, Yaw: {att['yaw']}°")
    else:
        print("⚠️ Telemetry not active (did not receive packets). Sending command blindly...")
        
    print("🕹️ Sending GOTO command: Pitch -10°, Yaw 0°...")
    gimbal.goto_angles(pitch=-10.0, yaw=0.0)
    
    print("⏳ Waiting 3 seconds to observe physical movement...")
    time.sleep(3)
    
    att = gimbal.get_attitude()
    print(f"📡 Final Attitude -> Pitch: {att['pitch']}°, Yaw: {att['yaw']}°")
    
    print("🔌 Closing driver...")
    gimbal.close()
    print("✅ Gimbal test complete.")

if __name__ == "__main__":
    print("==========================================")
    print("       C12 HARDWARE ISOLATION TEST        ")
    print("==========================================")
    
    test_camera()
    test_gimbal()
    
    print("\n🏁 ALL TESTS FINISHED.")
