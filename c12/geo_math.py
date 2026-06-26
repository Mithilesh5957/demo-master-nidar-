import math
import numpy as np

def calculate_gps_from_pixel(drone_gps, target_pixel, camera_intrinsics):
    """
    Calculate the precise ground coordinates of a detected object.
    
    Args:
        drone_gps (dict): Dictionary containing drone telemetry:
                          {'lat': float, 'lon': float, 'alt': float, 'heading': float}
        target_pixel (tuple): The (x, y) pixel coordinates of the target center.
        camera_intrinsics (dict): Dictionary containing camera parameters:
                                  {'width': int, 'height': int, 'hfov': float, 'vfov': float}
                                  
    Returns:
        tuple: (latitude, longitude) of the target.
    """
    
    # 1. Extract inputs
    drone_lat = drone_gps.get('lat', 0.0)
    drone_lon = drone_gps.get('lon', 0.0)
    drone_alt = drone_gps.get('alt', 10.0) # Relative altitude in meters
    drone_heading = drone_gps.get('heading', 0.0)
    
    px_x, px_y = target_pixel
    
    img_w = camera_intrinsics.get('width', 640)
    img_h = camera_intrinsics.get('height', 512) # Default thermal resolution
    hfov = camera_intrinsics.get('hfov', 80.0)
    vfov = camera_intrinsics.get('vfov', 60.0)
    
    # 2. Convert pixel to angle offsets
    # Calculate angle per pixel
    deg_per_px_h = hfov / img_w
    deg_per_px_v = vfov / img_h
    
    # Calculate offset from center (assuming center pixel is 0 angle)
    center_x = img_w / 2
    center_y = img_h / 2
    
    offset_x = px_x - center_x
    offset_y = center_y - px_y # Invert Y because pixel Y increases downwards
    
    angle_yaw = offset_x * deg_per_px_h
    angle_pitch = offset_y * deg_per_px_v # Positive is up, negative is down
    
    # 3. Calculate Real-World Distance from Drone to Target (Ground Distance)
    # We assume the camera is pointing generally downwards or forwards-downwards.
    # However, for a simple implementation often used in basic drone drops:
    # We assume the camera is fixed at a certain angle or we are just doing simple vertical projection if looking straight down.
    # BUT, let's implement a standard flat-earth projection assuming the drone is level.
    # If the drone has a gimbal pitch, it should be added to angle_pitch. 
    # Here we assume camera is fixed looking forward/down? Or just straight down?
    # The prompt says "Drone's live GPS... + Target's Pixel Position -> Real-world Lat/Lon".
    # Let's assume a fixed camera angle or that the drone is looking down. 
    # "Simple projection" often assumes camera is mounted at -90 deg (straight down) or -45 deg.
    # Let's assume -45 degrees pitch for a forward-facing camera, or -90 for a belly camera.
    # Given "Scout Drone" and "Thermal", it's likely a gimbal or fixed mount.
    # Let's add a 'gimbal_pitch' parameter or assume -45 for now if not provided, 
    # but strictly speaking we only have what's in arguments.
    # We'll calculate offset distance based on altitude and angle.
    
    # Let's assume the camera is effectively looking "down" but with some forward tilt?
    # Actually, for many of these projects, simple flat earth approx with heading is key.
    
    # Angle of Depression = - (Camera Pitch + Pixel Pitch Offset)
    # Let's assume camera pitch is -45 degrees (looking diagonally down).
    camera_pitch_mount = -45.0 
    
    total_pitch = camera_pitch_mount + angle_pitch
    
    if total_pitch >= 0:
        # Looking at horizon or sky, cannot determine ground intersection simply
        ground_dist = 100.0 # Cap at 100m
    else:
        # tan(pitch) = height / dist -> dist = height / tan(-pitch)
        ground_dist = drone_alt / math.tan(math.radians(-total_pitch))
        
    # 4. Calculate Bearing
    # Bearing to target = Drone Heading + Yaw Offset
    target_bearing_deg = drone_heading + angle_yaw
    target_bearing_rad = math.radians(target_bearing_deg)
    
    # 5. Calculate Delta Lat/Lon (Meters)
    delta_north = ground_dist * math.cos(target_bearing_rad)
    delta_east = ground_dist * math.sin(target_bearing_rad)
    
    # 6. Convert Meters to GPS Coordinates
    # Earth Radius approx 6378137m
    R = 6378137
    
    dLat = delta_north / R
    dLon = delta_east / (R * math.cos(math.radians(drone_lat)))
    
    target_lat = drone_lat + math.degrees(dLat)
    target_lon = drone_lon + math.degrees(dLon)
    
    return target_lat, target_lon

def calculate_distance_to_target(drone_gps, target_lat, target_lon):
    """
    Calculate the 3D distance and ground distance between the drone and the target.
    
    Args:
        drone_gps (dict): {'lat', 'lon', 'alt', ...}
        target_lat (float): Target latitude
        target_lon (float): Target longitude
        
    Returns:
        dict: {'distance_3d': float, 'distance_ground': float} (in meters)
    """
    drone_lat = drone_gps.get('lat', 0.0)
    drone_lon = drone_gps.get('lon', 0.0)
    drone_alt = drone_gps.get('alt', 0.0)
    
    # Haversine for Ground Distance
    R = 6378137 # Earth Radius
    dLat = math.radians(target_lat - drone_lat)
    dLon = math.radians(target_lon - drone_lon)
    a = math.sin(dLat/2) * math.sin(dLat/2) + \
        math.cos(math.radians(drone_lat)) * math.cos(math.radians(target_lat)) * \
        math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    dist_ground = R * c
    
    # 3D Distance (Slant Range)
    dist_3d = math.sqrt(dist_ground**2 + drone_alt**2)
    
    return {'distance_3d': dist_3d, 'distance_ground': dist_ground}
