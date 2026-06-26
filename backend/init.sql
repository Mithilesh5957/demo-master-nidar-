-- Nidar Database Schema

-- Detections table - stores all survivor detections
CREATE TABLE IF NOT EXISTS detections (
    id INT AUTO_INCREMENT PRIMARY KEY,
    lat DECIMAL(10, 7) NOT NULL,
    lon DECIMAL(10, 7) NOT NULL,
    conf DECIMAL(4, 2) DEFAULT 0.00,
    source VARCHAR(50) DEFAULT 'unknown',
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    drone_lat DECIMAL(10, 7),
    drone_lon DECIMAL(10, 7),
    helmet_detected BOOLEAN DEFAULT FALSE,
    track_id INT,
    marked BOOLEAN DEFAULT TRUE,
    INDEX idx_lat_lon (lat, lon),
    INDEX idx_timestamp (timestamp)
);

-- Missions table - stores planned missions
CREATE TABLE IF NOT EXISTS missions (
    id INT AUTO_INCREMENT PRIMARY KEY,
    name VARCHAR(100),
    drone VARCHAR(20) DEFAULT 'scout',
    waypoints JSON,
    altitude DECIMAL(6, 2),
    speed_limit DECIMAL(4, 2),
    status ENUM('planned', 'active', 'completed', 'aborted') DEFAULT 'planned',
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    completed_at DATETIME
);

-- Telemetry logs - optional for historical data
CREATE TABLE IF NOT EXISTS telemetry_logs (
    id INT AUTO_INCREMENT PRIMARY KEY,
    drone VARCHAR(20) NOT NULL,
    lat DECIMAL(10, 7),
    lon DECIMAL(10, 7),
    alt DECIMAL(6, 2),
    heading DECIMAL(5, 2),
    speed DECIMAL(5, 2),
    battery INT,
    mode VARCHAR(20),
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    INDEX idx_drone_time (drone, timestamp)
);
