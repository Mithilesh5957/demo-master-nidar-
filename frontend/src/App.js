import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMapEvents, useMap, Polyline, Polygon } from 'react-leaflet';
import axios from 'axios';
import io from 'socket.io-client';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import './App.css';
import VirtualRemote from './VirtualRemote';
import CameraView from './CameraView';

// --- Icons ---
const scoutIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-blue.png',
    iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34],
});
const deliveryIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
    iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34],
});
const survivorIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-orange.png',
    iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34],
});
const targetIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
    iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34],
});
const homeIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-green.png',
    iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34],
});

// --- Config ---
const IP_ADDRESS = "localhost"; // Use localhost for local Docker development
const SCOUT_IP = "100.80.246.90"; // aerovhyn Pi's Tailscale IP
const socket = io(`http://${IP_ADDRESS}:5000`);

// --- Helpers ---
function isPointInPoly(pt, vs) {
    var x = pt.lat, y = pt.lng;
    var inside = false;
    for (var i = 0, j = vs.length - 1; i < vs.length; j = i++) {
        var xi = vs[i].lat, yi = vs[i].lng;
        var xj = vs[j].lat, yj = vs[j].lng;
        var intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

function generateGrid(polyPoints) {
    if (polyPoints.length < 3) return [];
    let minLat = 90, maxLat = -90, minLon = 180, maxLon = -180;
    polyPoints.forEach(p => {
        if (p.lat < minLat) minLat = p.lat;
        if (p.lat > maxLat) maxLat = p.lat;
        if (p.lng < minLon) minLon = p.lng;
        if (p.lng > maxLon) maxLon = p.lng;
    });

    const step = 0.00015; // ~15-20m
    let grid = [];
    let latSteps = Math.ceil((maxLat - minLat) / step);
    let lonSteps = Math.ceil((maxLon - minLon) / step);

    for (let i = 0; i <= latSteps; i++) {
        let currentLat = minLat + (i * step);
        let row = [];
        for (let j = 0; j <= lonSteps; j++) {
            let currentLon = minLon + (j * step);
            if (isPointInPoly({ lat: currentLat, lng: currentLon }, polyPoints)) {
                row.push({ lat: currentLat, lng: currentLon });
            }
        }
        if (i % 2 === 1) row.reverse(); // Zig-zag
        grid.push(...row);
    }
    return grid;
}

// --- UI Components ---
const Card = ({ children, title, className = "" }) => (
    <div className={`bg-white p-4 rounded-lg shadow-sm border border-slate-200 ${className}`}>
        {title && <h3 className="text-xs font-bold text-slate-400 uppercase tracking-wider mb-3">{title}</h3>}
        {children}
    </div>
);

const Button = ({ children, onClick, variant = "primary", className = "" }) => {
    const variants = {
        primary: "bg-slate-900 text-white hover:bg-slate-800",
        secondary: "bg-white text-slate-700 border border-slate-300 hover:bg-slate-50",
        danger: "bg-red-500 text-white hover:bg-red-600",
        success: "bg-green-600 text-white hover:bg-green-700",
        warning: "bg-orange-500 text-white hover:bg-orange-600"
    };
    return (
        <button
            onClick={onClick}
            className={`px-4 py-2 rounded-md text-sm font-medium transition-colors duration-200 ${variants[variant]} ${className}`}
        >
            {children}
        </button>
    );
};


// --- Map Components ---
function MapClicker({ addWaypoint, mode }) {
    useMapEvents({
        click(e) {
            // Only add points directly if NOT in 'area' mode (or handled differently)
            // But here we reuse the same clicker for both, just storing in different lists state upstairs
            addWaypoint(e.latlng);
        },
    });
    return null;
}

function DroneTracker({ position, followMode }) {
    const map = useMap();
    const [hasInitialized, setHasInitialized] = React.useState(false);

    useEffect(() => {
        // Auto-center on first valid GPS lock
        if (!hasInitialized && position.lat !== 0 && position.lat !== 16.506) {
            map.setView([position.lat, position.lon], 18, { animate: true });
            setHasInitialized(true);
        }
        // Regular follow mode
        else if (followMode && position.lat !== 0 && position.lat !== 16.506) {
            map.setView([position.lat, position.lon], map.getZoom(), { animate: true });
        }
    }, [position, followMode, map, hasInitialized]);
    return null;
}

function App() {
    const [waypoints, setWaypoints] = useState([]); // Mission Points
    const [polygonPoints, setPolygonPoints] = useState([]); // Area Points
    const [drawMode, setDrawMode] = useState('mission'); // 'mission' or 'area'

    const [scout, setScout] = useState({
        lat: 16.506, lon: 80.648, alt: 0, bat: 0, bat_voltage: 0, bat_current: 0, bat_time_min: 0, gps_sats: 0,
        status: "DISARMED", speed: 0, mode: "UNKNOWN",
        heading: 0, dist_to_home: 0, current_wp: -1,
        dist_covered: 0, motors: [0, 0, 0, 0]
    });
    const [delivery, setDelivery] = useState({
        lat: 16.506, lon: 80.648, alt: 0, bat: 0, bat_voltage: 0, bat_current: 0, bat_time_min: 0, gps_sats: 0,
        status: "DISARMED", speed: 0, mode: "UNKNOWN",
        heading: 0, dist_to_home: 0, current_wp: -1,
        dist_covered: 0, motors: [0, 0, 0, 0]
    });

    // SURVIVOR TRACKING - Array of detected survivors
    const [survivors, setSurvivors] = useState([]);  // [{id, lat, lon, conf, source}, ...]
    const [deliveryRoute, setDeliveryRoute] = useState([]);  // Optimized route [[lat,lon], ...]
    const [routeDistance, setRouteDistance] = useState(0);  // Total route distance in meters

    const [view, setView] = useState('dashboard'); // 'dashboard' or 'remote'

    // Altitude conversion: 1m = 3.28084ft, 1ft = 0.3048m
    const METERS_TO_FEET = 3.28084;
    const FEET_TO_METERS = 0.3048;

    const [scoutTakeoffAlt, setScoutTakeoffAlt] = useState(33);  // feet
    const [deliveryTakeoffAlt, setDeliveryTakeoffAlt] = useState(33);  // feet
    const [missionAlt, setMissionAlt] = useState(65);  // feet (20m)
    const [speedLimit, setSpeedLimit] = useState(5);   // m/s

    // Hover altitude for each drone (in feet)
    const [scoutHoverAlt, setScoutHoverAlt] = useState(50);    // feet (~15m)
    const [deliveryHoverAlt, setDeliveryHoverAlt] = useState(65);  // feet (~20m)

    const [activeDrone, setActiveDrone] = useState('scout');
    const [followMode, setFollowMode] = useState(true);

    // Haversine distance for local deduplication
    const haversineDistance = (lat1, lon1, lat2, lon2) => {
        const R = 6371000;
        const dLat = (lat2 - lat1) * Math.PI / 180;
        const dLon = (lon2 - lon1) * Math.PI / 180;
        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
            Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
            Math.sin(dLon / 2) * Math.sin(dLon / 2);
        return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    };

    useEffect(() => {
        socket.on('telemetry_update', (msg) => {
            if (msg.drone === 'scout') {
                setScout(prev => ({ ...prev, ...msg.data, lastHeartbeat: Date.now() }));
            }
            if (msg.drone === 'delivery') {
                setDelivery(prev => ({ ...prev, ...msg.data, lastHeartbeat: Date.now() }));
            }
        });

        socket.on('connect', () => console.log("✅ Socket.IO Connected!"));
        socket.on('disconnect', () => console.log("❌ Socket.IO Disconnected!"));

        // Survivor alert - add to array with deduplication
        socket.on('survivor_alert', (msg) => {
            console.log("🚨 SURVIVOR ALERT:", msg);
            setSurvivors(prev => {
                // Check if already exists within 2m
                const isDuplicate = prev.some(s =>
                    haversineDistance(s.lat, s.lon, msg.lat, msg.lon) < 2
                );
                if (isDuplicate) {
                    console.log("⚠️ Duplicate survivor ignored (frontend)");
                    return prev;
                }
                return [...prev, {
                    id: msg.id || prev.length + 1,
                    lat: msg.lat,
                    lon: msg.lon,
                    conf: msg.conf || 0,
                    source: msg.source || 'unknown'
                }];
            });
        });

        // Clear targets event
        socket.on('targets_cleared', () => {
            console.log("🗑️ Targets cleared");
            setSurvivors([]);
            setDeliveryRoute([]);
        });

        return () => {
            socket.off('telemetry_update');
            socket.off('survivor_alert');
            socket.off('targets_cleared');
        };
    }, []);

    const [statusOverride, setStatusOverride] = useState(null); // { drone: 'scout', status: 'ARMED', timestamp: 0 }

    const sendCommand = (cmd, payload = {}) => {
        // Allow payload to override activeDrone
        const droneTarget = payload.drone || activeDrone;
        const finalPayload = { ...payload, drone: droneTarget };

        // Optimistic UI Update for Arm/Disarm
        if (cmd === 'arm') {
            setStatusOverride({ drone: droneTarget, status: 'ARMED', timestamp: Date.now() });
            // Clear override after 4 seconds (enough time for telemetry to catch up)
            setTimeout(() => setStatusOverride(null), 4000);
        } else if (cmd === 'disarm') {
            setStatusOverride({ drone: droneTarget, status: 'DISARMED', timestamp: Date.now() });
            setTimeout(() => setStatusOverride(null), 4000);
        }

        axios.post(`http://${IP_ADDRESS}:5000/${cmd}`, finalPayload)
            .then(() => console.log(`✅ CMD ${cmd} sent to ${droneTarget}`))
            .catch(() => {
                alert("❌ Command Failed");
                setStatusOverride(null); // Revert on failure
            });
    };

    const importScoutMission = () => {
        if (!scout.ip && !SCOUT_IP) return alert("Scout IP needed");
        const ip = scout.ip || SCOUT_IP;

        axios.get(`http://${ip}:5001/get_delivery_mission`)
            .then(res => {
                if (res.data.count === 0) return alert("⚠️ No detections found on Scout yet.");

                // --- OPTIMIZATION LOGIC (Nearest Neighbor TSP) ---
                const rawPoints = res.data.waypoints
                    .filter(wp => wp.cmd === 'WAYPOINT')
                    .map(wp => ({ lat: wp.lat, lng: wp.lng, visited: false }));

                const optimizedRoute = [];
                // Start from Delivery Drone Home (or current loc)
                let currentPos = { lat: delivery.lat || 16.506, lng: delivery.lon || 80.648 };

                while (rawPoints.some(p => !p.visited)) {
                    // Find nearest unvisited neighbor
                    let nearestIdx = -1;
                    let minDist = Infinity;

                    rawPoints.forEach((p, idx) => {
                        if (!p.visited) {
                            const d = haversineDistance(currentPos.lat, currentPos.lng, p.lat, p.lng);
                            if (d < minDist) {
                                minDist = d;
                                nearestIdx = idx;
                            }
                        }
                    });

                    if (nearestIdx !== -1) {
                        rawPoints[nearestIdx].visited = true;
                        optimizedRoute.push([rawPoints[nearestIdx].lat, rawPoints[nearestIdx].lng]);
                        currentPos = rawPoints[nearestIdx]; // Move to this point
                    }
                }

                setWaypoints(optimizedRoute);

                // AUTO UPLOAD after Optimization
                if (optimizedRoute.length > 0) {
                    // Force 25ft limit for Delivery Drone (Safety)
                    const safeAlt = Math.min(missionAlt, 25);
                    const altMeters = (safeAlt * FEET_TO_METERS).toFixed(1);
                    axios.post(`http://${IP_ADDRESS}:5000/upload_mission`, {
                        waypoints: optimizedRoute,
                        altitude: parseFloat(altMeters),
                        // Force 3m/s limit for Delivery Drone
                        speed_limit: Math.min(speedLimit, 3),
                        drone: activeDrone
                    })
                        .then(() => alert(`✅ Imported & Uploaded ${optimizedRoute.length} Points!`))
                        .catch(() => alert("⚠️ Imported but Upload Failed (Backend Error)"));
                }
            })
            .catch(err => alert("❌ Failed to fetch from Scout: " + err.message));
    };

    // Manual Upload (kept as backup)
    const uploadMission = () => {
        if (waypoints.length === 0) return alert("Select points first!");
        const altMeters = (missionAlt * FEET_TO_METERS).toFixed(1);
        const speedMs = speedLimit;

        axios.post(`http://${IP_ADDRESS}:5000/upload_mission`, {
            waypoints,
            altitude: parseFloat(altMeters),
            speed_limit: speedMs,
            drone: activeDrone
        })
            .then(() => alert("✅ Mission Uploaded!"))
            .catch(() => alert("❌ Backend Error"));
    };

    // --- ROUTE OPTIMIZATION ---
    const arrangeRoute = () => {
        if (survivors.length === 0) {
            alert("No targets detected! Wait for survivor detections.");
            return;
        }

        // Use delivery drone home as starting point
        const home = [delivery.lat, delivery.lon];

        if (home[0] === 0 || home[0] === 16.506) {
            alert("⚠️ Delivery drone GPS not available. Using Scout position.");
            home[0] = scout.lat;
            home[1] = scout.lon;
        }

        console.log("🛣️ Calculating optimal route...");
        axios.post(`http://${IP_ADDRESS}:5000/api/arrange`, { home })
            .then(res => {
                const route = res.data.route;
                setDeliveryRoute(route);
                setRouteDistance(res.data.total_distance_m);
                console.log(`✅ Route calculated: ${route.length} points, ${res.data.total_distance_m}m`);
            })
            .catch(() => alert("❌ Backend Error: Upload Failed"));
    };

    const clearTargets = () => {
        if (survivors.length === 0) return;

        if (window.confirm(`Clear all ${survivors.length} detected targets?`)) {
            axios.post(`http://${IP_ADDRESS}:5000/api/clear_targets`)
                .then(() => {
                    setSurvivors([]);
                    setDeliveryRoute([]);
                    setRouteDistance(0);
                    console.log("🗑️ Targets cleared");
                })
                .catch(() => alert("❌ Failed to clear targets"));
        }
    };

    const startMission = () => {
        if (waypoints.length === 0) return alert("No mission points! Import targets & Upload first.");

        const altMeters = (missionAlt * FEET_TO_METERS).toFixed(1);

        console.log("🚀 Starting Flight Sequence (Assumes Mission Uploaded)...");

        // Step 1: Switch to GUIDED
        axios.post(`http://${IP_ADDRESS}:5000/set_mode`, { mode: 'GUIDED', drone: activeDrone })
            .then(() => {
                // Step 2: ARM
                setTimeout(() => {
                    axios.post(`http://${IP_ADDRESS}:5000/arm`, { drone: activeDrone })
                        .then(() => {
                            // Step 3: TAKEOFF
                            setTimeout(() => {
                                axios.post(`http://${IP_ADDRESS}:5000/takeoff`, { alt: parseFloat(altMeters), drone: activeDrone })
                                    .then(() => {
                                        // Step 4: AUTO Mode (Start Mission)
                                        setTimeout(() => {
                                            axios.post(`http://${IP_ADDRESS}:5000/set_mode`, { mode: 'AUTO', drone: activeDrone })
                                                .then(() => alert("🚀 DELIVERY MISSION STARTED!"))
                                                .catch(() => alert("❌ Failed to switch to AUTO"));
                                        }, 8000); // 8s wait for takeoff (Safety Buffer)
                                    })
                                    .catch(() => alert("❌ Failed to Takeoff"));
                            }, 2000);
                        })
                        .catch(() => alert("❌ Failed to ARM"));
                }, 1000);
            })
            .catch(() => alert("❌ Failed to switch to GUIDED"));
    };



    const handleKMLUpload = (event) => {
        const file = event.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (e) => {
            const text = e.target.result;
            try {
                const parser = new DOMParser();
                const xmlDoc = parser.parseFromString(text, "text/xml");
                const coordinates = xmlDoc.getElementsByTagName("coordinates");
                const newPoints = [];

                for (let i = 0; i < coordinates.length; i++) {
                    const coordText = coordinates[i].textContent.trim();
                    const coordPairs = coordText.split(/\s+/);

                    coordPairs.forEach(pair => {
                        const parts = pair.split(',');
                        if (parts.length >= 2) {
                            // KML is Longitude, Latitude
                            const lng = parseFloat(parts[0]);
                            const lat = parseFloat(parts[1]);
                            if (!isNaN(lat) && !isNaN(lng)) {
                                newPoints.push({ lat, lng });
                            }
                        }
                    });
                }

                if (newPoints.length > 0) {
                    setWaypoints(newPoints);
                    alert(`✅ Loaded ${newPoints.length} points from KML`);
                } else {
                    alert("❌ No valid coordinates found in KML");
                }
            } catch (err) {
                console.error(err);
                alert("❌ Error parsing KML file");
            }
        };
        reader.readAsText(file);
    };

    const handleMapClick = (latlng) => {
        if (drawMode === 'mission') {
            setWaypoints([...waypoints, latlng]);
        } else if (drawMode === 'area') {
            setPolygonPoints([...polygonPoints, latlng]);
        }
    };

    const generatePath = () => {
        if (polygonPoints.length < 3) return alert("Draw a polygon first (3+ points)!");
        const grid = generateGrid(polygonPoints);
        setWaypoints(grid);
        setDrawMode('mission'); // Switch back to visualize points
        alert(`✅ Generated ${grid.length} Search Points!`);
    };

    const ModeBadge = ({ mode }) => (
        <span className="px-2 py-0.5 rounded text-[10px] font-bold bg-blue-100 text-blue-700">
            {mode}
        </span>
    );

    const Badge = ({ status }) => (
        <span className={`px-2 py-0.5 rounded text-[10px] font-bold ${status === 'ARMED' ? 'bg-green-100 text-green-700' : 'bg-slate-100 text-slate-500'}`}>
            {status}
        </span>
    );

    const activeData = activeDrone === 'scout' ? scout : delivery;

    // Handle rescue for the first unrescued survivor
    const handleRescue = (targetSurvivor = null) => {
        const target = targetSurvivor || (survivors.length > 0 ? survivors[0] : null);
        if (!target) return;

        sendCommand('rescue', { lat: target.lat, lon: target.lon });
        setActiveDrone('delivery'); // Switch view to delivery drone
    };

    // Get latest survivor for modal display
    const latestSurvivor = survivors.length > 0 ? survivors[survivors.length - 1] : null;
    const showSurvivorModal = survivors.length === 1; // Only show modal for first detection

    return (
        <div className="flex h-screen w-screen overflow-hidden bg-slate-50">

            {/* SURVIVOR ALERT MODAL - Shows on first detection */}
            {showSurvivorModal && latestSurvivor && (
                <div className="absolute inset-0 z-[2000] bg-black/50 flex items-center justify-center backdrop-blur-sm">
                    <div className="bg-white p-6 rounded-xl shadow-2xl max-w-sm w-full border-l-4 border-orange-500 animate-bounce-in">
                        <div className="flex items-center gap-3 mb-4">
                            <div className="bg-orange-100 p-2 rounded-full">
                                <span className="text-2xl">🚨</span>
                            </div>
                            <div>
                                <h2 className="text-lg font-bold text-slate-900">Survivor Detected!</h2>
                                <p className="text-xs text-slate-500">Location: {latestSurvivor.lat.toFixed(5)}, {latestSurvivor.lon.toFixed(5)}</p>
                            </div>
                        </div>

                        <div className="bg-slate-50 p-3 rounded mb-4 text-xs text-slate-600">
                            <strong>Mission Profile:</strong> Autonomous delivery. Drone will descend to 5m, drop kit, and RTL.
                        </div>

                        <div className="flex gap-3">
                            <button
                                onClick={() => setSurvivors(prev => prev.slice(0, -1))}
                                className="flex-1 py-2 px-4 rounded-lg border border-slate-200 text-slate-600 font-semibold hover:bg-slate-50 text-sm"
                            >
                                Dismiss
                            </button>
                            <button
                                onClick={() => handleRescue(latestSurvivor)}
                                className="flex-1 py-2 px-4 rounded-lg bg-orange-500 text-white font-bold hover:bg-orange-600 shadow-lg shadow-orange-500/30 text-sm animate-pulse"
                            >
                                🚀 LAUNCH RESCUE
                            </button>
                        </div>
                    </div>
                </div>
            )}

            <aside className="w-80 flex-shrink-0 border-r border-slate-200 bg-white flex flex-col z-10">
                <div className="p-4 border-b border-slate-100 flex justify-between items-center">
                    <h1 className="text-lg font-bold text-slate-900 tracking-tight">Fossil</h1>
                </div>
                {/* Navigation */}
                <div className="flex p-2 gap-2 border-b border-slate-100">
                    <button
                        onClick={() => setView('dashboard')}
                        className={`flex-1 text-xs py-2 rounded font-bold ${view === 'dashboard' ? 'bg-slate-900 text-white' : 'bg-slate-100 text-slate-600 hover:bg-slate-200'}`}
                    >
                        🗺️ MAP
                    </button>
                    <button
                        onClick={() => setView('remote')}
                        className={`flex-1 text-xs py-2 rounded font-bold ${view === 'remote' ? 'bg-orange-500 text-white' : 'bg-slate-100 text-slate-600 hover:bg-slate-200'}`}
                    >
                        🎮 REMOTE
                    </button>
                    <button
                        onClick={() => setView('camera')}
                        className={`flex-1 text-xs py-2 rounded font-bold ${view === 'camera' ? 'bg-indigo-500 text-white' : 'bg-slate-100 text-slate-600 hover:bg-slate-200'}`}
                    >
                        👁️ VISION
                    </button>
                </div>

                <div className="flex-1 overflow-y-auto p-4 space-y-4">

                    <Card title="Fleet Status">
                        {/* Scout Card */}
                        <div
                            className={`p-3 rounded-md cursor-pointer border transition-colors mb-2 ${activeDrone === 'scout' ? 'bg-blue-50 border-blue-200' : 'bg-slate-50 border-transparent hover:bg-slate-100'}`}
                            onClick={() => setActiveDrone('scout')}
                        >
                            <div className="flex justify-between items-center mb-2">
                                <div className="flex items-center gap-2">
                                    <div className={`w-2 h-2 rounded-full ${Date.now() - (scout.lastHeartbeat || 0) < 3000 ? 'bg-green-500 animate-pulse' : 'bg-red-500'}`}></div>
                                    <span className="font-semibold text-sm">Scout One</span>
                                </div>
                                <div className="flex gap-1.5">
                                    <ModeBadge mode={scout.mode} />
                                    <Badge status={(statusOverride && statusOverride.drone === 'scout') ? statusOverride.status : scout.status} />
                                </div>
                            </div>
                            <div className="text-xs text-slate-500">
                                <div className="grid grid-cols-2 gap-x-4 gap-y-1.5">
                                    <div className="flex justify-between"><span>Alt:</span> <span className="text-slate-900 font-medium">{scout.alt.toFixed(1)}m / {(scout.alt * METERS_TO_FEET).toFixed(0)}ft</span></div>
                                    <div className="flex justify-between"><span>Speed:</span> <span className="text-slate-900 font-medium">{scout.speed}m/s</span></div>
                                    <div className="flex justify-between"><span>Battery:</span> <span className="text-slate-900 font-medium">{scout.bat}%</span></div>
                                    <div className="flex justify-between"><span>Voltage:</span> <span className="text-slate-900 font-medium">{scout.bat_voltage ? scout.bat_voltage.toFixed(1) + 'V' : '---'}</span></div>
                                    <div className="col-span-2 flex justify-between">🛰️ GPS: <span className={`font-semibold ${scout.gps_sats >= 10 ? 'text-green-600' : scout.gps_sats >= 6 ? 'text-yellow-600' : 'text-red-600'}`}>{scout.gps_sats} sats</span></div>
                                    <div className="flex justify-between"><span>HDG:</span> <span className="text-slate-900 font-medium">{scout.heading}°</span></div>
                                    <div className="flex justify-between"><span>Home:</span> <span className="text-slate-900 font-medium">{scout.dist_to_home?.toFixed(1) || 0}m</span></div>
                                    <div className="flex justify-between"><span>W.P:</span> <span className="text-slate-900 font-medium">{scout.current_wp >= 0 ? `#${scout.current_wp}` : '--'}</span></div>
                                    <div className="flex justify-between"><span>📏 Trip:</span> <span className="text-slate-900 font-medium">{scout.dist_covered}m</span></div>
                                </div>
                                <div className="mt-1">
                                    <div className="text-[10px] text-slate-400 mb-0.5">MOTORS (RPM/%)</div>
                                    <div className="flex gap-1 h-8 items-end">
                                        {(scout.motors || []).map((m, i) => (
                                            <div key={i} className="flex-1 bg-slate-200 rounded-sm relative group overflow-hidden">
                                                <div
                                                    className="absolute bottom-0 w-full bg-orange-500 transition-all duration-300"
                                                    style={{ height: `${Math.min(m / 20, 100)}%` }} // Scale: 2000rpm = 100% or 100% = 100%
                                                />
                                                <div className="absolute inset-0 flex items-center justify-center text-[9px] font-bold text-slate-600 z-10 group-hover:block hidden bg-white/80">
                                                    {m}
                                                </div>
                                            </div>
                                        ))}
                                    </div>
                                </div>
                                {scout.bat_time_min > 0 && (
                                    <div className="pt-1 border-t border-slate-200 grid grid-cols-2 gap-2">
                                        <div>⏱️ <span className="text-blue-600 font-semibold">{scout.bat_time_min.toFixed(1)}min</span></div>
                                        <div>
                                            {scout.rtl_bat_min > 0 && (
                                                <span className={`text-[10px] font-bold ${scout.bat < scout.rtl_bat_min ? 'text-red-600 animate-pulse' : 'text-slate-500'}`}>
                                                    REQ RTL: {scout.rtl_bat_min}%
                                                </span>
                                            )}
                                        </div>
                                    </div>
                                )}
                            </div>
                        </div>

                        {/* Delivery Card */}
                        <div
                            className={`p-3 rounded-md cursor-pointer border transition-colors ${activeDrone === 'delivery' ? 'bg-blue-50 border-blue-200' : 'bg-slate-50 border-transparent hover:bg-slate-100'}`}
                            onClick={() => setActiveDrone('delivery')}
                        >
                            <div className="flex justify-between items-center mb-2">
                                <div className="flex items-center gap-2">
                                    <div className={`w-2 h-2 rounded-full ${Date.now() - (delivery.lastHeartbeat || 0) < 3000 ? 'bg-green-500 animate-pulse' : 'bg-red-500'}`}></div>
                                    <span className="font-semibold text-sm">Delivery Two</span>
                                </div>
                                <div className="flex gap-1.5">
                                    <ModeBadge mode={delivery.mode} />
                                    <Badge status={(statusOverride && statusOverride.drone === 'delivery') ? statusOverride.status : delivery.status} />
                                </div>
                            </div>
                            <div className="text-xs text-slate-500">
                                <div className="grid grid-cols-2 gap-x-4 gap-y-1.5">
                                    <div className="flex justify-between"><span>Alt:</span> <span className="text-slate-900 font-medium">{delivery.alt.toFixed(1)}m / {(delivery.alt * METERS_TO_FEET).toFixed(0)}ft</span></div>
                                    <div className="flex justify-between"><span>Speed:</span> <span className="text-slate-900 font-medium">{delivery.speed}m/s</span></div>
                                    <div className="flex justify-between"><span>Battery:</span> <span className="text-slate-900 font-medium">{delivery.bat}%</span></div>
                                    <div className="flex justify-between"><span>Voltage:</span> <span className="text-slate-900 font-medium">{delivery.bat_voltage ? delivery.bat_voltage.toFixed(1) + 'V' : '---'}</span></div>
                                    <div className="col-span-2 flex justify-between">🛰️ GPS: <span className={`font-semibold ${delivery.gps_sats >= 10 ? 'text-green-600' : delivery.gps_sats >= 6 ? 'text-yellow-600' : 'text-red-600'}`}>{delivery.gps_sats} sats</span></div>
                                    <div className="flex justify-between"><span>HDG:</span> <span className="text-slate-900 font-medium">{delivery.heading}°</span></div>
                                    <div className="flex justify-between"><span>Home:</span> <span className="text-slate-900 font-medium">{delivery.dist_to_home?.toFixed(1) || 0}m</span></div>
                                    <div className="flex justify-between"><span>W.P:</span> <span className="text-slate-900 font-medium">{delivery.current_wp >= 0 ? `#${delivery.current_wp}` : '--'}</span></div>
                                    <div className="flex justify-between"><span>📏 Trip:</span> <span className="text-slate-900 font-medium">{delivery.dist_covered}m</span></div>
                                </div>
                                <div className="mt-1">
                                    <div className="text-[10px] text-slate-400 mb-0.5">MOTORS (RPM/%)</div>
                                    <div className="flex gap-1 h-8 items-end">
                                        {(delivery.motors || []).map((m, i) => (
                                            <div key={i} className="flex-1 bg-slate-200 rounded-sm relative group overflow-hidden">
                                                <div
                                                    className="absolute bottom-0 w-full bg-orange-500 transition-all duration-300"
                                                    style={{ height: `${Math.min(m / 20, 100)}%` }}
                                                />
                                                <div className="absolute inset-0 flex items-center justify-center text-[9px] font-bold text-slate-600 z-10 group-hover:block hidden bg-white/80">
                                                    {m}
                                                </div>
                                            </div>
                                        ))}
                                    </div>
                                </div>
                                {delivery.bat_time_min > 0 && (
                                    <div className="pt-1 border-t border-slate-200 grid grid-cols-2 gap-2">
                                        <div>⏱️ <span className="text-blue-600 font-semibold">{delivery.bat_time_min.toFixed(1)}min</span></div>
                                        <div>
                                            {delivery.rtl_bat_min > 0 && (
                                                <span className={`text-[10px] font-bold ${delivery.bat < delivery.rtl_bat_min ? 'text-red-600 animate-pulse' : 'text-slate-500'}`}>
                                                    REQ RTL: {delivery.rtl_bat_min}%
                                                </span>
                                            )}
                                        </div>
                                    </div>
                                )}
                            </div>
                        </div>
                    </Card>

                    {/* SCOUT CONTROLS */}
                    <Card title="Scout Control">
                        <div className="grid grid-cols-2 gap-2 mb-4">
                            <Button variant="success" onClick={() => sendCommand('arm', { drone: 'scout' })}>ARM (S)</Button>
                            <Button variant="warning" onClick={() => sendCommand('disarm', { drone: 'scout' })}>DISARM</Button>
                            <Button variant="danger" onClick={() => sendCommand('land', { drone: 'scout' })}>LAND</Button>
                            <Button variant="primary" onClick={() => sendCommand('rtl', { drone: 'scout' })}>🏠 RTL</Button>
                        </div>

                        {/* SCOUT TAKEOFF */}
                        <div className="bg-slate-50 p-2 rounded border border-slate-200 mb-2">
                            <label className="text-[10px] uppercase font-bold text-slate-400 block mb-1">Takeoff Altitude (ft)</label>
                            <div className="flex gap-2">
                                <input
                                    type="number"
                                    value={scoutTakeoffAlt}
                                    onChange={(e) => setScoutTakeoffAlt(e.target.value)}
                                    className="w-full text-sm bg-white border border-slate-200 rounded px-2 outline-none focus:border-blue-500"
                                />
                                <Button variant="secondary" className="py-1 px-3 text-xs" onClick={() => {
                                    const altMeters = (scoutTakeoffAlt * FEET_TO_METERS).toFixed(1);
                                    sendCommand('takeoff', { alt: parseFloat(altMeters), drone: 'scout' });
                                }}>GO</Button>
                            </div>
                        </div>

                        {/* SCOUT MODE */}
                        <div className="bg-slate-50 p-2 rounded border border-slate-200 mb-2 flex gap-2">
                            <select
                                id="mode-select-scout"
                                className="w-full text-xs h-8 bg-white border border-slate-200 rounded px-2 outline-none"
                            >
                                <option value="STABILIZE">STABILIZE</option>
                                <option value="LOITER">LOITER</option>
                                <option value="GUIDED">GUIDED</option>
                                <option value="RTL">RTL</option>
                                <option value="AUTO">AUTO</option>
                                <option value="LAND">LAND</option>
                            </select>
                            <Button
                                variant="secondary"
                                className="py-1 px-3 text-xs"
                                onClick={() => sendCommand('set_mode', { mode: document.getElementById('mode-select-scout').value, drone: 'scout' })}
                            >
                                SET
                            </Button>
                        </div>
                    </Card>

                    {/* DELIVERY CONTROLS */}
                    <Card title="Delivery Control">
                        <div className="grid grid-cols-2 gap-2 mb-4">
                            <Button variant="success" onClick={() => sendCommand('arm', { drone: 'delivery' })}>ARM (D)</Button>
                            <Button variant="warning" onClick={() => sendCommand('disarm', { drone: 'delivery' })}>DISARM</Button>
                            <Button variant="danger" onClick={() => sendCommand('land', { drone: 'delivery' })}>LAND</Button>
                            <Button variant="primary" onClick={() => sendCommand('rtl', { drone: 'delivery' })}>🏠 RTL</Button>
                        </div>

                        {/* DELIVERY TAKEOFF */}
                        <div className="bg-slate-50 p-2 rounded border border-slate-200 mb-2">
                            <label className="text-[10px] uppercase font-bold text-slate-400 block mb-1">Takeoff Altitude (ft)</label>
                            <div className="flex gap-2">
                                <input
                                    type="number"
                                    value={deliveryTakeoffAlt}
                                    onChange={(e) => setDeliveryTakeoffAlt(e.target.value)}
                                    className="w-full text-sm bg-white border border-slate-200 rounded px-2 outline-none focus:border-blue-500"
                                />
                                <Button variant="secondary" className="py-1 px-3 text-xs" onClick={() => {
                                    const altMeters = (deliveryTakeoffAlt * FEET_TO_METERS).toFixed(1);
                                    sendCommand('takeoff', { alt: parseFloat(altMeters), drone: 'delivery' });
                                }}>GO</Button>
                            </div>
                        </div>

                        {/* DELIVERY MODE */}
                        <div className="bg-slate-50 p-2 rounded border border-slate-200 mb-2 flex gap-2">
                            <select
                                id="mode-select-delivery"
                                className="w-full text-xs h-8 bg-white border border-slate-200 rounded px-2 outline-none"
                            >
                                <option value="STABILIZE">STABILIZE</option>
                                <option value="LOITER">LOITER</option>
                                <option value="GUIDED">GUIDED</option>
                                <option value="RTL">RTL</option>
                                <option value="AUTO">AUTO</option>
                                <option value="LAND">LAND</option>
                            </select>
                            <Button
                                variant="secondary"
                                className="py-1 px-3 text-xs"
                                onClick={() => sendCommand('set_mode', { mode: document.getElementById('mode-select-delivery').value, drone: 'delivery' })}
                            >
                                SET
                            </Button>
                        </div>
                    </Card>

                    <Card title="System">
                        <div className="mb-4">
                            <Button variant="secondary" className="w-full text-xs" onClick={() => sendCommand('indoor_mode', { drone: 'scout' })}>🛡️ INDOOR (Scout)</Button>
                        </div>
                        <div className="grid grid-cols-2 gap-2 mb-4">
                            <Button variant="success" className="text-xs" onClick={() => sendCommand('start_rc_override', { drone: 'scout' })}>⚠️ OVERRIDE (S)</Button>
                            <Button variant="danger" className="text-xs" onClick={() => sendCommand('stop_rc_override', { drone: 'scout' })}>🛑 RELEASE (S)</Button>
                        </div>
                        <div className="flex items-center justify-between p-2 bg-slate-50 rounded border border-slate-200">
                            <span className="text-xs font-semibold text-slate-700">Auto-Follow Map</span>
                            <button
                                onClick={() => setFollowMode(!followMode)}
                                className={`w-10 h-5 rounded-full relative transition-colors ${followMode ? 'bg-blue-500' : 'bg-slate-300'}`}
                            >
                                <div className={`absolute top-1 w-3 h-3 bg-white rounded-full transition-all ${followMode ? 'left-6' : 'left-1'}`} />
                            </button>
                        </div>


                        {/* START MISSION Button */}
                        <div className="mt-4 p-3 bg-blue-50 border border-blue-200 rounded-lg">
                            <div className="text-xs font-bold text-blue-900 mb-2">🚀 Mission Execution</div>
                            <Button
                                variant="success"
                                onClick={startMission}
                                className="w-full font-bold"
                            >
                                START MISSION (ARM + AUTO)
                            </Button>
                            <div className="text-[9px] text-blue-600 mt-1 text-center mb-2">Arms drone & switches to AUTO mode</div>

                            <Button
                                variant="warning"
                                onClick={() => sendCommand('resume_mission')}
                                className="w-full font-bold text-xs"
                            >
                                ♻️ RESUME FROM LAST WP
                            </Button>
                        </div>
                    </Card>

                    {/* SMART PLANNER */}
                    <Card title="Start Planner">
                        <div className="flex gap-2 mb-2">
                            <Button
                                variant={drawMode === 'mission' ? 'primary' : 'secondary'}
                                onClick={() => setDrawMode('mission')}
                                className="flex-1 text-xs"
                            >
                                Points
                            </Button>
                            <Button
                                variant={drawMode === 'area' ? 'warning' : 'secondary'}
                                onClick={() => setDrawMode('area')}
                                className="flex-1 text-xs"
                            >
                                Area (Box)
                            </Button>
                        </div>

                        {drawMode === 'area' ? (
                            <div className="space-y-2">
                                <div className="text-xs text-slate-500">
                                    Click map to draw search area ({polygonPoints.length} pts).
                                </div>
                                <Button variant="primary" onClick={generatePath} className="w-full">
                                    Generate Grid
                                </Button>
                                <Button variant="secondary" onClick={() => setPolygonPoints([])} className="w-full">
                                    Clear Area
                                </Button>
                            </div>
                        ) : (
                            <div className="space-y-2">
                                <div className="text-xs text-slate-500">
                                    {waypoints.length} waypoints ready.
                                </div>

                                {/* Mission Altitude Input */}
                                <div className="bg-slate-50 p-2 rounded border border-slate-200">
                                    <label className="text-[10px] uppercase font-bold text-slate-400 block mb-1">Mission Altitude (ft)</label>
                                    <input
                                        type="number"
                                        value={missionAlt}
                                        onChange={(e) => setMissionAlt(e.target.value)}
                                        className="w-full text-sm bg-white border border-slate-200 rounded px-2 py-1 outline-none focus:border-blue-500"
                                        min="10"
                                        max={activeDrone === 'delivery' ? "25" : "164"}
                                    />
                                    <div className="text-[9px] text-slate-400 mt-1">{(missionAlt * FEET_TO_METERS).toFixed(1)}m ≈ {missionAlt}ft</div>
                                </div>

                                {/* Speed Limit Input */}
                                <div className="bg-slate-50 p-2 rounded border border-slate-200">
                                    <label className="text-[10px] uppercase font-bold text-slate-400 block mb-1">Speed Limit (m/s)</label>
                                    <input
                                        type="number"
                                        value={speedLimit}
                                        onChange={(e) => setSpeedLimit(e.target.value)}
                                        className="w-full text-sm bg-white border border-slate-200 rounded px-2 py-1 outline-none focus:border-blue-500"
                                        min="2"
                                        max={activeDrone === 'delivery' ? "3" : "15"}
                                        step="0.5"
                                    />
                                    <div className="text-[9px] text-slate-400 mt-1">Max waypoint navigation speed</div>
                                </div>

                                <Button variant="primary" onClick={importScoutMission} className="w-full mb-2">
                                    📥 Import Scout Detections
                                </Button>

                                <Button variant="success" onClick={uploadMission} className="w-full mb-2">
                                    Upload Mission
                                </Button>

                                <Button variant="danger" onClick={startMission} className="w-full font-bold">
                                    🚀 START DELIVERY MISSION
                                </Button>

                                <Button variant="secondary" onClick={() => setWaypoints([])} className="w-full mt-4">
                                    Clear Points
                                </Button>
                                <div className="mt-2 text-xs text-slate-400 text-center">- OR -</div>
                                <label className="block w-full text-center p-2 border border-dashed border-slate-300 rounded cursor-pointer hover:bg-slate-100 mt-2">
                                    <span className="text-xs text-slate-600">📂 Import KML File</span>
                                    <input type="file" accept=".kml" className="hidden" onChange={handleKMLUpload} />
                                </label>
                            </div>
                        )}
                    </Card>

                    {/* DETECTED TARGETS */}
                    <Card title={`Detected Targets (${survivors.length})`}>
                        {survivors.length === 0 ? (
                            <div className="text-xs text-slate-400 text-center py-3">
                                No survivors detected yet.<br />
                                <span className="text-[10px]">Detections will appear here automatically.</span>
                            </div>
                        ) : (
                            <div className="space-y-2">
                                {/* Target List */}
                                <div className="max-h-32 overflow-y-auto space-y-1">
                                    {survivors.map((s, i) => (
                                        <div key={s.id || i} className="flex items-center justify-between bg-red-50 p-2 rounded border border-red-100">
                                            <div className="flex items-center gap-2">
                                                <div className="w-2 h-2 rounded-full bg-red-500"></div>
                                                <span className="text-xs font-semibold text-slate-700">Target #{s.id || i + 1}</span>
                                            </div>
                                            <span className="text-[10px] text-slate-500">{(s.conf * 100).toFixed(0)}%</span>
                                        </div>
                                    ))}
                                </div>

                                {/* Route Info */}
                                {deliveryRoute.length > 0 && (
                                    <div className="bg-green-50 p-2 rounded border border-green-200 text-xs">
                                        <div className="font-semibold text-green-700">✅ Route Calculated</div>
                                        <div className="text-green-600">
                                            {deliveryRoute.length - 2} stops • {routeDistance.toFixed(0)}m total
                                        </div>
                                    </div>
                                )}

                                {/* Action Buttons */}
                                <Button
                                    variant="warning"
                                    onClick={arrangeRoute}
                                    className="w-full"
                                >
                                    🛣️ ARRANGE (Optimize Route)
                                </Button>
                                <Button
                                    variant="secondary"
                                    onClick={clearTargets}
                                    className="w-full"
                                >
                                    🗑️ Clear All Targets
                                </Button>
                            </div>
                        )}
                    </Card>

                    {/* VIDEO FEED */}
                    <Card title="Live Vision">
                        <div className="flex gap-1 overflow-x-auto">
                            {/* WEBCAM ONLY */}
                            <div className="flex-1 bg-black rounded overflow-hidden aspect-video relative">
                                <img
                                    src={`http://${SCOUT_IP}:5001/webcam_feed`}
                                    className="w-full h-full object-cover"
                                    alt="Webcam Feed"
                                    onError={(e) => { e.target.style.display = 'none' }}
                                />
                                <div className="absolute top-1 left-1 text-[10px] text-white bg-blue-600/80 px-1 rounded animate-pulse">LIVE WEBCAM</div>
                            </div>
                        </div>
                    </Card>

                </div>
            </aside >

            {view === 'dashboard' ? (
                <main className="flex-1 relative">
                    <MapContainer center={[16.5062, 80.6480]} zoom={16} zoomControl={false} style={{ height: "100%", width: "100%" }}>
                        <TileLayer
                            attribution='&copy; Google Maps'
                            url="http://mt0.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}"
                            subdomains={['mt0', 'mt1', 'mt2', 'mt3']}
                        />
                        <MapClicker addWaypoint={handleMapClick} mode={drawMode} />

                        <DroneTracker position={activeData} followMode={followMode} />

                        {/* Waypoints Path */}
                        {waypoints.map((wp, i) => <Marker key={i} position={wp} />)}
                        <Polyline positions={waypoints} color="#3b82f6" weight={3} />

                        {/* Area Polygon */}
                        {polygonPoints.length > 0 && (
                            <>
                                {polygonPoints.map((p, i) => <Marker key={`poly-${i}`} position={p} icon={scoutIcon} opacity={0.5} />)}
                                <Polygon positions={polygonPoints} color="orange" dashArray="5, 5" />
                            </>
                        )}

                        <Marker position={[scout.lat, scout.lon]} icon={scoutIcon}>
                            <Popup>
                                <strong>Scout</strong><br />
                                Hdg: {scout.heading}°<br />
                                Dist: {scout.dist_to_home}m<br />
                                WP: {scout.current_wp}
                            </Popup>
                        </Marker>
                        <Marker position={[delivery.lat, delivery.lon]} icon={deliveryIcon}>
                            <Popup>
                                <strong>Delivery</strong><br />
                                Hdg: {delivery.heading}°<br />
                                Dist: {delivery.dist_to_home}m<br />
                                WP: {delivery.current_wp}
                            </Popup>
                        </Marker>

                        {/* SURVIVOR MARKERS - Multiple red markers for all detected targets */}
                        {survivors.map((s, i) => (
                            <Marker key={`survivor-${s.id || i}`} position={[s.lat, s.lon]} icon={targetIcon}>
                                <Popup>
                                    <strong>Target #{s.id || i + 1}</strong><br />
                                    Conf: {(s.conf * 100).toFixed(0)}%<br />
                                    Src: {s.source}<br />
                                    <button
                                        onClick={() => handleRescue(s)}
                                        style={{ marginTop: '5px', padding: '3px 8px', background: '#f97316', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
                                    >
                                        🚀 Rescue
                                    </button>
                                </Popup>
                            </Marker>
                        ))}

                        {/* DELIVERY ROUTE POLYLINE - Green dashed line showing optimized path */}
                        {deliveryRoute.length > 1 && (
                            <>
                                <Polyline
                                    positions={deliveryRoute}
                                    color="#22c55e"
                                    weight={4}
                                    dashArray="10, 10"
                                    opacity={0.8}
                                />
                                {/* Home marker at start of route */}
                                <Marker position={deliveryRoute[0]} icon={homeIcon}>
                                    <Popup>
                                        <strong>🏠 Home Base</strong><br />
                                        Route: {deliveryRoute.length - 2} targets<br />
                                        Distance: {routeDistance.toFixed(0)}m
                                    </Popup>
                                </Marker>
                            </>
                        )}

                    </MapContainer>

                    <div className="absolute top-4 right-4 bg-white/90 backdrop-blur px-3 py-1 rounded shadow text-xs font-mono z-[1000] flex gap-2 items-center">
                        <span>System Ready • Online</span>
                        {drawMode === 'area' && <span className="text-orange-500 font-bold">• Drawing Area</span>}
                    </div>

                    {/* Compass Overlay */}
                    <div className="absolute top-14 right-4 z-[1000]">
                        <div className="w-12 h-12 bg-white/90 backdrop-blur rounded-full shadow-lg border-2 border-slate-300 flex items-center justify-center relative">
                            <span className="text-xs font-bold text-red-600 absolute top-1">N</span>
                            <span className="text-[8px] font-bold text-slate-500 absolute bottom-1">S</span>
                            <span className="text-[8px] font-bold text-slate-500 absolute left-1">W</span>
                            <span className="text-[8px] font-bold text-slate-500 absolute right-1">E</span>
                            {/* Needle */}
                            <div className="w-0.5 h-8 bg-slate-300 rounded-full relative">
                                <div className="w-0.5 h-4 bg-red-500 absolute top-0 rounded-t-full"></div>
                            </div>
                        </div>
                    </div>

                    {/* GPS Coordinates Display */}
                    <div className="absolute bottom-4 left-4 bg-white/95 backdrop-blur px-3 py-2 rounded-lg shadow-lg z-[1000] border border-slate-200">
                        <div className="text-[10px] font-bold text-slate-400 uppercase mb-1">Live GPS Position</div>
                        <div className="space-y-0.5">
                            <div className="text-xs font-mono">
                                <span className="text-blue-600 font-semibold">Scout:</span>
                                <span className="ml-2 text-slate-700">{scout.lat === 0 ? 'No GPS' : `${scout.lat.toFixed(6)}°N, ${scout.lon.toFixed(6)}°E`}</span>
                            </div>
                            <div className="text-xs font-mono">
                                <span className="text-red-600 font-semibold">Deliv:</span>
                                <span className="ml-2 text-slate-700">{delivery.lat === 0 ? 'No GPS' : `${delivery.lat.toFixed(6)}°N, ${delivery.lon.toFixed(6)}°E`}</span>
                            </div>
                        </div>
                    </div>
                </main>
            ) : view === 'camera' ? (
                <main className="flex-1 bg-slate-50 overflow-hidden">
                    <CameraView scoutIp={scout.ip || SCOUT_IP} socket={socket} scoutData={scout} />
                </main>
            ) : (
                <main className="flex-1 bg-slate-50 overflow-hidden">
                    <VirtualRemote socket={socket} activeDrone={activeDrone} />
                </main>
            )}
        </div >
    );
}

export default App;
