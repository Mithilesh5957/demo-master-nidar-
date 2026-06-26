# 🚁 SkyLink.NIDAR (Competition Edition)

**Team:** Fossil (Ravuri Rithesh)
**Objective:** Gold Trophy (1000 Points) - Autonomous Search & Rescue System

---

## 🌟 Key Features (Winning Strategy)

We have implemented a **Fully Autonomous System** to maximize points and minimize penalties.

### 1. 🧠 Smart Scout Grid (Autonomous)
*   **Grid Generator:** Draw a polygon on the map, and the system automatically generates a "Lawnmower" search pattern.
*   **0% Manual Effort:** No need to click 50 individual waypoints.
*   **Point Strategy:** Guarantees 100% area coverage for detecting survivors.

### 2. ⚡ Auto-Dispatch Rescue (Speed Multiplier)
*   **Instant Reaction:** When the Scout detects a survivor, a "🚨 LAUNCH RESCUE" popup appears instantly on the GCS.
*   **One-Click Launch:** A single click deploys the Delivery drone to the exact coordinates.
*   **Zero Latency:** Saves critical seconds compared to manual coordinate entry.

### 3. 🎯 Precision Drop Maneuver (Rule Compliance)
*   **Safety First:** Delivery drone cruises fast at **15m**.
*   **The Dive:** Upon reaching the target, it autonomously descends to **5m** (< 20ft rule).
*   **Accuracy:** Drops the payload from low altitude to ensure Zone A accuracy (1.5m radius), then ascends and returns home.

---

## 🛠️ System Architecture

*   **Network:** Tailscale VPN (Mesh Network) - Connects Laptop and Pis across any 4G/WiFi.
*   **GCS (Laptop):** React Frontend + Python/Flask Backend (Dockerized).
*   **Edge Nodes (Pis):** Raspberry Pi 4 running Python MAVLink Scripts with Mavlink Router.

---

## 🚀 Setup Guide

### Phase 1: Laptop (Mission Control)

1.  **Tailscale IP Check:**
    *   Open `frontend/src/App.js` and `pi_scripts/*.py`.
    *   Ensure `BROKER_IP` matches your Laptop's Tailscale IP (Currently: `100.125.45.22`).

2.  **Start GCS:**
    ```powershell
    docker-compose up --build
    ```

3.  **Access Dashboard:**
    *   Open Chrome: `http://localhost:3000`
    *   You will see the **SkyLink.NIDAR** interface.

### Phase 2: Raspberry Pi (Drones)

**Prerequisites:**
*   Tailscale installed and connected (`tailscale up`).
*   Mavlink Router running (`sudo systemctl start mavlink-router`).

**1. Scout Drone:**
*   Copy `scout_tailscale.py` to the Pi.
*   Run:
    ```bash
    python3 scout_tailscale.py
    ```
    *Expect: "✅ Scout Online" and "✅ Connected to GCS MQTT Broker"*

**2. Delivery Drone:**
*   Copy `delivery_tailscale.py` to the Pi.
*   Run:
    ```bash
    python3 delivery_tailscale.py
    ```
    *Expect: "✅ Delivery Drone Online" and "✅ Connected to GCS MQTT Broker"*

---

## 🎮 How to Run a Mission

1.  **Map Area:** On the website, switch to **"Area (Box)"** mode and draw the search boundary.
2.  **Generate Path:** Click **"Generate Grid"**. The system creates the flight plan.
3.  **Upload:** Click **"Points"** -> **"Upload Mission"**. The Scout drone is now programmed.
4.  **ARM & Takeoff:** Click **ARM** then **GO** (Takeoff). The Scout starts searching.
5.  **Wait for Detection:** Watch the screen. When a survivor is found...
6.  **LAUNCH:** A popup appears. Click **"🚀 LAUNCH RESCUE"**.
7.  **Win:** The Delivery drone handles the rest. Sit back and accept the trophy. 🏆

---

## 🔧 Troubleshooting (Field Guide)

### ❌ Error: "No heartbeat yet..." (On Pi)
*   **Cause:** The Pi cannot talk to the Cube (Flight Controller).
*   **Fix 1 (Hardware):** Unplug and Replug the USB cable connecting Pi <-> Cube.
*   **Fix 2 (Software):** Restart the router service:
    ```bash
    sudo systemctl restart mavlink-router
    ```
*   **Fix 3 (Verify):** Check if the Cube is detected:
    ```bash
    ls /dev/ttyACM*
    ```
    *(If no result, the USB cable is bad or Cube is dead).*

### ❌ Error: "red dot" (Offline) on Website
*   **Cause:** MAVLink is working, but MQTT is blocked.
*   **Fix:**
    1.  Check Tailscale on Laptop: `tailscale ip` (Must be `100.125.45.22`).
    2.  Check Firewall: Disable Windows Firewall temporarily.
    3.  Restart Docker: `docker-compose restart`

### ❌ Error: "backend Exited (1)"
*   **Cause:** Crash in `server.py`.
*   **Fix:** Run `docker logs nidarfinal-backend-1` to see the error. Usually a syntax error or missing `allow_unsafe_werkzeug=True`.
