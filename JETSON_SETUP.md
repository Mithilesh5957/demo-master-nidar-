# 🚀 Jetson Scout - StrongSORT Upgrade Guide

## Upgrading Your Existing Scout Script

Your Jetson already has the scout script running. This guide upgrades it to StrongSORT tracking.

---

## ✅ Step 1: Install New Dependencies (DONE)

```bash
pip install boxmot lap
pip install pymavlink paho-mqtt flask ultralytics
pip uninstall opencv-python-headless -y
```

---

## ✅ Step 2: Verify Dependencies (DONE)

```bash
python3 -c "import numpy; print('numpy:', numpy.__version__)"
python3 -c "from boxmot import StrongSORT; print('✅ StrongSORT OK')"
```

---

## 🔄 Step 3: Backup Your Current Script

```bash
# Find your current script
ls -la ~/scout*.py

# Backup it
cp ~/scout_tailscale.py ~/scout_tailscale_backup.py
```

---

## � Step 4: Update the Script

**Option A: SCP from Windows PC**
```bash
# Run this on Windows CMD/PowerShell:
scp e:\mainthings\Nidar-Final\pi_scripts\scout_tailscale.py jetsonorinnano@<JETSON_IP>:~/scout_tailscale.py
```

**Option B: Manual copy-paste**
1. Open `scout_tailscale.py` on Windows (`e:\mainthings\Nidar-Final\pi_scripts\`)
2. Copy all content (Ctrl+A, Ctrl+C)
3. On Jetson: `nano ~/scout_tailscale.py`
4. Delete all (Ctrl+K repeatedly) then paste (right-click)
5. Save (Ctrl+X, Y, Enter)

---

## 🏃 Step 5: Run the Upgraded Script

```bash
python3 ~/scout_tailscale.py
```

### Expected Output:
```
✅ StrongSORT tracking available
🎯 RGB StrongSORT Tracker Ready
🎯 Thermal StrongSORT Tracker Ready
```

---

## ↩️ Rollback (If Issues)

```bash
cp ~/scout_tailscale_backup.py ~/scout_tailscale.py
python3 ~/scout_tailscale.py
```

---

## ✅ New Features After Upgrade

| Feature | Description |
|---------|-------------|
| **Persistent IDs** | Same person = same ID (e.g., ID:5) |
| **Track Confirmation** | 5 frames before GPS marker |
| **Yellow Helmet** | Detects rescue workers |
| **RGB+Thermal Fusion** | Thermal detects, RGB confirms |
