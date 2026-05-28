# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This workspace contains two subsystems for the 2025 NUEDC drone competition:

1. **UWB Indoor Positioning** (`UWB/`) — Trilateration-based 3D localization using ultra-wideband anchor ranging, with EKF smoothing and multi-channel output (terminal, UDP for GCS, MAVLink GPS_INPUT for flight controller).
2. **MAVLink Mission Script** (`drone_mission.py`) — Educational example demonstrating ArduCopter control via pymavlink: connect → arm → takeoff → fly rectangle → land.

## Architecture

### UWB Positioning Pipeline

```
Serial (UWB anchors) → SerialReader → UWBSolver (least-squares + EKF) → OutputRouter
                                                                          ├── terminal
                                                                          ├── UDP JSON → GCS
                                                                          └── MAVLink GPS_INPUT → flight controller
```

**Serial protocol**: `$DIST,M1,S<id>,<distance_meters>` — one line per anchor reading. `SerialReader` accumulates distances across anchors and returns an ordered list `[d1, d2, d3, d4]` once at least 3 anchors have reported.

**Solver modes**: `UWBSolver` auto-selects between 3D (4+ anchors, four-sphere intersection with least-squares) and 2D (3 anchors, fixed-height xy-only). Both modes feed through a 6-state EKF `[x, y, z, vx, vy, vz]`.

**Output routing**: `OutputRouter` sends position/velocity to up to 3 channels simultaneously. The MAVLink channel converts local NED → GPS via `local_to_gps()` and injects `GPS_INPUT` messages over UDP — the flight controller treats these as real GPS readings. Configure channels in `uwb_config.json` under `output.*`.

### MAVLink Mission Script

Linear procedural script — no classes, just functions. Key NED coordinate convention: X=North, Y=East, Z=Down (so altitude = negative Z). `goto_local_ned()` sends `SET_POSITION_TARGET_LOCAL_NED` with a type_mask that ignores velocity/acceleration/yaw. `wait_until_reached()` polls `LOCAL_POSITION_NED` until within tolerance.

## Commands

```bash
# GCS Bridge (WebSocket ↔ MAVLink)
python gcs_bridge.py

# UWB localization — first deploy: calibrate
python UWB/uwb.py --calibrate

# UWB localization — daily use (auto-discover serial + load calibration)
python UWB/uwb.py

# UWB with manual overrides
python UWB/uwb.py --port COM3 --baud 19200 --default-height 1.2
python UWB/uwb.py --mavlink-host 192.168.1.100:14550

# Drone mission (edit CONN/BAUD at bottom of file first)
python drone_mission.py
```

## Dependencies

```
pip install pymavlink websockets pyserial numpy
```

`pymavlink` is optional for the UWB system — only needed if `mavlink.enabled: true` in config.
`websockets` is required for the GCS bridge.

## Configuration

`UWB/uwb_config.json` is now **optional** — all fields auto-detected. Calibration results cached to `~/.uwb_calib.json`.

Priority: CLI args > uwb_config.json > ~/.uwb_calib.json > auto-detect

| Key | Purpose | Auto-fallback |
|-----|---------|---------------|
| `serial_port` / `baud_rate` | UWB serial connection | Enumeration with `$DIST` probe |
| `anchors` | `{"S1": [x,y,z], ...}` — anchor positions (meters) | `--calibrate` three-point calibration |
| `output.terminal` | Print to stdout | Always on |
| `output.udp_broadcast` | UDP JSON to GCS (host/port) | Off by default |
| `output.mavlink` | MAVLink GPS_INPUT to FC (host/port) | Off by default |
| `ekf_origin_*` | E7-format GPS origin | Read from FC HOME_POSITION |
