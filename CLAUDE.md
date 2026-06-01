# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Project Overview

NUEDC 2026 drone competition preliminary work — three subsystems working together:

| Subsystem | Entry Point | Role |
|-----------|------------|------|
| **Ground Control Station** | `gcs_bridge.py` + `gcs.html` | Browser-based GCS with WebSocket ↔ MAVLink bridge, dual indoor/outdoor mode |
| **UWB Indoor Positioning** | `UWB/uwb.py` | Trilateration-based 3D localization, EKF smoothing, NMEA GPS emulation to flight controller |
| **Mission Script** | `drone_mission_rectangle.py` | Educational ArduCopter control example (arm → takeoff → fly rectangle → land) |

## File Map

```
main/
├── gcs_bridge.py                  # Python WebSocket ↔ MAVLink bridge (asyncio)
├── gcs.html                       # Browser GCS frontend (Canvas + Leaflet)
├── drone_mission_rectangle.py     # MAVLink mission script example
├── UWB/
│   ├── uwb.py                     # UWB localizer — all-in-one (zero-config, 3 calibration modes)
│   └── uwb_config.json            # Optional config overrides (all fields auto-detected)
├── docs/
│   └── hardware-deployment.md     # Pi-based deployment, wiring, systemd, UWB anchor layout
├── README.md                      # Quick-start and feature overview
├── DEVLOG.md                      # Development log
└── CLAUDE.md                      # This file
```

## Architecture

### System Data Flow

```
UWB Anchors ──[$DIST]──→ uwb.py ──UDP JSON──→ gcs_bridge.py ──WebSocket──→ gcs.html
                              ──NMEA $GPGGA/$GPRMC──→ Flight Controller (ArduPilot GPS port)
                                                │
                                                │ MAVLink (serial/UDP/SITL)
                                                ↓
                                          gcs_bridge.py ←── telemetry + commands
```

### GCS Bridge (`gcs_bridge.py`)

MAVLink ↔ WebSocket relay. No classes — procedural asyncio architecture.

**Connection**: `connect_mavlink(params)` supports `serial` (port/baud), `udp` (host:port), or `sitl` modes. Returns `{"ok": True}` or `{"error": "..."}`.

**Commands** (JSON over WebSocket, field `cmd`):

| Command | Action |
|---------|--------|
| `connect` / `disconnect` | MAVLink session management |
| `arm` / `disarm` | Motor arm/disarm (MAV_CMD_COMPONENT_ARM_DISARM) |
| `takeoff` | Takeoff to altitude (MAV_CMD_NAV_TAKEOFF) |
| `land` | Switch to LAND mode |
| `set_mode` | Change flight mode (GUIDED/AUTO/LOITER/RTL/LAND/STABILIZE) |
| `goto` | NED position command (SET_POSITION_TARGET_LOCAL_NED) |
| `upload_mission` | Upload waypoint list (MISSION_ITEM_INT) |
| `mission_start` / `mission_pause` | Mission control |
| `ping` | Health check (returns bridge + MAVLink status) |
| `get_config` | Returns UWB anchor positions for NED canvas display |

**Telemetry broadcast**: Background task pushes 250ms snapshots to all WebSocket clients. Key fields: `lat/lon/alt`, `ned_x/ned_y/ned_z`, `heading`, `mode`, `armed`, `battery`, `gps_fix`, `home_lat/home_lon`.

**UWB integration**: `load_uwb_config()` reads anchor positions from `~/.uwb_calib.json` → `UWB/uwb_config.json` for the NED canvas.

### GCS Frontend (`gcs.html`)

Single-file HTML/JS browser application. Three-column Flexbox layout:
- **Left (280px)**: Connection config (serial/WiFi/SITL), indoor/outdoor toggle, flight controls
- **Center (flex)**: Map area — NED Canvas (indoor) or Leaflet tiles (outdoor), telemetry status bar
- **Right (280px)**: Waypoint table, mission upload/start/pause controls

**Dual coordinate mode**: Indoor uses NED grid (Canvas with home-centered origin, 1m grid, zoom 10–300 px/m). Outdoor uses GPS with Leaflet (multiple tile sources: Gaode, Bing, Google, Tencent, OSM).

**Waypoint system**: Click-to-add on map + editable table, bidirectional sync. Per-waypoint params: altitude, hover time, yaw. Upload via MISSION_ITEM_INT or step-through execution.

**Flight trail**: Purple polyline tracking drone position, 120s TTL auto-cleanup.

**Follow mode**: Auto-pan map to keep drone centered. Manual drag disables; toggle button re-enables.

### UWB Localizer (`UWB/uwb.py`)

Single-file, zero-config design. Key classes:

| Class | Purpose |
|-------|---------|
| `AnchorConfig` | Config loading with platform-aware auto-detection fallback |
| `SerialReader` | UWB serial protocol parser (`$DIST,M1,S<id>,<m>`) |
| `UWBSolver` | Trilateration solver (3D for 4+ anchors, 2D for 3 anchors) |
| `UWBEKF` | 6-state extended Kalman filter `[x,y,z,vx,vy,vz]` |
| `OutputRouter` | Fan-out to terminal print, UDP JSON broadcast, serial NMEA GPS emulation |

**Serial protocol**: `$DIST,M1,S<id>,<distance_meters>` — one line per anchor. `SerialReader` buffers partial lines, accumulates 3+ anchors, returns ordered `[d1, d2, d3, d4]`.

**Auto-discovery** (`auto_discover_uwb`): Enumerates serial ports, probes 7 baud rates (921600→9600), identifies UWB by `$DIST` response. Falls back to platform-appropriate default (`COM3` on Windows, `/dev/ttyUSB0` on Linux).

**GPS emulation output**: Sends standard NMEA sentences (`$GPGGA` + `$GPRMC`) over serial to the flight controller's GPS port. The FC receives them as if from a real GPS module — no MAVLink, no pymavlink dependency. Baud rate defaults to 57600 (matching typical GPS module rates). The GPS port is input-only, so EKF origin must be configured manually in `uwb_config.json`.

**Calibration** (`--calibrate`): Interactive 3-mode menu:
1. **Step-based**: Move tag origin→X→Y, enter actual measured distance (any distance, not hardcoded 1m)
2. **Known-points**: Place tag at N field positions with known coordinates, least-squares trilateration solves anchor positions
3. **Direct entry**: Type anchor coordinates manually (skip measurement)

Non-interactive equivalents: `--cal-points "x,y,z;..."` and `--set-anchors "x,y,z;..."`.

**Calibration persistence**: Results saved to `uwb_config.json` directly (primary), with `~/.uwb_calib.json` as backup.

**Output channels**: Terminal (always on), UDP JSON broadcast (`127.0.0.1:14550` default), NMEA GPS emulation (serial NMEA sentences to FC GPS port). NMEA is output-only — HOME_POSITION cannot be auto-read; configure `ekf_origin_lat/lon/alt` manually in config. Default baud 38400, matching u-blox NEO-M9N default.

### Mission Script (`drone_mission_rectangle.py`)

Procedural ArduPilot control example by 李希才 (NJU). Functions: `connect_vehicle()` → `arm_and_takeoff()` → `goto_local_ned()` → `land_vehicle()`. Flies a rectangle pattern. Edit `CONN`/`BAUD` at bottom of file before running.

NED convention: X=North, Y=East, Z=Down (altitude = -Z). Supports USB serial, telemetry radio, and SITL UDP.

## Commands

```bash
# ── GCS ──
python gcs_bridge.py                           # Start bridge on :8765
python gcs_bridge.py --ws-port 9000            # Custom WebSocket port

# ── UWB ──
python UWB/uwb.py --calibrate                  # Interactive calibration (3 modes)
python UWB/uwb.py                              # Daily use (auto-discover + load config)
python UWB/uwb.py --set-anchors "0,0,1.5;2,0,1.5;0,2,1.5;2,2,1.5"
python UWB/uwb.py --calibrate --cal-points "0,0,0;1.5,0,0;0,2,0"
python UWB/uwb.py --port /dev/ttyUSB0 --baud 57600 --default-height 1.2
python UWB/uwb.py --gps-emu-serial /dev/ttyS6 --gps-emu-baud 57600

# ── Mission ──
python drone_mission_rectangle.py              # Edit CONN/BAUD at file bottom first
```

## Dependencies

```bash
pip install pymavlink websockets pyserial numpy
```

- `pymavlink` — required for GCS bridge + mission script; NOT needed for UWB (uses raw pyserial for NMEA output)
- `websockets` — required for GCS bridge (v13+ for `asyncio.server`, graceful fallback for older)
- `pyserial` — required for UWB serial + GCS serial connection
- `numpy` — required for UWB solver + EKF

## Configuration

### UWB (`UWB/uwb_config.json`)

All fields optional. Priority chain: **CLI args > uwb_config.json > ~/.uwb_calib.json > auto-detect**

| Key | Purpose | Auto-fallback |
|-----|---------|---------------|
| `serial_port` / `baud_rate` | UWB serial connection | Platform-aware enumeration with `$DIST` probe |
| `anchors` | `{"S1":[x,y,z], ...}` — anchor positions (m) | Calibration menu (3 modes) |
| `output.terminal` | Print to stdout | Always on |
| `output.udp_broadcast` | UDP JSON (`enabled`, `host`, `port`) | Off by default |
| `output.gps_emulation` | NMEA GPS emulation (`enabled`, `serial_port`, `serial_baud`) — serial NMEA to FC GPS port | Off by default |
| `ekf_origin_lat/lon/alt` | E7-format GPS reference origin | Must be manually configured for NMEA mode |
## Platform Notes

- **Windows**: Serial ports `COMx`, no permission issues. Default fallback `COM3`.
- **Linux (Pi/Ubuntu)**: Serial ports `/dev/ttyUSBx` or `/dev/ttyAMAx`. Requires `dialout` group: `sudo usermod -aG dialout $USER`. Default fallback `/dev/ttyUSB0`.
- **Python ≥ 3.8** required (`from __future__ import annotations` + f-strings).
- **SITL testing**: `sim_vehicle.py -v ArduCopter --console --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551`. Bridge on :14550, mission script on :14551.

## Design Conventions

- **No premature abstraction**: Single-file modules preferred over package splits unless the file exceeds ~500 lines. Three identical patterns are better than one wrong abstraction.
- **Zero-config first**: Auto-detect everything, CLI args for overrides, config file for persistence. Never require manual JSON editing.
- **Comments document WHY, not WHAT**: Well-named functions and variables speak for themselves.
- **Calibration math**: `_solve_anchors_from_measurements()` is the shared core — given N known tag positions + distances, solves for each anchor's 3D coordinate via least-squares. Used by both step-based and known-points calibration.
- **Encoding**: Always `open(..., encoding="utf-8")` — Windows defaults to GBK otherwise.
- **UWB serial protocol**: `$DIST,M1,S<id>,<distance_meters>` with `\n` delimiter. Master ID is always M1 (the drone tag). Anchor IDs are S1–S4.
