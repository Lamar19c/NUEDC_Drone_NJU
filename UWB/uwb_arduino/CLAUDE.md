# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Arduino (C++) port of the Python UWB indoor 3D localization system. Runs on a microcontroller instead of a companion computer — reads UWB `$DIST` serial data, computes 3D position via trilateration, smooths with median filters, and outputs NMEA `$GPGGA`/`$GPRMC` sentences over a second serial port directly to the flight controller's GPS input.

**Key difference from the Python version**: Uses dual median filters (distance + position) instead of a 6-state extended Kalman filter. Simpler, no matrix math, no tuning required.

## File Map

```
uwb_arduino/
├── uwb_arduino.ino    # Main sketch — setup(), loop(), serial plumbing
├── uwb_config.h       # All configuration — anchors, pins, baud rates, GPS origin
├── uwb_solver.h       # Trilateration solver + DistanceFilter + PositionFilter + UWB_Parser
├── uwb_nmea.h         # NMEA $GPGGA/$GPRMC sentence generator with local→GPS conversion
└── CLAUDE.md          # This file
```

## Architecture

### Filter Pipeline

```
$DIST,M1,S<id>,<m>
       │
       ▼
  UWB_Parser.feed()        — character-by-character, accumulates per-anchor distances
       │
       ▼
  DistanceFilter.filter()  — per-anchor sliding-window median + jump-limit (MAX_DIST_JUMP)
       │
       ▼
  UWBSolver.solve()        — weighted least-squares trilateration (3D for 4+ anchors, 2D for 3)
       │
       ▼
  PositionFilter.filter()  — sliding-window median over (x, y, z) outputs
       │
       ├─→ velocity = finite difference (dx/dt, dy/dt, dz/dt)
       │
       ▼
  NMEA_Generator.generate() → $GPGGA + $GPRMC → GPS_SERIAL_PORT → flight controller
```

### Serial Layout

| Port | Purpose | Baud | Notes |
|------|---------|------|-------|
| `Serial` (USB) | Debug output | 115200 | Terminal position print |
| UWB input | `$DIST` data from tag | 19200 | Platform-dependent (SoftwareSerial / Serial1 / Serial2) |
| GPS output | NMEA to flight controller | 57600 | Platform-dependent (Serial1 / Serial0) |

### Platform Auto-Detection

`uwb_config.h` uses preprocessor macros to select the right serial hardware:
- **nRF52840** (Nano 33 BLE): only 1 hardware UART → `SoftwareSerial` on D2/D3 for UWB
- **ESP32**: `Serial1` (RX=GPIO5, TX=GPIO6) for UWB, `Serial0` (TX=GPIO43) for GPS NMEA
- **Other**: `Serial2` for UWB, `Serial1` for GPS

## Build & Upload

Open `uwb_arduino.ino` in Arduino IDE (or PlatformIO). All dependencies are headers in the same directory — no external libraries needed beyond `SoftwareSerial` (bundled with Arduino).

1. Select board: **Arduino Nano 33 BLE** or **ESP32 Dev Module**
2. Edit `uwb_config.h`: set `ANCHOR_POSITIONS`, `ANCHOR_COUNT`, `GPS_ORIGIN_*`
3. Upload
4. Open Serial Monitor at 115200 baud to see position output

## Configuration (`uwb_config.h`)

**Must be edited before first use:**

| Define | Purpose | Example |
|--------|---------|---------|
| `ANCHOR_COUNT` | 3 for 2D, 4 for full 3D | `4` |
| `ANCHOR_POSITIONS` | Anchor coordinates (m), X=east Y=north Z=up | Run Python `--calibrate` first |
| `GPS_ORIGIN_LAT/LON/ALT` | GPS reference origin for NMEA | Match your field location |
| `DEFAULT_HEIGHT` | Fixed Z when only 3 anchors | `1.0` |

**Tuning knobs (usually leave at defaults):**

| Define | Default | Effect |
|--------|---------|--------|
| `DIST_WINDOW_SIZE` | 8 | Median window for distance + position filters |
| `MAX_DIST_JUMP` | 0.8 | Single-frame distance jump limit (outlier rejection) |
| `POS_CLAMP_MIN/MAX` | ±10 | Hard position clamp after trilateration |
| `LOOP_INTERVAL_MS` | 20 | Main loop rate (50 Hz target) |
| `NMEA_RATE_HZ` | 5 | NMEA sentence output rate |

## UWB Serial Protocol

```
$DIST,M1,S<id>,<distance_meters>\n
```

- `M1` is always the drone tag (master)
- `S1`–`S4` are anchor IDs (slaves)
- Distance in meters, parsed by `UWB_Parser::parseLine()`
- Invalid distances (<0 or >50m) are marked as -1 and ignored by the solver

## NMEA Output

Same as the Python version — `localToGPS()` converts UWB local (x,y,z) to WGS84 lat/lon using a flat-earth approximation from `GPS_ORIGIN_*`. Two sentences per update cycle at `NMEA_RATE_HZ`:

```
$GPGGA,time,lat,N,lon,E,1,08,1.0,alt,M,0.0,M,,*CK
$GPRMC,time,A,lat,N,lon,E,speed_kn,course,date,,,A*CK
```

Flight controller receives these on its GPS port with `SERIALx_PROTOCOL=5`.

## Design Conventions

- **Single-header modules**: Each `.h` file is self-contained with include guards, no `.cpp` files. Keep them under ~500 lines.
- **No heap in loop**: All objects are statically allocated or created once in `setup()`.
- **Preprocessor over runtime config**: Platform differences resolved at compile time with `#if defined(...)` rather than runtime checks — saves flash and RAM on constrained MCUs.
- **Median always wins**: When choosing between filter approaches, median is preferred over mean for outlier robustness.
