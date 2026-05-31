#!/usr/bin/env python3
"""
GCS Bridge — WebSocket <-> MAVLink 桥接进程

启动: python gcs_bridge.py [--ws-port 8765]
"""

import asyncio
import json
import math
import sys
import threading
import argparse

import os

from pymavlink import mavutil
import websockets

# websockets >= 13 uses websockets.asyncio.server; older versions use websockets.serve
try:
    from websockets.asyncio.server import serve as _ws_serve
    WS_USE_CONTEXT = True
except ImportError:
    _ws_serve = websockets.serve
    WS_USE_CONTEXT = False

# Global state
mav = None
telemetry_data = {}
telemetry_lock = threading.Lock()
ws_clients = set()
indoor_mode = False
mission_seq = 0


def load_uwb_config():
    """Load UWB anchor positions from calibration cache or config file."""
    calib_path = os.path.join(os.path.expanduser("~"), ".uwb_calib.json")
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "UWB", "uwb_config.json")

    # Try calibration cache first, then config file
    paths = [calib_path, config_path]
    for path in paths:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            anchors = []
            for key in sorted(data.get("anchors", {}).keys()):
                pos = data["anchors"][key]
                anchors.append({"id": key, "x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])})
            if anchors:
                return {"ok": True, "anchors": anchors, "source": path}
        except Exception:
            continue

    return {"ok": False, "error": "未找到锚点数据，请先运行 python UWB/uwb.py --calibrate"}


def _friendly_socket_error(e, conn_str):
    """Convert raw socket errors to Chinese troubleshooting messages."""
    # errno 10049 = WSAEADDRNOTAVAIL on Windows; errno 99 = EADDRNOTAVAIL on Linux
    if e.errno in (10049, 99):
        return (
            "地址不可达 ({})。\n".format(conn_str) +
            "WiFi数传请确认：① 电脑已连上无人机WiFi热点 ② IP地址输入正确（默认192.168.1.100是示例，请改为实际无人机IP）\n" +
            "串口连接请确认：串口号和波特率正确"
        )
    if e.errno in (10061, 111):  # Connection refused
        return "连接被拒绝 ({}): 飞控可能未运行或端口不对".format(conn_str)
    if e.errno in (10060, 110):  # Timeout
        return "连接超时 ({}): 请检查网络连通性".format(conn_str)
    return "Socket error ({}): {}".format(conn_str, e)


def connect_mavlink(params):
    global mav
    disconnect_mavlink()

    conn_type = params.get("type", "sitl")

    try:
        if conn_type == "serial":
            port = params.get("port", "/dev/ttyUSB0")
            baud = params.get("baud", 57600)
            conn_str = "{} @ {}bps".format(port, baud)
            master = mavutil.mavlink_connection(port, baud=baud)
        elif conn_type == "udp":
            host = params.get("host", "192.168.1.100")
            port = params.get("port", 14550)
            conn_str = "udp:{}:{}".format(host, port)
            master = mavutil.mavlink_connection(conn_str)
        else:  # sitl
            conn_str = "udp:127.0.0.1:14550"
            master = mavutil.mavlink_connection(conn_str)

        # Wait for heartbeat with timeout — SITL may not be running
        print("[MAVLink] Waiting for heartbeat from {} ...".format(conn_str))
        master.wait_heartbeat(timeout=10)
    except OSError as e:
        msg = _friendly_socket_error(e, conn_str)
        print("[MAVLink] {}".format(msg))
        return {"error": msg}
    except Exception as e:
        msg = "Connection failed: {}".format(e)
        print("[MAVLink] {}".format(msg))
        return {"error": msg}

    mav = master

    t = threading.Thread(target=_mavlink_reader, daemon=True)
    t.start()

    print("[MAVLink] Connected: {}, sys={}, comp={}".format(
        conn_str, master.target_system, master.target_component))
    return {"ok": True, "connection": conn_str, "system": master.target_system,
            "component": master.target_component}


def disconnect_mavlink():
    global mav
    if mav:
        try:
            mav.close()
        except Exception:
            pass
        mav = None
        print("[MAVLink] Disconnected")


def _mavlink_reader():
    global telemetry_data, mission_seq

    while mav:
        try:
            msg = mav.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
        except Exception:
            continue

        msg_type = msg.get_type()

        with telemetry_lock:
            if msg_type == "HEARTBEAT":
                telemetry_data["mode"] = mav.flightmode
                telemetry_data["armed"] = (msg.base_mode & 0x80) != 0

            elif msg_type == "GLOBAL_POSITION_INT":
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                rel_alt = msg.relative_alt / 1000.0
                heading = msg.hdg / 100.0 if msg.hdg < 36000 else 0
                telemetry_data["lat"] = lat
                telemetry_data["lon"] = lon
                telemetry_data["alt"] = rel_alt
                telemetry_data["heading"] = heading

                if "home_lat" not in telemetry_data:
                    telemetry_data["home_lat"] = lat
                    telemetry_data["home_lon"] = lon
                dlat = lat - telemetry_data["home_lat"]
                dlon = lon - telemetry_data["home_lon"]
                cos_lat = math.cos(math.radians(telemetry_data["home_lat"]))
                telemetry_data["ned_y"] = dlat * 111320.0
                telemetry_data["ned_x"] = dlon * 111320.0 * cos_lat
                telemetry_data["ned_z"] = -rel_alt

            elif msg_type == "LOCAL_POSITION_NED":
                telemetry_data["ned_x"] = msg.x
                telemetry_data["ned_y"] = msg.y
                telemetry_data["ned_z"] = msg.z
                telemetry_data["vx"] = msg.vx
                telemetry_data["vy"] = msg.vy
                telemetry_data["vz"] = msg.vz

            elif msg_type == "SYS_STATUS":
                telemetry_data["battery_voltage"] = (
                    msg.voltage_battery / 1000.0 if msg.voltage_battery < 65535 else 0
                )
                telemetry_data["battery_percent"] = (
                    msg.battery_remaining if msg.battery_remaining < 101 else 0
                )

            elif msg_type == "GPS_RAW_INT":
                telemetry_data["satellites"] = msg.satellites_visible
                telemetry_data["hdop"] = msg.eph / 100.0 if msg.eph < 65535 else 99

            elif msg_type == "MISSION_CURRENT":
                mission_seq = msg.seq
                telemetry_data["mission_seq"] = msg.seq
                telemetry_data["mission_total"] = telemetry_data.get("mission_total", 0)


def do_set_mode(mode_name):
    if not mav or mode_name not in mav.mode_mapping():
        return False
    mode_id = mav.mode_mapping()[mode_name]
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    return True


def do_arm():
    if not mav:
        return False
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    return True


def do_disarm():
    if not mav:
        return False
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0,
    )
    return True


def do_takeoff(alt_m):
    if not mav:
        return False
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt_m,
    )
    return True


def do_land():
    if not mav:
        return False
    do_set_mode("LAND")
    return True


def do_goto(north, east, down):
    if not mav:
        return False
    type_mask = 0b0000111111111000
    mav.mav.set_position_target_local_ned_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask, north, east, down, 0, 0, 0, 0, 0, 0, 0, 0,
    )
    return True


def _ned_to_global_float(north, east, down):
    """Convert NED meters to GPS float degrees (for mission_item_send)."""
    with telemetry_lock:
        home_lat = telemetry_data.get("home_lat", 0)
        home_lon = telemetry_data.get("home_lon", 0)
    lat_rad = math.radians(home_lat)
    lat = home_lat + north / 111320.0
    lon = home_lon + east / (111320.0 * math.cos(lat_rad))
    alt = -down
    return lat, lon, alt


def do_mission_upload(waypoints):
    """waypoints: [{lat,lon,alt} or {x,y,z}, delay, yaw]"""
    if not mav:
        return False
    count = len(waypoints)
    # Clear existing mission first to avoid stale items
    mav.mav.mission_clear_all_send(mav.target_system, mav.target_component)
    import time as _time
    _time.sleep(0.3)
    mav.mav.mission_count_send(mav.target_system, mav.target_component, count, 0)
    for i, wp in enumerate(waypoints):
        # Prefer GPS lat/lon/alt (float degrees from frontend)
        if "lat" in wp and "lon" in wp and wp["lat"] != 0:
            lat = float(wp["lat"])
            lon = float(wp["lon"])
            alt = float(wp.get("alt", 10))
        else:
            lat, lon, alt = _ned_to_global_float(wp["x"], wp["y"], wp["z"])
        mav.mav.mission_item_int_send(
            mav.target_system, mav.target_component,
            i,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,
            float(wp.get("delay", 0)),
            0, 0,
            float(wp.get("yaw", 0)),
            int(lat * 1e7), int(lon * 1e7), alt,
        )
    with telemetry_lock:
        telemetry_data["mission_total"] = count
    return True


def do_mission_start():
    if not mav:
        return False
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0,
    )
    return True


def do_mission_pause():
    if not mav:
        return False
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
        0, 0, 0, 0, 0, 0, 0, 0,
    )
    return True


async def ws_handler(websocket, path=None):
    """Handle one WebSocket client. path is only used by websockets < 13."""
    ws_clients.add(websocket)
    try:
        addr = websocket.remote_address
    except Exception:
        addr = "unknown"
    print("[WS] Client connected: {}".format(addr))
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                await websocket.send(json.dumps({
                    "type": "status", "msg": "Invalid JSON", "level": "error"
                }))
                continue

            cmd = data.get("cmd", "")
            response = {"type": "status", "msg": "ok"}

            if cmd == "connect":
                result = connect_mavlink(data)
                if result.get("ok"):
                    response = {
                        "type": "connected",
                        "system": result.get("system", 0),
                        "component": result.get("component", 0),
                        "connection": result.get("connection", ""),
                    }
                else:
                    response = {
                        "type": "status",
                        "msg": result.get("error", "Connection failed"),
                        "level": "error",
                    }
            elif cmd == "disconnect":
                disconnect_mavlink()
                response = {"type": "disconnected"}
            elif cmd == "set_mode":
                ok = do_set_mode(data.get("mode", "GUIDED"))
                response = {"type": "status", "msg": "mode={}".format("ok" if ok else "fail")}
            elif cmd == "arm":
                ok = do_arm()
                response = {"type": "status", "msg": "arm={}".format("ok" if ok else "fail")}
            elif cmd == "disarm":
                ok = do_disarm()
                response = {"type": "status", "msg": "disarm={}".format("ok" if ok else "fail")}
            elif cmd == "takeoff":
                ok = do_takeoff(data.get("alt", 3.0))
                response = {"type": "status", "msg": "takeoff={}".format("ok" if ok else "fail")}
            elif cmd == "land":
                ok = do_land()
                response = {"type": "status", "msg": "land={}".format("ok" if ok else "fail")}
            elif cmd == "goto":
                ok = do_goto(data.get("north", 0), data.get("east", 0), data.get("down", 0))
                response = {"type": "status", "msg": "goto={}".format("ok" if ok else "fail")}
            elif cmd == "upload_mission":
                wps = data.get("waypoints", [])
                ok = do_mission_upload(wps)
                response = {"type": "status", "msg": "mission_upload={} ({} wpts)".format(
                    "ok" if ok else "fail", len(wps))}
            elif cmd == "mission_start":
                ok = do_mission_start()
                response = {"type": "status", "msg": "mission_start={}".format("ok" if ok else "fail")}
            elif cmd == "mission_pause":
                ok = do_mission_pause()
                response = {"type": "status", "msg": "mission_pause={}".format("ok" if ok else "fail")}
            elif cmd == "set_param":
                global indoor_mode
                if "indoor_mode" in data:
                    indoor_mode = data["indoor_mode"]
                response = {"type": "status", "msg": "indoor_mode={}".format(indoor_mode)}
            elif cmd == "ping":
                response = {"type": "pong", "mav_connected": mav is not None}
            elif cmd == "get_config":
                response = load_uwb_config()
                response["type"] = "config"
            else:
                response = {"type": "status", "msg": "unknown cmd: {}".format(cmd), "level": "warn"}

            await websocket.send(json.dumps(response))

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        ws_clients.discard(websocket)
        print("[WS] Client disconnected: {}".format(addr))


async def telemetry_broadcast():
    while True:
        await asyncio.sleep(0.25)
        if not ws_clients:
            continue
        with telemetry_lock:
            t = dict(telemetry_data)
        t["type"] = "telemetry"
        t["mission_seq"] = mission_seq
        payload = json.dumps(t)
        stale = set()
        for ws in list(ws_clients):
            try:
                await ws.send(payload)
            except websockets.exceptions.ConnectionClosed:
                stale.add(ws)
        ws_clients.difference_update(stale)


async def run_server(ws_port):
    """Start WebSocket server + telemetry loop."""
    if WS_USE_CONTEXT:
        async with _ws_serve(ws_handler, "0.0.0.0", ws_port):
            print("[WS] Server listening on ws://0.0.0.0:{}".format(ws_port))
            await telemetry_broadcast()
    else:
        server = await _ws_serve(ws_handler, "0.0.0.0", ws_port)
        print("[WS] Server listening on ws://0.0.0.0:{}".format(ws_port))
        await telemetry_broadcast()


def main():
    parser = argparse.ArgumentParser(description="GCS WebSocket Bridge")
    parser.add_argument("--ws-port", type=int, default=8765, help="WebSocket server port")
    args = parser.parse_args()

    print("=" * 50)
    print("GCS Bridge v1.0")
    print("WebSocket server: ws://0.0.0.0:{}".format(args.ws_port))
    print("=" * 50)
    print("Available commands: connect, disconnect, arm, disarm, takeoff,")
    print("  land, set_mode, goto, upload_mission, mission_start, mission_pause")
    print("Waiting for browser to connect on http://localhost:{}".format(args.ws_port))
    print("")

    try:
        asyncio.run(run_server(args.ws_port))
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print("FATAL: {}".format(e), file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
