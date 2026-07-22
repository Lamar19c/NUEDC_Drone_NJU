#!/usr/bin/env python3
"""
GCS Bridge — WebSocket <-> MAVLink 桥接进程

启动: python gcs_bridge.py [--ws-port 8765]
"""

import asyncio
import json
import math
import queue
import sys
import threading
import time
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
mav_reader_thread = None
mav_stop_event = None
mav_state_lock = threading.RLock()
mav_tx_lock = threading.Lock()
telemetry_data = {}
telemetry_lock = threading.Lock()
ws_clients = set()
indoor_mode = False
mission_seq = 0
protocol_queues = {
    "COMMAND_ACK": queue.Queue(),
    "MISSION_REQUEST": queue.Queue(),
    "MISSION_ACK": queue.Queue(),
}


def _drain_protocol_queue(name):
    q = protocol_queues[name]
    while True:
        try:
            q.get_nowait()
        except queue.Empty:
            return


def _finite_number(value, name, minimum=None, maximum=None):
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("{} must be a number".format(name)) from exc
    if not math.isfinite(number):
        raise ValueError("{} must be finite".format(name))
    if minimum is not None and number < minimum:
        raise ValueError("{} must be >= {}".format(name, minimum))
    if maximum is not None and number > maximum:
        raise ValueError("{} must be <= {}".format(name, maximum))
    return number


def _close_quietly(connection):
    if not connection:
        return
    try:
        connection.close()
    except Exception:
        pass


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
    global mav, mav_reader_thread, mav_stop_event
    disconnect_mavlink()

    conn_type = params.get("type", "sitl")
    master = None

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
        _close_quietly(master)
        msg = _friendly_socket_error(e, conn_str)
        print("[MAVLink] {}".format(msg))
        return {"error": msg}
    except Exception as e:
        _close_quietly(master)
        msg = "Connection failed: {}".format(e)
        print("[MAVLink] {}".format(msg))
        return {"error": msg}

    with mav_state_lock:
        mav = master
        mav_stop_event = threading.Event()
        mav_reader_thread = threading.Thread(
            target=_mavlink_reader,
            args=(master, mav_stop_event),
            daemon=True,
        )
        mav_reader_thread.start()

    print("[MAVLink] Connected: {}, sys={}, comp={}".format(
        conn_str, master.target_system, master.target_component))
    return {"ok": True, "connection": conn_str, "system": master.target_system,
            "component": master.target_component}


def disconnect_mavlink():
    global mav, mav_reader_thread, mav_stop_event
    with mav_state_lock:
        connection = mav
        thread = mav_reader_thread
        stop_event = mav_stop_event
        mav = None
        mav_reader_thread = None
        mav_stop_event = None
    if stop_event:
        stop_event.set()
    if connection:
        _close_quietly(connection)
        if thread and thread is not threading.current_thread():
            thread.join(timeout=2)
        print("[MAVLink] Disconnected")


def _mavlink_reader(connection, stop_event):
    global telemetry_data, mission_seq

    while not stop_event.is_set() and connection is mav:
        try:
            msg = connection.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
        except Exception:
            if stop_event.is_set() or connection is not mav:
                break
            continue

        msg_type = msg.get_type()

        if msg_type == "COMMAND_ACK":
            protocol_queues["COMMAND_ACK"].put(msg)
            continue
        if msg_type in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            protocol_queues["MISSION_REQUEST"].put(msg)
            continue
        if msg_type == "MISSION_ACK":
            protocol_queues["MISSION_ACK"].put(msg)
            continue

        with telemetry_lock:
            if msg_type == "HEARTBEAT":
                telemetry_data["mode"] = connection.flightmode
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
                north, east = _global_to_ned(
                    lat, lon, telemetry_data["home_lat"], telemetry_data["home_lon"]
                )
                telemetry_data["ned_x"] = north
                telemetry_data["ned_y"] = east
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


def _global_to_ned(lat, lon, home_lat, home_lon):
    """Return NED north/east offsets in metres."""
    north = (lat - home_lat) * 111320.0
    east = (lon - home_lon) * 111320.0 * math.cos(math.radians(home_lat))
    return north, east


def _wait_command_ack(command, timeout=3.0):
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        try:
            ack = protocol_queues["COMMAND_ACK"].get(timeout=remaining)
        except queue.Empty:
            return False
        if getattr(ack, "command", None) == command:
            result = getattr(ack, "result", -1)
            if result == 5:  # MAV_RESULT_IN_PROGRESS
                continue
            return result == 0


def _send_command(command, params, timeout=3.0):
    connection = mav
    if not connection:
        return False
    with mav_tx_lock:
        _drain_protocol_queue("COMMAND_ACK")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            command,
            0,
            *params,
        )
        return _wait_command_ack(command, timeout)


def do_set_mode(mode_name):
    connection = mav
    if not connection or mode_name not in connection.mode_mapping():
        return False
    mode_id = connection.mode_mapping()[mode_name]
    return _send_command(
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        [mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0],
    )


def do_arm():
    return _send_command(
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        [1, 0, 0, 0, 0, 0, 0],
    )


def do_disarm():
    return _send_command(
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        [0, 0, 0, 0, 0, 0, 0],
    )


def do_takeoff(alt_m):
    altitude = _finite_number(alt_m, "alt", 0.1, 500.0)
    return _send_command(
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        [0, 0, 0, 0, 0, 0, altitude],
    )


def do_land():
    if not mav:
        return False
    return do_set_mode("LAND")


def do_goto(north, east, down):
    connection = mav
    if not connection:
        return False
    north = _finite_number(north, "north", -10000, 10000)
    east = _finite_number(east, "east", -10000, 10000)
    down = _finite_number(down, "down", -500, 500)
    type_mask = 0b0000111111111000
    connection.mav.set_position_target_local_ned_send(
        0, connection.target_system, connection.target_component,
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


def _validated_waypoint(wp):
    if not isinstance(wp, dict):
        raise ValueError("each waypoint must be an object")
    if "lat" in wp and "lon" in wp and wp["lat"] not in (0, None):
        lat = _finite_number(wp["lat"], "lat", -90, 90)
        lon = _finite_number(wp["lon"], "lon", -180, 180)
        alt = _finite_number(wp.get("alt", 10), "alt", -100, 10000)
    else:
        north = _finite_number(wp.get("x"), "x", -10000, 10000)
        east = _finite_number(wp.get("y"), "y", -10000, 10000)
        down = _finite_number(wp.get("z"), "z", -10000, 10000)
        lat, lon, alt = _ned_to_global_float(north, east, down)
    delay = _finite_number(wp.get("delay", 0), "delay", 0, 3600)
    yaw = _finite_number(wp.get("yaw", 0), "yaw", -360, 360)
    return lat, lon, alt, delay, yaw


def _wait_mission_ack(timeout):
    try:
        ack = protocol_queues["MISSION_ACK"].get(timeout=max(0, timeout))
    except queue.Empty:
        return False
    return getattr(ack, "type", -1) == mavutil.mavlink.MAV_MISSION_ACCEPTED


def do_mission_upload(waypoints, timeout=5.0):
    """waypoints: [{lat,lon,alt} or {x,y,z}, delay, yaw]"""
    connection = mav
    if not connection or not isinstance(waypoints, list) or not (1 <= len(waypoints) <= 500):
        return False
    validated = [_validated_waypoint(wp) for wp in waypoints]
    count = len(validated)
    with mav_tx_lock:
        _drain_protocol_queue("MISSION_REQUEST")
        _drain_protocol_queue("MISSION_ACK")
        connection.mav.mission_clear_all_send(connection.target_system, connection.target_component)
        deadline = time.monotonic() + timeout
        if not _wait_mission_ack(deadline - time.monotonic()):
            return False
        connection.mav.mission_count_send(connection.target_system, connection.target_component, count, 0)

        sent = set()
        while len(sent) < count:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            try:
                request = protocol_queues["MISSION_REQUEST"].get(timeout=remaining)
            except queue.Empty:
                return False
            seq = int(getattr(request, "seq", -1))
            if not 0 <= seq < count:
                return False
            lat, lon, alt, delay, yaw = validated[seq]
            connection.mav.mission_item_int_send(
                connection.target_system, connection.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1,
                delay, 0, 0, yaw,
                int(lat * 1e7), int(lon * 1e7), alt,
            )
            sent.add(seq)

        remaining = max(0, deadline - time.monotonic())
        if not _wait_mission_ack(remaining):
            return False
    with telemetry_lock:
        telemetry_data["mission_total"] = count
    return True


def do_mission_start():
    return _send_command(
        mavutil.mavlink.MAV_CMD_MISSION_START,
        [0, 0, 0, 0, 0, 0, 0],
    )


def do_mission_pause():
    return _send_command(
        mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
        [0, 0, 0, 0, 0, 0, 0],
    )


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
            if not isinstance(data, dict):
                await websocket.send(json.dumps({
                    "type": "status", "msg": "JSON message must be an object", "level": "error"
                }))
                continue

            cmd = data.get("cmd", "")
            response = {"type": "status", "msg": "ok"}

            try:
                response = await _dispatch_command(cmd, data)
            except (TypeError, ValueError, KeyError) as exc:
                response = {"type": "status", "msg": str(exc), "level": "error"}
            except Exception as exc:
                print("[WS] Command {} failed: {}".format(cmd, exc))
                response = {"type": "status", "msg": "Command failed", "level": "error"}

            await websocket.send(json.dumps(response))

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        ws_clients.discard(websocket)
        print("[WS] Client disconnected: {}".format(addr))


async def _run_blocking(function, *args):
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, lambda: function(*args))


async def _dispatch_command(cmd, data):
    global indoor_mode
    if cmd == "connect":
        result = await _run_blocking(connect_mavlink, data)
        if result.get("ok"):
            return {
                "type": "connected",
                "system": result.get("system", 0),
                "component": result.get("component", 0),
                "connection": result.get("connection", ""),
            }
        return {
            "type": "status",
            "msg": result.get("error", "Connection failed"),
            "level": "error",
        }
    if cmd == "disconnect":
        await _run_blocking(disconnect_mavlink)
        return {"type": "disconnected"}

    command_calls = {
        "set_mode": (do_set_mode, (data.get("mode", "GUIDED"),)),
        "arm": (do_arm, ()),
        "disarm": (do_disarm, ()),
        "takeoff": (do_takeoff, (data.get("alt", 3.0),)),
        "land": (do_land, ()),
        "goto": (
            do_goto,
            (data.get("north", 0), data.get("east", 0), data.get("down", 0)),
        ),
        "mission_start": (do_mission_start, ()),
        "mission_pause": (do_mission_pause, ()),
    }
    if cmd in command_calls:
        function, args = command_calls[cmd]
        ok = await _run_blocking(function, *args)
        return {
            "type": "status",
            "msg": "{}={}".format(cmd, "ok" if ok else "fail"),
            "level": "info" if ok else "warn",
        }
    if cmd == "upload_mission":
        waypoints = data.get("waypoints", [])
        ok = await _run_blocking(do_mission_upload, waypoints)
        return {
            "type": "status",
            "msg": "mission_upload={} ({} wpts)".format(
                "ok" if ok else "fail", len(waypoints)
            ),
            "level": "info" if ok else "warn",
        }
    if cmd == "set_param":
        if "indoor_mode" in data:
            indoor_mode = bool(data["indoor_mode"])
        return {"type": "status", "msg": "indoor_mode={}".format(indoor_mode)}
    if cmd == "ping":
        return {"type": "pong", "mav_connected": mav is not None}
    if cmd == "get_config":
        response = load_uwb_config()
        response["type"] = "config"
        return response
    return {"type": "status", "msg": "unknown cmd: {}".format(cmd), "level": "warn"}


async def _send_to_clients(payload):
    stale = set()
    for ws in list(ws_clients):
        try:
            await ws.send(payload)
        except Exception:
            stale.add(ws)
    ws_clients.difference_update(stale)


async def telemetry_broadcast():
    while True:
        await asyncio.sleep(0.25)
        if not ws_clients:
            continue
        with telemetry_lock:
            t = dict(telemetry_data)
        t["type"] = "telemetry"
        t["mission_seq"] = mission_seq
        await _send_to_clients(json.dumps(t))


async def run_server(ws_host, ws_port):
    """Start WebSocket server + telemetry loop."""
    if WS_USE_CONTEXT:
        async with _ws_serve(ws_handler, ws_host, ws_port):
            print("[WS] Server listening on ws://{}:{}".format(ws_host, ws_port))
            await telemetry_broadcast()
    else:
        server = await _ws_serve(ws_handler, ws_host, ws_port)
        print("[WS] Server listening on ws://{}:{}".format(ws_host, ws_port))
        try:
            await telemetry_broadcast()
        finally:
            server.close()
            await server.wait_closed()


def build_arg_parser():
    parser = argparse.ArgumentParser(description="GCS WebSocket Bridge")
    parser.add_argument(
        "--ws-host", default="127.0.0.1", choices=("127.0.0.1", "localhost", "::1"),
        help="Loopback WebSocket bind address",
    )
    parser.add_argument("--ws-port", type=int, default=8765, help="WebSocket server port")
    return parser


def main():
    args = build_arg_parser().parse_args()

    print("=" * 50)
    print("GCS Bridge v1.0")
    print("WebSocket server: ws://{}:{}".format(args.ws_host, args.ws_port))
    print("=" * 50)
    print("Available commands: connect, disconnect, arm, disarm, takeoff,")
    print("  land, set_mode, goto, upload_mission, mission_start, mission_pause")
    print("Waiting for browser to connect on http://localhost:{}".format(args.ws_port))
    print("")

    try:
        asyncio.run(run_server(args.ws_host, args.ws_port))
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print("FATAL: {}".format(e), file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
