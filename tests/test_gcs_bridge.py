import importlib.util
import inspect
import pathlib
import queue
import sys
import threading
import time
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


def load_bridge():
    mavlink = types.SimpleNamespace(
        MAV_MISSION_ACCEPTED=0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT=6,
        MAV_CMD_NAV_WAYPOINT=16,
        MAV_CMD_COMPONENT_ARM_DISARM=400,
        MAV_CMD_NAV_TAKEOFF=22,
        MAV_CMD_MISSION_START=300,
        MAV_CMD_DO_PAUSE_CONTINUE=193,
        MAV_CMD_DO_SET_MODE=176,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1,
        MAV_FRAME_LOCAL_NED=1,
    )
    mavutil = types.SimpleNamespace(mavlink=mavlink, mavlink_connection=lambda *a, **k: None)
    pymavlink = types.ModuleType("pymavlink")
    pymavlink.mavutil = mavutil
    sys.modules["pymavlink"] = pymavlink

    websockets = types.ModuleType("websockets")
    websockets.serve = object()
    websockets.exceptions = types.SimpleNamespace(ConnectionClosed=RuntimeError)
    sys.modules["websockets"] = websockets

    spec = importlib.util.spec_from_file_location("gcs_bridge_under_test", ROOT / "gcs_bridge.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Message:
    def __init__(self, msg_type, **fields):
        self._type = msg_type
        self.__dict__.update(fields)

    def get_type(self):
        return self._type


class FakeSender:
    def __init__(self):
        self.clears = []
        self.counts = []
        self.items = []

    def mission_clear_all_send(self, *args):
        self.clears.append(args)

    def mission_count_send(self, *args):
        self.counts.append(args)

    def mission_item_int_send(self, *args):
        self.items.append(args)


class FakeConnection:
    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.mav = FakeSender()


class GroundStationRegressionTests(unittest.TestCase):
    def setUp(self):
        self.bridge = load_bridge()

    def html_source(self):
        return (ROOT / "gcs.html").read_text(encoding="utf-8")

    def test_global_position_maps_latitude_to_north_x_and_longitude_to_east_y(self):
        north, east = self.bridge._global_to_ned(32.0001, 118.0002, 32.0, 118.0)
        self.assertAlmostEqual(north, 11.132, places=3)
        self.assertGreater(east, 18.0)

    def test_reader_stops_when_its_connection_is_no_longer_current(self):
        class BlockingConnection:
            def __init__(self):
                self.calls = 0

            def recv_match(self, **kwargs):
                self.calls += 1
                time.sleep(0.01)
                return None

        self.assertEqual(
            list(inspect.signature(self.bridge._mavlink_reader).parameters),
            ["connection", "stop_event"],
        )
        old = BlockingConnection()
        replacement = object()
        self.bridge.mav = old
        stop = threading.Event()
        thread = threading.Thread(target=self.bridge._mavlink_reader, args=(old, stop))
        thread.start()
        time.sleep(0.03)
        self.bridge.mav = replacement
        thread.join(0.2)
        self.assertFalse(thread.is_alive())

    def test_mission_upload_responds_to_requested_sequence_and_waits_for_ack(self):
        connection = FakeConnection()
        self.bridge.mav = connection

        def flight_controller():
            while not connection.mav.clears:
                time.sleep(0.001)
            self.bridge.protocol_queues["MISSION_ACK"].put(Message("MISSION_ACK", type=0))
            while not connection.mav.counts:
                time.sleep(0.001)
            self.bridge.protocol_queues["MISSION_REQUEST"].put(Message("MISSION_REQUEST_INT", seq=1))
            self.bridge.protocol_queues["MISSION_REQUEST"].put(Message("MISSION_REQUEST_INT", seq=0))
            self.bridge.protocol_queues["MISSION_ACK"].put(Message("MISSION_ACK", type=0))

        threading.Thread(target=flight_controller, daemon=True).start()

        waypoints = [
            {"lat": 32.0, "lon": 118.0, "alt": 3, "delay": 0, "yaw": 0},
            {"lat": 32.0001, "lon": 118.0001, "alt": 4, "delay": 1, "yaw": 90},
        ]
        self.assertTrue(self.bridge.do_mission_upload(waypoints, timeout=0.1))
        self.assertEqual([item[2] for item in connection.mav.items], [1, 0])
        self.assertTrue(self.bridge.protocol_queues["MISSION_ACK"].empty())

    def test_default_websocket_host_is_loopback(self):
        parser = self.bridge.build_arg_parser()
        args = parser.parse_args([])
        self.assertEqual(args.ws_host, "127.0.0.1")

    def test_land_reports_mode_change_rejection(self):
        self.bridge.mav = object()
        self.bridge.do_set_mode = lambda mode: False
        self.assertFalse(self.bridge.do_land())

    def test_command_waits_past_in_progress_ack_for_final_result(self):
        command = 400
        self.bridge.protocol_queues["COMMAND_ACK"].put(
            Message("COMMAND_ACK", command=command, result=5)
        )
        self.bridge.protocol_queues["COMMAND_ACK"].put(
            Message("COMMAND_ACK", command=command, result=0)
        )
        self.assertTrue(self.bridge._wait_command_ack(command, timeout=0.1))

    def test_failed_heartbeat_closes_partial_connection(self):
        class PartialConnection:
            def __init__(self):
                self.closed = False

            def wait_heartbeat(self, timeout):
                raise TimeoutError("no heartbeat")

            def close(self):
                self.closed = True

        connection = PartialConnection()
        self.bridge.mavutil.mavlink_connection = lambda *args, **kwargs: connection
        result = self.bridge.connect_mavlink({"type": "sitl"})
        self.assertIn("error", result)
        self.assertTrue(connection.closed)

    def test_closing_one_browser_does_not_disconnect_shared_flight_controller(self):
        html = (ROOT / "gcs.html").read_text(encoding="utf-8")
        self.assertNotIn(
            'window.addEventListener("beforeunload", function() { wsSend("disconnect"); });',
            html,
        )

    def test_status_messages_are_not_inserted_as_html(self):
        html = (ROOT / "gcs.html").read_text(encoding="utf-8")
        start = html.index("function logStatus")
        end = html.index("// ============== Drone position", start)
        self.assertNotIn("innerHTML", html[start:end])

    def test_rejected_command_does_not_mark_transport_disconnected(self):
        html = (ROOT / "gcs.html").read_text(encoding="utf-8")
        start = html.index('} else if (msg.type === "status")')
        end = html.index('} else if (msg.type === "mission_progress")', start)
        self.assertNotIn("mavConnected = false", html[start:end])

    def test_one_broken_websocket_does_not_block_other_clients(self):
        class Client:
            def __init__(self, broken=False):
                self.broken = broken
                self.messages = []

            async def send(self, payload):
                if self.broken:
                    raise OSError("closed")
                self.messages.append(payload)

        broken = Client(broken=True)
        healthy = Client()
        self.bridge.ws_clients = {broken, healthy}
        __import__("asyncio").run(self.bridge._send_to_clients("telemetry"))
        self.assertEqual(healthy.messages, ["telemetry"])
        self.assertNotIn(broken, self.bridge.ws_clients)

    def test_compact_layout_contract_is_present(self):
        html = self.html_source()
        self.assertIn("@media (max-width: 700px)", html)
        self.assertIn('id="compact-controls"', html)
        self.assertIn('id="drawer-backdrop"', html)
        self.assertIn('id="compact-open-controls"', html)
        self.assertIn('id="compact-open-waypoints"', html)
        self.assertIn('aria-expanded="false"', html)

    def test_desktop_panel_structure_is_preserved(self):
        html = self.html_source()
        self.assertEqual(html.count('id="left-panel"'), 1)
        self.assertEqual(html.count('id="center-panel"'), 1)
        self.assertEqual(html.count('id="right-panel"'), 1)

    def test_compact_flight_controls_forward_to_existing_buttons(self):
        html = self.html_source()
        marker = "// ============== Compact layout =============="
        self.assertIn(marker, html)
        start = html.index(marker)
        end = html.index("// ============== Init ==============", start)
        compact_script_segment = html[start:end]
        self.assertIn('forwardCompactAction("btn-arm")', compact_script_segment)
        self.assertIn('forwardCompactAction("btn-takeoff")', compact_script_segment)
        self.assertIn('forwardCompactAction("btn-land")', compact_script_segment)
        self.assertNotIn('wsSend("arm")', compact_script_segment)
        self.assertNotIn('wsSend("takeoff")', compact_script_segment)
        self.assertNotIn('wsSend("land")', compact_script_segment)


if __name__ == "__main__":
    unittest.main()
