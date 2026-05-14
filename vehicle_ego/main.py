import json
import os
import threading
import time
from typing import Any

from flask import Flask, jsonify, render_template
from flask_socketio import SocketIO
import paho.mqtt.client as mqtt


MAIN_BROKER_HOST = os.getenv("MAIN_BROKER_HOST", "main-broker")
MAIN_BROKER_PORT = int(os.getenv("MAIN_BROKER_PORT", "1883"))
EGO_BROKER_HOST = os.getenv("EGO_BROKER_HOST", "ego-broker")
EGO_BROKER_PORT = int(os.getenv("EGO_BROKER_PORT", "1883"))
LEAD_BROKER_HOST = os.getenv("LEAD_BROKER_HOST", "lead-broker")
LEAD_BROKER_PORT = int(os.getenv("LEAD_BROKER_PORT", "1883"))
TOPIC_WORLD_EGO = os.getenv("WORLD_TOPIC_EGO", "world/pos/ego")
TOPIC_CPM_OUT = os.getenv("CPM_OUT_TOPIC", "vanetza/out/cpm")
TOPIC_CAM_OUT = os.getenv("CAM_OUT_TOPIC", "vanetza/out/cam")
TOPIC_CAM_TIME = os.getenv("CAM_TIME_TOPIC", "vanetza/time/cam")
UI_PORT = int(os.getenv("UI_PORT", "8080"))
STALE_SECONDS = float(os.getenv("STALE_SECONDS", "3.0"))
CAM_TIME_MATCH_WINDOW = 3.0
BASE_LAT = float(os.getenv("WORLD_BASE_LAT", "40.628300"))
BASE_LON = float(os.getenv("WORLD_BASE_LON", "-8.654400"))


app = Flask(__name__, template_folder="templates", static_folder="static")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

state_lock = threading.Lock()
state: dict[str, Any] = {
    "self": {"x": 20.0, "y": 0.0, "heading": 0.0, "speed": 0.0, "updated_at": 0.0},
    "objects": {},
    "metrics": {"cam_rate_hz": 0.0, "last_cam_age_sec": None, "last_cam_latency_sec": None, "stale": True},
}
_cam_counter = 0
_cam_window_start = time.time()
_last_wave_timestamp: float | None = None


def meters_from_lon_delta(delta_lon: float, latitude: float) -> float:
    import math

    return delta_lon * 111320.0 * math.cos(math.radians(latitude))


def meters_from_lat_delta(delta_lat: float) -> float:
    return delta_lat * 111320.0


def emit_state() -> None:
    with state_lock:
        payload = json.loads(json.dumps(state))
    socketio.emit("state_update", payload)


def on_world_ego(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        with state_lock:
            state["self"]["x"] = float(payload["x"])
            state["self"]["y"] = float(payload.get("y", 0.0))
            state["self"]["heading"] = float(payload.get("heading", 0.0))
            state["self"]["speed"] = float(payload.get("speed", 0.0))
            state["self"]["updated_at"] = time.time()
    except (ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"world ego parse error: {exc}")
        return

    emit_state()


def parse_cam_payload(payload: dict[str, Any]) -> tuple[float, float, float, float, Any]:
    # Support both raw CAM JSON and wrapper formats like {"fields": {"header": ..., "cam": ...}}
    cam_root = payload.get("fields", {}).get("cam", payload)
    cam_params = cam_root.get("camParameters", {})
    basic = cam_params.get("basicContainer", {})
    ref_pos = basic.get("referencePosition", {})
    hf = cam_params.get("highFrequencyContainer", {}).get("basicVehicleContainerHighFrequency", {})

    lat = float(ref_pos["latitude"])
    lon = float(ref_pos["longitude"])
    heading = float(hf["heading"]["headingValue"]) if isinstance(hf.get("heading"), dict) else 0.0
    speed = float(hf["speed"]["speedValue"]) if isinstance(hf.get("speed"), dict) else 0.0
    station_id = (
        payload.get("fields", {}).get("header", {}).get("stationId")
        or payload.get("stationID")
        or payload.get("stationId")
        or payload.get("itsPduHeader", {}).get("stationId")
        or payload.get("header", {}).get("stationId")
    )
    return lat, lon, heading, speed, station_id


def on_cpm_out(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    """Parse vanetza/out/cpm — ETSI TR103562 CPM received over GeoNet."""
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        inner = payload.get("fields", {}).get("payload", {})
        sender_id = payload.get("stationID") or payload.get("fields", {}).get("header", {}).get("stationId")

        mgmt = inner.get("managementContainer", {})
        ref = mgmt.get("referencePosition", {})
        sender_lat = float(ref["latitude"])
        sender_lon = float(ref["longitude"])
        sender_x = meters_from_lon_delta(sender_lon - BASE_LON, sender_lat)
        sender_y = meters_from_lat_delta(sender_lat - BASE_LAT)

        now = time.time()
        with state_lock:
            for container in inner.get("cpmContainers", []):
                if container.get("containerId") != 5:
                    continue
                for obj in container.get("containerData", {}).get("perceivedObjects", []):
                    obj_id = obj.get("objectId", 0)
                    pos = obj.get("position", {})
                    dx = float(pos.get("xCoordinate", {}).get("value", 0.0))
                    dy = float(pos.get("yCoordinate", {}).get("value", 0.0))
                    key = f"cpm_{obj_id}"
                    state["objects"][key] = {
                        "x": sender_x + dx,
                        "y": sender_y + dy,
                        "source": "cpm",
                        "detected_by": sender_id,
                        "updated_at": now,
                        "stale": False,
                    }
    except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"cpm out parse error: {exc}")
        return

    emit_state()


def on_cam_out(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    global _cam_counter
    global _cam_window_start

    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        lat, lon, heading, speed, station_id = parse_cam_payload(payload)
    except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"cam parse error: {exc}")
        return

    _cam_counter += 1
    with state_lock:
        if _last_wave_timestamp is not None:
            latency = time.time() - _last_wave_timestamp
            if 0 <= latency <= CAM_TIME_MATCH_WINDOW:
                state["metrics"]["last_cam_latency_sec"] = latency

        x = meters_from_lon_delta(lon - BASE_LON, lat)
        y = meters_from_lat_delta(lat - BASE_LAT)
        obj_key = f"cam_{station_id}" if station_id is not None else "cam_unknown"
        state["objects"][obj_key] = {
            "x": x,
            "y": y,
            "lat": lat,
            "lon": lon,
            "heading": heading,
            "speed": speed,
            "source": "cam",
            "station_id": station_id,
            "updated_at": time.time(),
            "stale": False,
        }

        now = time.time()
        elapsed = max(now - _cam_window_start, 1e-6)
        state["metrics"]["cam_rate_hz"] = _cam_counter / elapsed
        state["metrics"]["last_cam_age_sec"] = 0.0
        state["metrics"]["stale"] = False

        if elapsed >= 10.0:
            _cam_counter = 0
            _cam_window_start = now

    emit_state()


def on_cam_time(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    global _last_wave_timestamp

    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        test = payload.get("test", {})
        wave_timestamp = float(test.get("wave_timestamp"))
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        print(f"cam time parse error: {exc}")
        return

    _last_wave_timestamp = wave_timestamp


def on_cam_default(_client: mqtt.Client, _userdata: Any, _msg: mqtt.MQTTMessage) -> None:
    return


def monitor_stale_loop() -> None:
    while True:
        with state_lock:
            now = time.time()
            cam_ages: list[float] = []
            for obj in state["objects"].values():
                age = max(now - obj["updated_at"], 0.0)
                obj["stale"] = age > STALE_SECONDS
                if obj["source"] == "cam":
                    cam_ages.append(age)

            if cam_ages:
                oldest = max(cam_ages)
                state["metrics"]["last_cam_age_sec"] = oldest
                state["metrics"]["stale"] = oldest > STALE_SECONDS
            else:
                state["metrics"]["last_cam_age_sec"] = None
                state["metrics"]["stale"] = True

        emit_state()
        time.sleep(0.5)


def _connect_with_retry(host: str, port: int, client_id: str) -> mqtt.Client:
    """Connect to MQTT broker with exponential backoff retry."""
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    
    max_retries = 10
    retry_delay = 1
    for attempt in range(max_retries):
        try:
            client.connect(host, port, keepalive=30)
            client.loop_start()
            print(f"[{client_id}] Connected to {host}:{port}")
            return client
        except Exception as e:
            if attempt < max_retries - 1:
                print(f"[{client_id}] Connection attempt {attempt + 1}/{max_retries} failed: {e}. Retrying in {retry_delay}s...")
                time.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, 30)  # Cap at 30s
            else:
                print(f"[{client_id}] Failed to connect after {max_retries} attempts. Giving up.")
                raise


def start_mqtt() -> None:
    world_client = _connect_with_retry(MAIN_BROKER_HOST, MAIN_BROKER_PORT, "vehicle-ego-world")
    world_client.on_message = on_cam_default
    world_client.message_callback_add(TOPIC_WORLD_EGO, on_world_ego)
    world_client.subscribe(TOPIC_WORLD_EGO, qos=1)

    cam_client = _connect_with_retry(EGO_BROKER_HOST, EGO_BROKER_PORT, "vehicle-ego-cam")
    cam_client.on_message = on_cam_default
    cam_client.message_callback_add(TOPIC_CAM_OUT, on_cam_out)
    cam_client.message_callback_add(TOPIC_CPM_OUT, on_cpm_out)
    cam_client.subscribe(TOPIC_CAM_OUT, qos=1)
    cam_client.subscribe(TOPIC_CPM_OUT, qos=1)

    time_client = _connect_with_retry(LEAD_BROKER_HOST, LEAD_BROKER_PORT, "vehicle-ego-time")
    time_client.on_message = on_cam_time
    time_client.subscribe(TOPIC_CAM_TIME, qos=1)


@app.route("/")
def index() -> str:
    return render_template("index.html")


@app.route("/api/status")
def status() -> Any:
    with state_lock:
        return jsonify(state)


if __name__ == "__main__":
    start_mqtt()
    threading.Thread(target=monitor_stale_loop, daemon=True).start()
    socketio.run(app, host="0.0.0.0", port=UI_PORT, allow_unsafe_werkzeug=True)
