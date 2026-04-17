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
TOPIC_WORLD_EGO = os.getenv("WORLD_TOPIC_EGO", "world/pos/ego")
TOPIC_CAM_OUT = os.getenv("CAM_OUT_TOPIC", "vanetza/out/cam")
UI_PORT = int(os.getenv("UI_PORT", "8080"))
STALE_SECONDS = 3.0


app = Flask(__name__, template_folder="templates", static_folder="static")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

state_lock = threading.Lock()
state: dict[str, Any] = {
    "ego": {"x": 20.0, "updated_at": 0.0},
    "lead": {"x": 50.0, "lat": None, "lon": None, "updated_at": 0.0, "station_id": None},
    "metrics": {"cam_rate_hz": 0.0, "last_cam_age_sec": None, "stale": True},
}
_cam_counter = 0
_cam_window_start = time.time()


def meters_from_lon_delta(delta_lon: float, latitude: float) -> float:
    import math

    return delta_lon * 111320.0 * math.cos(math.radians(latitude))


def emit_state() -> None:
    with state_lock:
        payload = json.loads(json.dumps(state))
    socketio.emit("state_update", payload)


def on_world_ego(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        x_value = float(payload["x"])
    except (ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"world ego parse error: {exc}")
        return

    with state_lock:
        state["ego"]["x"] = x_value
        state["ego"]["updated_at"] = time.time()

    emit_state()


def on_cam(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    global _cam_counter
    global _cam_window_start

    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        # Support both raw CAM JSON and wrapper formats like {"fields": {"header": ..., "cam": ...}}
        cam_root = payload.get("fields", {}).get("cam", payload)
        cam_params = cam_root.get("camParameters", {})
        basic = cam_params.get("basicContainer", {})
        ref_pos = basic.get("referencePosition", {})

        lat = float(ref_pos["latitude"])
        lon = float(ref_pos["longitude"])
        station_id = (
            payload.get("fields", {}).get("header", {}).get("stationId")
            or payload.get("itsPduHeader", {}).get("stationId")
            or payload.get("header", {}).get("stationId")
        )
    except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"cam parse error: {exc}")
        return

    _cam_counter += 1
    with state_lock:
        base_lon = -8.6544
        lead_x = meters_from_lon_delta(lon - base_lon, lat)

        state["lead"]["x"] = lead_x
        state["lead"]["lat"] = lat
        state["lead"]["lon"] = lon
        state["lead"]["updated_at"] = time.time()
        state["lead"]["station_id"] = station_id

        now = time.time()
        elapsed = max(now - _cam_window_start, 1e-6)
        state["metrics"]["cam_rate_hz"] = _cam_counter / elapsed
        state["metrics"]["last_cam_age_sec"] = 0.0
        state["metrics"]["stale"] = False

        if elapsed >= 10.0:
            _cam_counter = 0
            _cam_window_start = now

    emit_state()


def monitor_stale_loop() -> None:
    while True:
        with state_lock:
            lead_updated = state["lead"]["updated_at"]
            if lead_updated <= 0:
                age = None
                stale = True
            else:
                age = max(time.time() - lead_updated, 0.0)
                stale = age > STALE_SECONDS

            state["metrics"]["last_cam_age_sec"] = age
            state["metrics"]["stale"] = stale

        emit_state()
        time.sleep(0.5)


def start_mqtt() -> None:
    world_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="vehicle-ego-world")
    world_client.on_message = on_world_ego
    world_client.connect(MAIN_BROKER_HOST, MAIN_BROKER_PORT, keepalive=30)
    world_client.subscribe(TOPIC_WORLD_EGO, qos=1)
    world_client.loop_start()

    cam_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="vehicle-ego-cam")
    cam_client.on_message = on_cam
    cam_client.connect(EGO_BROKER_HOST, EGO_BROKER_PORT, keepalive=30)
    cam_client.subscribe(TOPIC_CAM_OUT, qos=1)
    cam_client.loop_start()


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
