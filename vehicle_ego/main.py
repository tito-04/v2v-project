import json
import os
import threading
import time
from typing import Any

from flask import Flask, jsonify, render_template
from flask_socketio import SocketIO
import paho.mqtt.client as mqtt

from vehicle_ego.model_state import cam_model_key, match_world_object_by_position, model_key_for_world_candidate, upsert_model_object
from world_generator.network_metrics import NetworkMetrics
from world_generator.risk import first_risk_object, is_in_fov
from world_generator.scenarios import load_scenario_from_env, public_metadata


MAIN_BROKER_HOST = os.getenv("MAIN_BROKER_HOST", "main-broker")
MAIN_BROKER_PORT = int(os.getenv("MAIN_BROKER_PORT", "1883"))
EGO_BROKER_HOST = os.getenv("EGO_BROKER_HOST", "ego-broker")
EGO_BROKER_PORT = int(os.getenv("EGO_BROKER_PORT", "1883"))
LEAD_BROKER_HOST = os.getenv("LEAD_BROKER_HOST", "lead-broker")
LEAD_BROKER_PORT = int(os.getenv("LEAD_BROKER_PORT", "1883"))
TOPIC_WORLD_EGO = os.getenv("WORLD_TOPIC_EGO", "world/pos/ego")
TOPIC_WORLD_LEAD = os.getenv("WORLD_TOPIC_LEAD", "world/pos/lead")
TOPIC_WORLD_OBSTACLE = os.getenv("WORLD_TOPIC_OBSTACLE", "world/pos/obstacle")
TOPIC_WORLD_SCENARIO = os.getenv("WORLD_TOPIC_SCENARIO", "world/scenario")
TOPIC_WORLD_CONTROL = os.getenv("WORLD_TOPIC_CONTROL", "world/control")
TOPIC_TX_CAM = os.getenv("WORLD_TOPIC_TX_CAM", "world/tx/cam")
TOPIC_TX_CPM = os.getenv("WORLD_TOPIC_TX_CPM", "world/tx/cpm")
TOPIC_CPM_OUT = os.getenv("CPM_OUT_TOPIC", "vanetza/out/cpm")
TOPIC_CAM_OUT = os.getenv("CAM_OUT_TOPIC", "vanetza/out/cam")
TOPIC_CAM_TIME = os.getenv("CAM_TIME_TOPIC", "vanetza/time/cam")
UI_PORT = int(os.getenv("UI_PORT", "8080"))
STALE_SECONDS = float(os.getenv("STALE_SECONDS", "3.0"))
CAM_TIME_MATCH_WINDOW = 3.0
BASE_LAT = float(os.getenv("WORLD_BASE_LAT", "40.628300"))
BASE_LON = float(os.getenv("WORLD_BASE_LON", "-8.654400"))
FOV_RANGE_M = 80.0
FOV_HALF_ANGLE_DEG = 60.0
SCENARIO = load_scenario_from_env()
OCCLUDERS = [
    occluder for occluder in SCENARIO.get("layout", {}).get("occluders", [])
    if occluder.get("type") == "rect"
]


app = Flask(__name__, template_folder="templates", static_folder="static")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

state_lock = threading.Lock()
network_metrics = NetworkMetrics(loss_timeout_seconds=max(STALE_SECONDS, 3.0))
control_client: mqtt.Client | None = None
_cam_counter = 0
_cam_window_start = time.time()
_last_wave_timestamp: float | None = None

state: dict[str, Any] = {
    "self": {
        "id": "ego",
        "kind": "vehicle",
        "x": 10.0,
        "y": 0.0,
        "heading": 0.0,
        "speed": 0.0,
        "status": "loading",
        "updated_at": 0.0,
        "fov_range": FOV_RANGE_M,
        "fov_half_angle": FOV_HALF_ANGLE_DEG,
    },
    "objects": {},
    "metrics": {
        "cam_rate_hz": 0.0,
        "last_cam_age_sec": None,
        "last_cam_latency_sec": None,
        "stale": True,
    },
    "scenario": public_metadata(SCENARIO),
    "world": {
        "vehicles": {},
        "objects": {},
    },
    "ego_model": {
        "self": {},
        "objects": {},
        "last_action": {"action": "resume", "risk_object_id": None, "timestamp": 0.0},
        "fov_range": FOV_RANGE_M,
        "fov_half_angle": FOV_HALF_ANGLE_DEG,
    },
    "network": network_metrics.snapshot(),
}
cooperative_hold_risk_ids: set[str] = set()


def meters_from_lon_delta(delta_lon: float, latitude: float) -> float:
    import math

    return delta_lon * 111320.0 * math.cos(math.radians(latitude))


def meters_from_lat_delta(delta_lat: float) -> float:
    return delta_lat * 111320.0


def emit_state() -> None:
    with state_lock:
        payload = json.loads(json.dumps(state))
    socketio.emit("state_update", payload)


def parse_actor_payload(payload: dict[str, Any], actor_id: str, kind: str) -> dict[str, Any]:
    now = time.time()
    return {
        "id": str(payload.get("id") or actor_id),
        "kind": str(payload.get("kind") or kind),
        "x": float(payload["x"]),
        "y": float(payload.get("y", 0.0)),
        "heading": float(payload.get("heading", 0.0)),
        "speed": float(payload.get("speed", 0.0)),
        "target_speed": float(payload.get("target_speed", payload.get("speed", 0.0))),
        "status": str(payload.get("status", "moving")),
        "reason": str(payload.get("reason", "")),
        "risk_object_id": payload.get("risk_object_id"),
        "blocks_vehicle_path": bool(payload.get("blocks_vehicle_path", False)),
        "width": float(payload.get("width") or 2.0),
        "length": float(payload.get("length") or 2.0),
        "updated_at": now,
        "stale": False,
    }


def sync_legacy_locked() -> None:
    state["objects"] = json.loads(json.dumps(state["ego_model"]["objects"]))
    state["network"] = network_metrics.snapshot()


def should_hold_for_crossing_pedestrian_locked() -> dict[str, Any] | None:
    for obj in active_model_objects_locked():
        if obj.get("kind") != "pedestrian":
            continue
        world_obj = world_object_locked(str(obj.get("id", "")))
        if world_obj and world_obj.get("status") == "done":
            continue
        if obj.get("status") == "crossing" and obj.get("blocks_vehicle_path", False):
            return obj
    return None


def world_object_locked(object_id: str) -> dict[str, Any] | None:
    obj = state["world"]["objects"].get(object_id)
    return obj if isinstance(obj, dict) else None


def cooperative_intersection_risk_locked() -> dict[str, Any] | None:
    if state.get("scenario", {}).get("name") != "intersection-occlusion":
        return None
    world_pedestrian = world_object_locked("pedestrian-1")
    if world_pedestrian and world_pedestrian.get("status") == "done":
        return None
    for obj in active_model_objects_locked():
        if obj.get("id") != "pedestrian-1":
            continue
        if obj.get("status") == "done":
            continue
        if obj.get("observed_via") == "v2v_cpm" and obj.get("blocks_vehicle_path", False):
            return obj
    return None


def cooperative_hold_risk_locked() -> dict[str, Any] | None:
    for risk_id in tuple(cooperative_hold_risk_ids):
        world_obj = world_object_locked(risk_id)
        if world_obj and world_obj.get("status") == "done":
            cooperative_hold_risk_ids.discard(risk_id)
            continue

        model_obj = state["ego_model"]["objects"].get(risk_id)
        item = dict(world_obj or model_obj or {})
        item.update({
            "id": risk_id,
            "kind": item.get("kind", "pedestrian"),
            "blocks_vehicle_path": True,
        })
        return item
    return None


def cooperative_stop_line_payload(risk: dict[str, Any]) -> dict[str, Any]:
    stop_config = SCENARIO.get("vehicles", {}).get("ego", {}).get("cooperative_stop", {})
    if not isinstance(stop_config, dict):
        return {}
    expected_risk = stop_config.get("risk_object_id")
    if expected_risk and risk.get("id") != expected_risk:
        return {}
    axis = stop_config.get("stop_axis")
    value = stop_config.get("stop_value")
    if axis not in {"x", "y"} or value is None:
        return {}
    payload = {
        "stop_axis": axis,
        "stop_value": float(value),
        "stop_direction": float(stop_config.get("stop_direction", 1)),
    }
    stop_pose = stop_config.get("stop_pose", {})
    if isinstance(stop_pose, dict):
        if "x" in stop_pose:
            payload["stop_x"] = float(stop_pose["x"])
        if "y" in stop_pose:
            payload["stop_y"] = float(stop_pose["y"])
        if "heading" in stop_pose:
            payload["stop_heading"] = float(stop_pose["heading"])
        if "route_index" in stop_pose:
            payload["stop_route_index"] = int(stop_pose["route_index"])
    return payload


def update_direct_perception_locked(now: float) -> None:
    ego = state["self"]
    state["ego_model"]["self"] = dict(ego)

    candidates: dict[str, dict[str, Any]] = {}
    for vehicle_name, vehicle in state["world"]["vehicles"].items():
        if vehicle_name != "ego":
            candidates[f"vehicle_{vehicle_name}"] = vehicle
    for object_id, obj in state["world"]["objects"].items():
        candidates[f"object_{object_id}"] = obj

    for key, obj in candidates.items():
        visible = is_in_fov(
            float(ego["x"]),
            float(ego.get("y", 0.0)),
            float(ego.get("heading", 0.0)),
            float(obj["x"]),
            float(obj.get("y", 0.0)),
            FOV_RANGE_M,
            FOV_HALF_ANGLE_DEG,
            OCCLUDERS,
        )
        if not visible:
            continue

        model_key = model_key_for_world_candidate(key, obj)
        item = dict(obj)
        item.update({
            "source": "direct",
            "observed_via": "ego_sensor",
            "updated_at": now,
            "stale": False,
            "in_ego_fov": True,
        })
        upsert_model_object(state["ego_model"]["objects"], model_key, item, source_priority=30)

    sync_legacy_locked()


def active_model_objects_locked() -> list[dict[str, Any]]:
    return [
        obj for obj in state["ego_model"]["objects"].values()
        if not obj.get("stale", False)
    ]


def publish_ego_control_locked() -> None:
    if control_client is None:
        return

    hold_risk = should_hold_for_crossing_pedestrian_locked()
    cooperative_risk = cooperative_intersection_risk_locked()
    if cooperative_risk:
        cooperative_hold_risk_ids.add(str(cooperative_risk.get("id")))
    cooperative_hold_risk = cooperative_hold_risk_locked()
    direct_risk = first_risk_object(
        state["ego_model"].get("self") or state["self"],
        active_model_objects_locked(),
        lookahead_m=70.0,
        half_width_m=7.0,
    )
    risk = cooperative_hold_risk or hold_risk or cooperative_risk or direct_risk
    stop_line = cooperative_stop_line_payload(risk) if risk else {}
    action = "stop_at" if risk and stop_line else ("stop" if risk else "resume")
    payload = {
        "action": action,
        "reason": "ego-model-risk" if risk else "",
        "risk_object_id": risk.get("id") if risk else None,
        "ttl_seconds": 0.8,
        "timestamp": time.time(),
    }
    payload.update(stop_line)
    state["ego_model"]["last_action"] = payload
    control_client.publish(f"{TOPIC_WORLD_CONTROL}/ego", json.dumps(payload), qos=1)


def on_world_scenario(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
    except json.JSONDecodeError as exc:
        print(f"world scenario parse error: {exc}")
        return

    with state_lock:
        state["scenario"] = payload
    emit_state()


def on_world_ego(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        actor = parse_actor_payload(payload, "ego", "vehicle")
    except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"world ego parse error: {exc}")
        return

    with state_lock:
        actor["fov_range"] = FOV_RANGE_M
        actor["fov_half_angle"] = FOV_HALF_ANGLE_DEG
        state["self"] = actor
        state["world"]["vehicles"]["ego"] = actor
        update_direct_perception_locked(time.time())
        publish_ego_control_locked()
    emit_state()


def on_world_lead(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        actor = parse_actor_payload(payload, "lead", "vehicle")
    except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"world lead parse error: {exc}")
        return

    with state_lock:
        state["world"]["vehicles"]["lead"] = actor
        update_direct_perception_locked(time.time())
        publish_ego_control_locked()
    emit_state()


def on_world_obstacle(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        obs_id = str(payload.get("id") or msg.topic.split("/")[-1])
        actor = parse_actor_payload(payload, obs_id, "obstacle")
    except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"world obstacle parse error: {exc}")
        return

    with state_lock:
        state["world"]["objects"][obs_id] = actor
        update_direct_perception_locked(time.time())
        publish_ego_control_locked()
    emit_state()


def parse_cam_payload(payload: dict[str, Any]) -> tuple[float, float, float, float, Any]:
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
    except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"cpm out parse error: {exc}")
        return

    now = time.time()
    with state_lock:
        network_metrics.record_rx("cpm", now)
        for container in inner.get("cpmContainers", []):
            if container.get("containerId") != 5:
                continue
            for obj in container.get("containerData", {}).get("perceivedObjects", []):
                obj_id = obj.get("objectId", 0)
                pos = obj.get("position", {})
                dx = float(pos.get("xCoordinate", {}).get("value", 0.0))
                dy = float(pos.get("yCoordinate", {}).get("value", 0.0))
                obj_x = sender_x + dx
                obj_y = sender_y + dy
                matched = match_world_object_by_position(state["world"]["objects"], obj_x, obj_y)
                if matched:
                    key, world_obj = matched
                    item = dict(world_obj)
                    item.update({
                        "id": world_obj.get("id", key),
                        "source": "cpm",
                        "observed_via": "v2v_cpm",
                        "detected_by": sender_id,
                        "updated_at": now,
                        "stale": False,
                        "blocks_vehicle_path": bool(world_obj.get("blocks_vehicle_path", True)),
                    })
                    upsert_model_object(state["ego_model"]["objects"], key, item, source_priority=20)
                    continue

                key = f"cpm_{sender_id if sender_id is not None else 'unknown'}_{obj_id}"
                item = {
                    "id": key,
                    "kind": "remote-object",
                    "x": obj_x,
                    "y": obj_y,
                    "heading": 0.0,
                    "speed": 0.0,
                    "source": "cpm",
                    "observed_via": "v2v_cpm",
                    "detected_by": sender_id,
                    "updated_at": now,
                    "stale": False,
                    "blocks_vehicle_path": True,
                }
                upsert_model_object(state["ego_model"]["objects"], key, item, source_priority=20)
        sync_legacy_locked()
        publish_ego_control_locked()
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

    now = time.time()
    _cam_counter += 1
    with state_lock:
        latency = network_metrics.record_rx("cam", now)
        if latency is None and _last_wave_timestamp is not None:
            fallback_latency = now - _last_wave_timestamp
            if 0 <= fallback_latency <= CAM_TIME_MATCH_WINDOW:
                latency = fallback_latency

        x = meters_from_lon_delta(lon - BASE_LON, lat)
        y = meters_from_lat_delta(lat - BASE_LAT)
        obj_key = cam_model_key(state["ego_model"]["objects"], x, y, station_id)
        state["ego_model"]["objects"].pop(f"cam_{station_id}", None)
        state["ego_model"]["objects"].pop("cam_unknown", None)
        item = {
            "id": obj_key,
            "kind": "vehicle",
            "x": x,
            "y": y,
            "lat": lat,
            "lon": lon,
            "heading": heading,
            "speed": speed,
            "source": "cam",
            "observed_via": "v2v_cam",
            "station_id": station_id,
            "updated_at": now,
            "stale": False,
            "blocks_vehicle_path": False,
        }
        upsert_model_object(state["ego_model"]["objects"], obj_key, item, source_priority=10)

        elapsed = max(now - _cam_window_start, 1e-6)
        state["metrics"]["cam_rate_hz"] = _cam_counter / elapsed
        state["metrics"]["last_cam_age_sec"] = 0.0
        state["metrics"]["last_cam_latency_sec"] = latency
        state["metrics"]["stale"] = False
        if elapsed >= 10.0:
            _cam_counter = 0
            _cam_window_start = now

        sync_legacy_locked()
        publish_ego_control_locked()
    emit_state()


def on_tx(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        message_type = str(payload.get("message_type") or msg.topic.split("/")[-1])
        sequence = int(payload["sequence"])
        generated_at = float(payload.get("generated_at", payload.get("sent_at", time.time())))
    except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"tx telemetry parse error: {exc}")
        return

    with state_lock:
        network_metrics.record_tx(message_type, sequence, generated_at, now=time.time())
        sync_legacy_locked()
    emit_state()


def on_cam_time(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    global _last_wave_timestamp

    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        test = payload.get("test", {})
        _last_wave_timestamp = float(test.get("wave_timestamp"))
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        print(f"cam time parse error: {exc}")


def on_cam_default(_client: mqtt.Client, _userdata: Any, _msg: mqtt.MQTTMessage) -> None:
    return


def monitor_stale_loop() -> None:
    while True:
        with state_lock:
            now = time.time()
            network_metrics.sweep(now)
            cam_ages: list[float] = []
            for obj in state["ego_model"]["objects"].values():
                updated_at = obj.get("updated_at", now)
                age = max(now - updated_at, 0.0)
                obj["stale"] = age > STALE_SECONDS
                if obj.get("source") == "cam":
                    cam_ages.append(age)

            for collection in (state["world"]["vehicles"], state["world"]["objects"]):
                for obj in collection.values():
                    updated_at = obj.get("updated_at", now)
                    obj["stale"] = max(now - updated_at, 0.0) > STALE_SECONDS

            if cam_ages:
                oldest = max(cam_ages)
                state["metrics"]["last_cam_age_sec"] = oldest
                state["metrics"]["stale"] = oldest > STALE_SECONDS
            else:
                state["metrics"]["last_cam_age_sec"] = None
                state["metrics"]["stale"] = True

            network = network_metrics.snapshot()
            state["network"] = network
            if network["cam"]["last_delay_sec"] is not None:
                state["metrics"]["last_cam_latency_sec"] = network["cam"]["last_delay_sec"]

            sync_legacy_locked()
            publish_ego_control_locked()

        emit_state()
        time.sleep(0.5)


def _connect_with_retry(host: str, port: int, client_id: str) -> mqtt.Client:
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
                retry_delay = min(retry_delay * 2, 30)
            else:
                print(f"[{client_id}] Failed to connect after {max_retries} attempts. Giving up.")
                raise


def start_mqtt() -> None:
    global control_client

    world_client = _connect_with_retry(MAIN_BROKER_HOST, MAIN_BROKER_PORT, "vehicle-ego-world")
    control_client = world_client
    world_client.on_message = on_cam_default
    world_client.message_callback_add(TOPIC_WORLD_EGO, on_world_ego)
    world_client.message_callback_add(TOPIC_WORLD_LEAD, on_world_lead)
    world_client.message_callback_add(TOPIC_WORLD_SCENARIO, on_world_scenario)
    world_client.message_callback_add(f"{TOPIC_WORLD_OBSTACLE}/+", on_world_obstacle)
    world_client.message_callback_add(TOPIC_TX_CAM, on_tx)
    world_client.message_callback_add(TOPIC_TX_CPM, on_tx)
    world_client.subscribe(TOPIC_WORLD_EGO, qos=1)
    world_client.subscribe(TOPIC_WORLD_LEAD, qos=1)
    world_client.subscribe(TOPIC_WORLD_SCENARIO, qos=1)
    world_client.subscribe(f"{TOPIC_WORLD_OBSTACLE}/+", qos=1)
    world_client.subscribe(TOPIC_TX_CAM, qos=1)
    world_client.subscribe(TOPIC_TX_CPM, qos=1)

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
