import json
import math
import os
import threading
import time
from typing import Any

import paho.mqtt.client as mqtt

from vehicle_lead.control_policy import lead_control_risk, public_object_id
from vehicle_lead.cpm_payload import build_cpm_payload as build_cpm_payload_with_config
from world_generator.scenarios import load_scenario_from_env
from world_generator.risk import first_risk_object, is_in_fov


MAIN_BROKER_HOST = os.getenv("MAIN_BROKER_HOST", "main-broker")
MAIN_BROKER_PORT = int(os.getenv("MAIN_BROKER_PORT", "1883"))
LEAD_BROKER_HOST = os.getenv("LEAD_BROKER_HOST", "lead-broker")
LEAD_BROKER_PORT = int(os.getenv("LEAD_BROKER_PORT", "1883"))
TOPIC_WORLD_LEAD = os.getenv("WORLD_TOPIC_LEAD", "world/pos/lead")
TOPIC_WORLD_EGO = os.getenv("WORLD_TOPIC_EGO", "world/pos/ego")
TOPIC_WORLD_OBSTACLE = os.getenv("WORLD_TOPIC_OBSTACLE", "world/pos/obstacle")
TOPIC_WORLD_CONTROL = os.getenv("WORLD_TOPIC_CONTROL", "world/control")
TOPIC_TX_CAM = os.getenv("WORLD_TOPIC_TX_CAM", "world/tx/cam")
TOPIC_TX_CPM = os.getenv("WORLD_TOPIC_TX_CPM", "world/tx/cpm")
TOPIC_CAM_IN = os.getenv("CAM_IN_TOPIC", "vanetza/in/cam")
TOPIC_CAM_TIME = os.getenv("CAM_TIME_TOPIC", "vanetza/time/cam")
TOPIC_CPM_IN = os.getenv("CPM_IN_TOPIC", "vanetza/in/cpm")
BASE_LAT = float(os.getenv("LEAD_LATITUDE", "40.628300"))
BASE_LON = float(os.getenv("LEAD_LONGITUDE", "-8.654400"))
FOV_RANGE_M = float(os.getenv("LEAD_FOV_RANGE_M", "80.0"))
FOV_HALF_ANGLE_DEG = float(os.getenv("LEAD_FOV_HALF_ANGLE_DEG", "60.0"))
LOOP_SECONDS = float(os.getenv("WORLD_TICK_SECONDS", "1.0"))
SCENARIO = load_scenario_from_env()
OCCLUDERS = [
    occluder for occluder in SCENARIO.get("layout", {}).get("occluders", [])
    if occluder.get("type") == "rect"
]


state_lock = threading.Lock()
lead_state: dict[str, Any] = {"x": 50.0, "y": 0.0, "heading": 0.0, "speed": 0.0}
world_objects: dict[str, dict[str, Any]] = {}
tx_counters = {"cam": 0, "cpm": 0}
lead_hold_risk_id: str | None = None


def meters_to_deg_lon(meters: float, latitude_deg: float) -> float:
    denom = 111320.0 * math.cos(math.radians(latitude_deg))
    if abs(denom) < 1e-9:
        return 0.0
    return meters / denom


def meters_to_deg_lat(meters: float) -> float:
    return meters / 111320.0


def objects_in_fov(vehicle_x: float, vehicle_y: float, heading_deg: float) -> list[dict[str, Any]]:
    perceived = []
    with state_lock:
        objects_snapshot = dict(world_objects)
    for obj_id, obj in objects_snapshot.items():
        dx = obj["x"] - vehicle_x
        dy = obj["y"] - vehicle_y
        distance = math.sqrt(dx * dx + dy * dy)
        if distance < 0.5:
            continue
        if is_in_fov(vehicle_x, vehicle_y, heading_deg, obj["x"], obj["y"], FOV_RANGE_M, FOV_HALF_ANGLE_DEG, OCCLUDERS):
            obj_angle_deg = math.degrees(math.atan2(dy, dx))
            rel_angle = (obj_angle_deg - heading_deg + 360.0) % 360.0
            if rel_angle > 180.0:
                rel_angle -= 360.0
            perceived.append({
                "object_id": public_object_id(obj, obj_id),
                "x": obj["x"],
                "y": obj["y"],
                "heading": obj.get("heading", 0.0),
                "speed": obj.get("speed", 0.0),
                "kind": obj.get("kind", "obstacle"),
                "blocks_vehicle_path": obj.get("blocks_vehicle_path", False),
                "distance_m": round(distance, 2),
                "rel_angle_deg": round(rel_angle, 2),
            })
    return perceived


def build_cam_payload(x_meter: float, y_meter: float = 0.0, heading_deg: float = 0.0, speed_mps: float = 0.0) -> dict[str, Any]:
    lon = BASE_LON + meters_to_deg_lon(x_meter, BASE_LAT)
    lat = BASE_LAT + meters_to_deg_lat(y_meter)
    generation_delta_time = int((time.time() * 1000.0) % 65536)

    return {
        "camParameters": {
            "basicContainer": {
                "stationType": 5,
                "referencePosition": {
                    "latitude": lat,
                    "longitude": lon,
                    "positionConfidenceEllipse": {
                        "semiMajorAxisLength": 4095,
                        "semiMinorAxisLength": 4095,
                        "semiMajorAxisOrientation": 3601,
                    },
                    "altitude": {
                        "altitudeValue": 800001,
                        "altitudeConfidence": 15,
                    },
                },
            },
            "highFrequencyContainer": {
                "basicVehicleContainerHighFrequency": {
                    "heading": {
                        "headingValue": heading_deg,
                        "headingConfidence": 127,
                    },
                    "speed": {
                        "speedValue": speed_mps,
                        "speedConfidence": 127,
                    },
                    "driveDirection": 2,
                    "vehicleLength": {
                        "vehicleLengthValue": 42,
                        "vehicleLengthConfidenceIndication": 4,
                    },
                    "vehicleWidth": 1.8,
                    "longitudinalAcceleration": {
                        "value": 0.0,
                        "confidence": 102,
                    },
                    "curvature": {
                        "curvatureValue": 1023,
                        "curvatureConfidence": 7,
                    },
                    "curvatureCalculationMode": 2,
                    "yawRate": {
                        "yawRateValue": 0.0,
                        "yawRateConfidence": 8,
                    },
                    "accelerationControl": {
                        "brakePedalEngaged": False,
                        "gasPedalEngaged": False,
                        "emergencyBrakeEngaged": False,
                        "collisionWarningEngaged": False,
                        "accEngaged": False,
                        "cruiseControlEngaged": False,
                        "speedLimiterEngaged": False,
                    },
                    "steeringWheelAngle": {
                        "steeringWheelAngleValue": 0.0,
                        "steeringWheelAngleConfidence": 127,
                    },
                }
            },
            "lowFrequencyContainer": {
                "basicVehicleContainerLowFrequency": {
                    "vehicleRole": 0,
                    "exteriorLights": {
                        "lowBeamHeadlightsOn": False,
                        "highBeamHeadlightsOn": False,
                        "leftTurnSignalOn": False,
                        "rightTurnSignalOn": False,
                        "daytimeRunningLightsOn": False,
                        "reverseLightOn": False,
                        "fogLightOn": False,
                        "parkingLightsOn": False,
                    },
                    "pathHistory": [],
                }
            },
        },
        "generationDeltaTime": generation_delta_time,
    }


def build_cpm_payload(lead_x: float, lead_y: float, perceived: list[dict[str, Any]]) -> dict[str, Any]:
    return build_cpm_payload_with_config(
        lead_x,
        lead_y,
        perceived,
        base_lat=BASE_LAT,
        base_lon=BASE_LON,
        fov_range_m=FOV_RANGE_M,
        fov_half_angle_deg=FOV_HALF_ANGLE_DEG,
    )


def on_world_lead(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        with state_lock:
            lead_state["x"] = float(payload["x"])
            lead_state["y"] = float(payload.get("y", 0.0))
            lead_state["heading"] = float(payload.get("heading", 0.0))
            lead_state["speed"] = float(payload.get("speed", 0.0))
    except (ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"world lead parse error: {exc}")


def on_world_ego(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    """Ego vehicle also counts as an obstacle for detection."""
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        with state_lock:
            world_objects["ego"] = {
                "id": "ego",
                "x": float(payload["x"]),
                "y": float(payload.get("y", 0.0)),
                "heading": float(payload.get("heading", 0.0)),
                "speed": float(payload.get("speed", 0.0)),
                "status": payload.get("status", "moving"),
                "kind": payload.get("kind", "vehicle"),
                "blocks_vehicle_path": False,
            }
    except (ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"world ego parse error: {exc}")


def on_world_obstacle(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        topic = msg.topic  # e.g., "world/pos/obstacle/1"
        # Extract obstacle ID from topic (e.g., "1" from "world/pos/obstacle/1")
        obs_id = topic.split("/")[-1] if "/" in topic else "obstacle"
        with state_lock:
            world_objects[f"obstacle_{obs_id}"] = {
                "id": str(payload.get("id") or obs_id),
                "x": float(payload["x"]),
                "y": float(payload.get("y", 0.0)),
                "heading": float(payload.get("heading", 0.0)),
                "speed": float(payload.get("speed", 0.0)),
                "status": payload.get("status", "moving"),
                "kind": payload.get("kind", "obstacle"),
                "blocks_vehicle_path": bool(payload.get("blocks_vehicle_path", True)),
            }
    except (ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"world obstacle parse error: {exc}")


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


def start_world_subscriber() -> mqtt.Client:
    client = _connect_with_retry(MAIN_BROKER_HOST, MAIN_BROKER_PORT, "vehicle-lead-world")
    client.message_callback_add(TOPIC_WORLD_LEAD, on_world_lead)
    client.message_callback_add(TOPIC_WORLD_EGO, on_world_ego)
    # Subscribe to all obstacles: world/pos/obstacle/+ (wildcard matches any obstacle ID)
    client.message_callback_add("world/pos/obstacle/+", on_world_obstacle)
    client.on_message = lambda c, u, m: None
    client.subscribe([(TOPIC_WORLD_LEAD, 1), (TOPIC_WORLD_EGO, 1), ("world/pos/obstacle/+", 1)])
    return client


def start_cam_publisher() -> mqtt.Client:
    client = _connect_with_retry(LEAD_BROKER_HOST, LEAD_BROKER_PORT, "vehicle-lead-cam")
    return client


def publish_control(client: mqtt.Client, vehicle_name: str, risk: dict[str, Any] | None) -> None:
    if risk:
        payload = {
            "action": "stop",
            "reason": "path-risk",
            "risk_object_id": risk.get("object_id"),
            "ttl_seconds": max(LOOP_SECONDS * 3.0, 0.5),
            "timestamp": time.time(),
        }
    else:
        payload = {
            "action": "resume",
            "reason": "",
            "risk_object_id": None,
            "ttl_seconds": max(LOOP_SECONDS * 3.0, 0.5),
            "timestamp": time.time(),
        }
    client.publish(f"{TOPIC_WORLD_CONTROL}/{vehicle_name}", json.dumps(payload), qos=1)


def publish_tx(client: mqtt.Client, topic: str, message_type: str, generated_at: float, object_count: int = 0) -> None:
    tx_counters[message_type] += 1
    payload = {
        "message_type": message_type,
        "sequence": tx_counters[message_type],
        "station": "lead",
        "generated_at": generated_at,
        "sent_at": time.time(),
        "object_count": object_count,
    }
    client.publish(topic, json.dumps(payload), qos=1)


if __name__ == "__main__":
    world_client = start_world_subscriber()
    cam_client = start_cam_publisher()

    while True:
        with state_lock:
            x_snapshot = lead_state["x"]
            y_snapshot = lead_state["y"]
            heading_snapshot = lead_state["heading"]
            speed_snapshot = lead_state["speed"]

        perceived = objects_in_fov(x_snapshot, y_snapshot, heading_snapshot)
        risk = first_risk_object(
            {"x": x_snapshot, "y": y_snapshot, "heading": heading_snapshot},
            perceived,
            lookahead_m=70.0,
            half_width_m=7.0,
        )
        with state_lock:
            objects_snapshot = {key: dict(obj) for key, obj in world_objects.items()}
        control_risk, lead_hold_risk_id = lead_control_risk(risk, objects_snapshot, lead_hold_risk_id)
        publish_control(world_client, "lead", control_risk)

        cpm_objects = perceived
        if cpm_objects:
            print(f"FoV detected {len(perceived)} object(s), CPM objects={len(cpm_objects)}: {perceived}")
            cpm = build_cpm_payload(x_snapshot, y_snapshot, cpm_objects)
            cpm_ts = time.time()
            publish_tx(world_client, TOPIC_TX_CPM, "cpm", cpm_ts, object_count=len(cpm_objects))
            cam_client.publish(TOPIC_CPM_IN, json.dumps(cpm), qos=1)
            print(f"published CPM -> {TOPIC_CPM_IN} with {len(cpm_objects)} object(s)")

        wave_ts = time.time()
        cam_client.publish(TOPIC_CAM_TIME, json.dumps({"test": {"wave_timestamp": wave_ts}}), qos=1)

        cam = build_cam_payload(x_snapshot, y_snapshot, heading_snapshot, speed_snapshot)
        publish_tx(world_client, TOPIC_TX_CAM, "cam", wave_ts)
        cam_client.publish(TOPIC_CAM_IN, json.dumps(cam), qos=1)

        risk_id = risk.get("object_id") if risk else "none"
        print(f"published CAM x={x_snapshot:.2f} perceived={len(perceived)} risk={risk_id}")
        time.sleep(max(LOOP_SECONDS, 0.01))
