import json
import math
import os
import threading
import time
from typing import Any

import paho.mqtt.client as mqtt


MAIN_BROKER_HOST = os.getenv("MAIN_BROKER_HOST", "main-broker")
MAIN_BROKER_PORT = int(os.getenv("MAIN_BROKER_PORT", "1883"))
LEAD_BROKER_HOST = os.getenv("LEAD_BROKER_HOST", "lead-broker")
LEAD_BROKER_PORT = int(os.getenv("LEAD_BROKER_PORT", "1883"))
TOPIC_WORLD_LEAD = os.getenv("WORLD_TOPIC_LEAD", "world/pos/lead")
TOPIC_WORLD_EGO = os.getenv("WORLD_TOPIC_EGO", "world/pos/ego")
TOPIC_WORLD_OBSTACLE = os.getenv("WORLD_TOPIC_OBSTACLE", "world/pos/obstacle")
TOPIC_CAM_IN = os.getenv("CAM_IN_TOPIC", "vanetza/in/cam")
TOPIC_CAM_TIME = os.getenv("CAM_TIME_TOPIC", "vanetza/time/cam")
TOPIC_CPM_IN = os.getenv("CPM_IN_TOPIC", "vanetza/in/cpm")
BASE_LAT = float(os.getenv("LEAD_LATITUDE", "40.628300"))
BASE_LON = float(os.getenv("LEAD_LONGITUDE", "-8.654400"))
FOV_RANGE_M = float(os.getenv("LEAD_FOV_RANGE_M", "80.0"))
FOV_HALF_ANGLE_DEG = float(os.getenv("LEAD_FOV_HALF_ANGLE_DEG", "60.0"))


state_lock = threading.Lock()
lead_state: dict[str, Any] = {"x": 50.0, "y": 0.0, "heading": 0.0, "speed": 0.0}
world_objects: dict[str, dict[str, Any]] = {}


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
        if distance < 0.5 or distance > FOV_RANGE_M:
            continue
        obj_angle_deg = math.degrees(math.atan2(dy, dx))
        rel_angle = (obj_angle_deg - heading_deg + 360.0) % 360.0
        if rel_angle > 180.0:
            rel_angle -= 360.0
        if abs(rel_angle) <= FOV_HALF_ANGLE_DEG:
            perceived.append({
                "object_id": obj_id,
                "x": obj["x"],
                "y": obj["y"],
                "distance_m": round(distance, 2),
                "rel_angle_deg": round(rel_angle, 2),
            })
    return perceived


def build_cam_payload(x_meter: float, y_meter: float = 0.0) -> dict[str, Any]:
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
                        "headingValue": 90.0,
                        "headingConfidence": 127,
                    },
                    "speed": {
                        "speedValue": 8.0,
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
    lat = BASE_LAT + meters_to_deg_lat(lead_y)
    lon = BASE_LON + meters_to_deg_lon(lead_x, BASE_LAT)
    objects = []
    for idx, obj in enumerate(perceived):
        # xCoordinate = longitudinal (forward, +X in our world)
        # yCoordinate = lateral (sideways, +Y in our world)
        dx = round(obj["x"] - lead_x, 2)
        dy = round(obj["y"] - lead_y, 2)
        objects.append({
            "objectId": idx + 1,
            "sensorIdList": [1],
            "measurementDeltaTime": 0,
            "position": {
                "xCoordinate": {"value": dx, "confidence": 1},
                "yCoordinate": {"value": dy, "confidence": 1},
            },
            "velocity": {
                "cartesianVelocity": {
                    "xVelocity": {"value": 0.0, "confidence": 1},
                    "yVelocity": {"value": 0.0, "confidence": 1},
                }
            },
            "objectDimensionX": {"value": 2.0, "confidence": 1},
            "objectDimensionY": {"value": 2.0, "confidence": 1},
        })
    return {
        "managementContainer": {
            "referenceTime": int((time.time() * 1000.0) % 65536),
            "referencePosition": {
                "latitude": lat,
                "longitude": lon,
                "positionConfidenceEllipse": {
                    "semiMajorConfidence": 4095,
                    "semiMajorOrientation": 0,
                    "semiMinorConfidence": 4095,
                },
                "altitude": {"altitudeValue": 800001, "altitudeConfidence": 15},
            },
        },
        "cpmContainers": [
            {
                "containerId": 3,
                "containerData": [{
                    "sensorId": 1,
                    "sensorType": 1,
                    "perceptionRegionShape": {
                        "radial": {
                            "range": int(FOV_RANGE_M),
                            "horizontalOpeningAngleStart": int(90 - FOV_HALF_ANGLE_DEG),
                            "horizontalOpeningAngleEnd": int(90 + FOV_HALF_ANGLE_DEG),
                        }
                    },
                    "shadowingApplies": False,
                }],
            },
            {
                "containerId": 5,
                "containerData": {
                    "numberOfPerceivedObjects": len(objects),
                    "perceivedObjects": objects,
                },
            },
        ],
    }


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
                "x": float(payload["x"]),
                "y": float(payload.get("y", 0.0)),
                "heading": float(payload.get("heading", 0.0)),
                "speed": float(payload.get("speed", 0.0)),
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
                "x": float(payload["x"]),
                "y": float(payload.get("y", 0.0)),
                "heading": float(payload.get("heading", 0.0)),
                "speed": float(payload.get("speed", 0.0)),
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


if __name__ == "__main__":
    world_client = start_world_subscriber()
    cam_client = start_cam_publisher()

    while True:
        with state_lock:
            x_snapshot = lead_state["x"]
            y_snapshot = lead_state["y"]
            heading_snapshot = lead_state["heading"]

        perceived = objects_in_fov(x_snapshot, y_snapshot, heading_snapshot)
        if perceived:
            print(f"FoV detected {len(perceived)} object(s): {perceived}")
            cpm = build_cpm_payload(x_snapshot, y_snapshot, perceived)
            cam_client.publish(TOPIC_CPM_IN, json.dumps(cpm), qos=1)
            print(f"published CPM -> {TOPIC_CPM_IN} with {len(perceived)} object(s)")

        wave_ts = time.time()
        cam_client.publish(TOPIC_CAM_TIME, json.dumps({"test": {"wave_timestamp": wave_ts}}), qos=1)

        cam = build_cam_payload(x_snapshot, y_snapshot)
        cam_client.publish(TOPIC_CAM_IN, json.dumps(cam), qos=1)

        print(f"published CAM x={x_snapshot:.2f} perceived={len(perceived)}")
        time.sleep(1.0)
