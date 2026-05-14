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
TOPIC_CAM_IN = os.getenv("CAM_IN_TOPIC", "vanetza/in/cam")
TOPIC_CAM_TIME = os.getenv("CAM_TIME_TOPIC", "vanetza/time/cam")
BASE_LAT = float(os.getenv("LEAD_LATITUDE", "40.628300"))
BASE_LON = float(os.getenv("LEAD_LONGITUDE", "-8.654400"))


state_lock = threading.Lock()
lead_x = 50.0


def meters_to_deg_lon(meters: float, latitude_deg: float) -> float:
    denom = 111320.0 * math.cos(math.radians(latitude_deg))
    if abs(denom) < 1e-9:
        return 0.0
    return meters / denom


def build_cam_payload(x_meter: float) -> dict[str, Any]:
    lon = BASE_LON + meters_to_deg_lon(x_meter, BASE_LAT)
    generation_delta_time = int((time.time() * 1000.0) % 65536)

    return {
        "camParameters": {
            "basicContainer": {
                "stationType": 5,
                "referencePosition": {
                    "latitude": BASE_LAT,
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


def on_world_message(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
    global lead_x

    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        new_x = float(payload["x"])
    except (ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"world parse error: {exc}")
        return

    with state_lock:
        lead_x = new_x


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
    client.on_message = on_world_message
    client.subscribe(TOPIC_WORLD_LEAD, qos=1)
    return client


def start_cam_publisher() -> mqtt.Client:
    client = _connect_with_retry(LEAD_BROKER_HOST, LEAD_BROKER_PORT, "vehicle-lead-cam")
    return client


if __name__ == "__main__":
    start_world_subscriber()
    cam_client = start_cam_publisher()

    while True:
        with state_lock:
            x_snapshot = lead_x

        wave_ts = time.time()
        cam_client.publish(TOPIC_CAM_TIME, json.dumps({"test": {"wave_timestamp": wave_ts}}), qos=1)

        cam = build_cam_payload(x_snapshot)
        cam_client.publish(TOPIC_CAM_IN, json.dumps(cam), qos=1)

        print(f"published CAM x={x_snapshot:.2f} topic={TOPIC_CAM_IN}")
        time.sleep(1.0)
