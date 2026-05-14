import json
import os
import time

import paho.mqtt.client as mqtt


MAIN_BROKER_HOST = os.getenv("MAIN_BROKER_HOST", "main-broker")
MAIN_BROKER_PORT = int(os.getenv("MAIN_BROKER_PORT", "1883"))
TOPIC_LEAD = os.getenv("WORLD_TOPIC_LEAD", "world/pos/lead")
TOPIC_EGO = os.getenv("WORLD_TOPIC_EGO", "world/pos/ego")
TOPIC_OBSTACLE = os.getenv("WORLD_TOPIC_OBSTACLE", "world/pos/obstacle")
TICK_SECONDS = float(os.getenv("WORLD_TICK_SECONDS", "1.0"))
X_STEP = float(os.getenv("X_STEP_METERS", "2.0"))

lead_x = float(os.getenv("LEAD_START_X", "50.0"))
ego_x = float(os.getenv("EGO_START_X", "20.0"))
OBSTACLE_X = float(os.getenv("OBSTACLE_X", "100.0"))
OBSTACLE_Y = float(os.getenv("OBSTACLE_Y", "0.0"))
SPEED = X_STEP / TICK_SECONDS
WORLD_LENGTH = float(os.getenv("WORLD_LENGTH", "500.0"))
LEAD_START_X = float(os.getenv("LEAD_START_X", "50.0"))
EGO_START_X = float(os.getenv("EGO_START_X", "20.0"))


def connect_client() -> mqtt.Client:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="world-generator")
    
    # Retry with exponential backoff
    max_retries = 10
    retry_delay = 1
    for attempt in range(max_retries):
        try:
            client.connect(MAIN_BROKER_HOST, MAIN_BROKER_PORT, keepalive=30)
            client.loop_start()
            print(f"Connected to {MAIN_BROKER_HOST}:{MAIN_BROKER_PORT}")
            return client
        except Exception as e:
            if attempt < max_retries - 1:
                print(f"Connection attempt {attempt + 1}/{max_retries} failed: {e}. Retrying in {retry_delay}s...")
                time.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, 30)  # Cap at 30s
            else:
                print(f"Failed to connect after {max_retries} attempts. Giving up.")
                raise


def publish_position(client: mqtt.Client, topic: str, x: float, y: float, heading: float, speed: float) -> None:
    payload = {
        "x": x,
        "y": y,
        "heading": heading,
        "speed": speed,
        "timestamp": time.time(),
    }
    client.publish(topic, json.dumps(payload), qos=1)


if __name__ == "__main__":
    client = connect_client()

    while True:
        lead_x += X_STEP
        ego_x += X_STEP

        if lead_x >= WORLD_LENGTH:
            lead_x = LEAD_START_X
            ego_x = EGO_START_X
            print("--- WORLD LOOP RESET ---")

        publish_position(client, TOPIC_LEAD, lead_x, 0.0, 0.0, SPEED)
        publish_position(client, TOPIC_EGO, ego_x, 0.0, 0.0, SPEED)
        publish_position(client, TOPIC_OBSTACLE, OBSTACLE_X, OBSTACLE_Y, 0.0, 0.0)

        print(f"tick lead_x={lead_x:.2f} ego_x={ego_x:.2f} obstacle=({OBSTACLE_X:.2f},{OBSTACLE_Y:.2f})")
        time.sleep(TICK_SECONDS)
