import json
import os
import time

import paho.mqtt.client as mqtt


MAIN_BROKER_HOST = os.getenv("MAIN_BROKER_HOST", "main-broker")
MAIN_BROKER_PORT = int(os.getenv("MAIN_BROKER_PORT", "1883"))
TOPIC_LEAD = os.getenv("WORLD_TOPIC_LEAD", "world/pos/lead")
TOPIC_EGO = os.getenv("WORLD_TOPIC_EGO", "world/pos/ego")
TICK_SECONDS = float(os.getenv("WORLD_TICK_SECONDS", "1.0"))
X_STEP = float(os.getenv("X_STEP_METERS", "2.0"))

lead_x = float(os.getenv("LEAD_START_X", "50.0"))
ego_x = float(os.getenv("EGO_START_X", "20.0"))


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


def publish_position(client: mqtt.Client, topic: str, x_value: float) -> None:
    payload = {
        "x": x_value,
        "timestamp": time.time(),
    }
    client.publish(topic, json.dumps(payload), qos=1)


if __name__ == "__main__":
    client = connect_client()

    while True:
        lead_x += X_STEP
        ego_x += X_STEP

        publish_position(client, TOPIC_LEAD, lead_x)
        publish_position(client, TOPIC_EGO, ego_x)

        print(f"tick lead_x={lead_x:.2f} ego_x={ego_x:.2f}")
        time.sleep(TICK_SECONDS)
