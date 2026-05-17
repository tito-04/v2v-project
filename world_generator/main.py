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
SPEED = X_STEP / TICK_SECONDS

# Intersection geometry
INTERSECTION_X = float(os.getenv("INTERSECTION_X", "200"))
INTERSECTION_Y = float(os.getenv("INTERSECTION_Y", "0"))
LANE_W = float(os.getenv("LANE_W", "4"))

# Start positions
EGO_START_X = float(os.getenv("EGO_START_X", "20"))
EGO_START_Y = float(os.getenv("EGO_START_Y", "-4"))
LEAD_START_X = float(os.getenv("LEAD_START_X", "204"))
LEAD_START_Y = float(os.getenv("LEAD_START_Y", "200"))

# Reset boundary (south — negative Y)
WORLD_LENGTH_S = float(os.getenv("WORLD_LENGTH_S", "200"))

# Static obstacles
NUM_OBSTACLES = int(os.getenv("NUM_OBSTACLES", "1"))
OBSTACLES = []
for _i in range(1, NUM_OBSTACLES + 1):
    obs_x = float(os.getenv(f"OBSTACLE_{_i}_X", "204"))
    obs_y = float(os.getenv(f"OBSTACLE_{_i}_Y", "-14"))
    OBSTACLES.append({"x": obs_x, "y": obs_y, "idx": _i})


def connect_client() -> mqtt.Client:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="world-generator")
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
                retry_delay = min(retry_delay * 2, 30)
            else:
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


def reset_vehicles() -> tuple[dict, dict]:
    ego = {"x": EGO_START_X, "y": EGO_START_Y, "heading": 0.0, "phase": "east"}
    lead = {"x": LEAD_START_X, "y": LEAD_START_Y, "heading": 270.0}
    return ego, lead


if __name__ == "__main__":
    client = connect_client()
    ego, lead = reset_vehicles()

    while True:
        # --- Ego movement ---
        if ego["phase"] == "east":
            ego["x"] += X_STEP
            if ego["x"] >= INTERSECTION_X:
                # Snap to southbound lane and turn south
                ego["x"] = INTERSECTION_X + LANE_W
                ego["y"] = EGO_START_Y  # still at y=-4 entering the turn
                ego["phase"] = "south"
                ego["heading"] = 270.0
                print(f"EGO: turned south at intersection, now at ({ego['x']:.1f}, {ego['y']:.1f})")
        elif ego["phase"] == "south":
            ego["y"] -= X_STEP

        # --- Lead movement (always south) ---
        lead["y"] -= X_STEP

        # --- Reset when both have exited south ---
        if ego["y"] < -WORLD_LENGTH_S and lead["y"] < -WORLD_LENGTH_S:
            ego, lead = reset_vehicles()
            print("--- WORLD LOOP RESET ---")

        # --- Publish positions ---
        publish_position(client, TOPIC_LEAD, lead["x"], lead["y"], lead["heading"], SPEED)
        publish_position(client, TOPIC_EGO, ego["x"], ego["y"], ego["heading"], SPEED)

        for obs in OBSTACLES:
            topic = f"world/pos/obstacle/{obs['idx']}"
            publish_position(client, topic, obs["x"], obs["y"], 0.0, 0.0)

        print(f"tick ego=({ego['x']:.1f},{ego['y']:.1f}) phase={ego['phase']} "
              f"lead=({lead['x']:.1f},{lead['y']:.1f})")
        time.sleep(TICK_SECONDS)
