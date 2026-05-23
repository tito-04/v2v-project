import json
import os
import time
from typing import Any

import paho.mqtt.client as mqtt

from world_generator.scenarios import load_scenario_from_env, public_metadata
from world_generator.simulation import WorldSimulation


MAIN_BROKER_HOST = os.getenv("MAIN_BROKER_HOST", "main-broker")
MAIN_BROKER_PORT = int(os.getenv("MAIN_BROKER_PORT", "1883"))
TOPIC_LEAD = os.getenv("WORLD_TOPIC_LEAD", "world/pos/lead")
TOPIC_EGO = os.getenv("WORLD_TOPIC_EGO", "world/pos/ego")
TOPIC_OBSTACLE = os.getenv("WORLD_TOPIC_OBSTACLE", "world/pos/obstacle")
TOPIC_SCENARIO = os.getenv("WORLD_TOPIC_SCENARIO", "world/scenario")
TOPIC_CONTROL = os.getenv("WORLD_TOPIC_CONTROL", "world/control")


def env_float(name: str, fallback: float) -> float:
    raw = os.getenv(name)
    if raw is None or raw == "":
        return fallback
    return float(raw)


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


def publish_actor_position(client: mqtt.Client, topic: str, actor: dict[str, Any]) -> None:
    timestamp = time.time()
    payload = {
        "id": actor.get("id"),
        "kind": actor.get("kind", "vehicle"),
        "x": actor["x"],
        "y": actor.get("y", 0.0),
        "heading": actor.get("heading", 0.0),
        "speed": actor.get("speed", 0.0),
        "target_speed": actor.get("target_speed", 0.0),
        "status": actor.get("status", "moving"),
        "reason": actor.get("reason", ""),
        "risk_object_id": actor.get("risk_object_id"),
        "blocks_vehicle_path": bool(actor.get("blocks_vehicle_path", False)),
        "width": actor.get("width"),
        "length": actor.get("length"),
        "timestamp": timestamp,
        "updated_at": timestamp,
    }
    client.publish(topic, json.dumps(payload), qos=1)


def publish_scenario(client: mqtt.Client, scenario: dict[str, Any]) -> None:
    payload = public_metadata(scenario)
    payload["timestamp"] = time.time()
    client.publish(TOPIC_SCENARIO, json.dumps(payload), qos=1, retain=True)


if __name__ == "__main__":
    scenario = load_scenario_from_env()
    defaults = scenario.get("defaults", {})
    tick_seconds = env_float("WORLD_TICK_SECONDS", float(defaults.get("tick_seconds", 1.0)))
    step_meters = env_float("X_STEP_METERS", float(defaults.get("step_meters", 2.0)))

    client = connect_client()
    simulation = WorldSimulation(scenario, tick_seconds, step_meters)

    def on_control(_client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
        try:
            vehicle_name = msg.topic.split("/")[-1]
            payload = json.loads(msg.payload.decode("utf-8"))
            simulation.apply_control(
                vehicle_name=vehicle_name,
                action=str(payload.get("action", "stop")),
                reason=str(payload.get("reason", "")),
                risk_object_id=payload.get("risk_object_id"),
                ttl_seconds=float(payload.get("ttl_seconds", max(tick_seconds * 3, 0.5))),
            )
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            print(f"control parse error: {exc}")

    client.on_message = lambda _c, _u, _m: None
    client.message_callback_add(f"{TOPIC_CONTROL}/+", on_control)
    client.subscribe(f"{TOPIC_CONTROL}/+", qos=1)

    publish_scenario(client, scenario)
    print(f"Loaded scenario: {scenario['name']}")

    while True:
        if simulation.tick():
            print("--- WORLD LOOP RESET ---")

        lead = simulation.vehicles["lead"]
        ego = simulation.vehicles["ego"]
        publish_actor_position(client, TOPIC_LEAD, lead)
        publish_actor_position(client, TOPIC_EGO, ego)

        for idx, obstacle in enumerate(simulation.obstacles(), start=1):
            obstacle_id = obstacle.get("id", idx)
            topic = f"{TOPIC_OBSTACLE}/{obstacle_id}"
            publish_actor_position(client, topic, obstacle)

        print(
            f"tick scenario={scenario['name']} "
            f"ego=({ego['x']:.1f},{ego['y']:.1f}) status={ego['status']} "
            f"lead=({lead['x']:.1f},{lead['y']:.1f}) status={lead['status']} "
            f"obstacles={len(simulation.obstacles())}"
        )
        time.sleep(tick_seconds)
