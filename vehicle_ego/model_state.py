from typing import Any


def upsert_model_object(
    objects: dict[str, dict[str, Any]],
    key: str,
    item: dict[str, Any],
    source_priority: int,
) -> None:
    existing = objects.get(key)
    if existing and int(existing.get("source_priority", 0)) > source_priority and not existing.get("stale", False):
        existing["updated_at"] = max(float(existing.get("updated_at", 0.0)), float(item.get("updated_at", 0.0)))
        existing["stale"] = False
        existing["secondary_sources"] = sorted(set(existing.get("secondary_sources", []) + [item.get("source", "unknown")]))
        return

    item["source_priority"] = source_priority
    if existing:
        secondary = set(existing.get("secondary_sources", []))
        secondary.add(existing.get("source", "unknown"))
        item["secondary_sources"] = sorted(source for source in secondary if source != item.get("source"))
    objects[key] = item


def model_key_for_world_candidate(source_key: str, obj: dict[str, Any]) -> str:
    if obj.get("kind") == "vehicle" and source_key == "vehicle_lead":
        return "lead"
    if obj.get("id") == "lead":
        return "lead"
    if obj.get("kind") == "pedestrian" or obj.get("id") == "pedestrian-1":
        return str(obj.get("id", source_key))
    return source_key


def cam_model_key(objects: dict[str, dict[str, Any]], x: float, y: float, station_id: Any) -> str:
    lead = objects.get("lead")
    if lead:
        dist = ((x - float(lead.get("x", 0.0))) ** 2 + (y - float(lead.get("y", 0.0))) ** 2) ** 0.5
        if dist < 8.0 or lead.get("source") == "direct":
            return "lead"
    return f"cam_{station_id}" if station_id is not None else "cam_unknown"
