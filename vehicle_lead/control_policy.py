from typing import Any


PEDESTRIAN_RISK_ID = "pedestrian-1"


def public_object_id(obj: dict[str, Any], fallback: str = "") -> str:
    object_id = str(obj.get("id") or obj.get("object_id") or fallback)
    if object_id.startswith("obstacle_"):
        return object_id.removeprefix("obstacle_")
    return object_id


def world_object_status(world_objects: dict[str, dict[str, Any]], object_id: str) -> str | None:
    for key, obj in world_objects.items():
        if public_object_id(obj, key) == object_id:
            status = obj.get("status")
            return str(status) if status is not None else None
    return None


def lead_control_risk(
    risk: dict[str, Any] | None,
    world_objects: dict[str, dict[str, Any]],
    held_risk_id: str | None,
) -> tuple[dict[str, Any] | None, str | None]:
    if held_risk_id:
        if world_object_status(world_objects, held_risk_id) == "done":
            held_risk_id = None
        else:
            return {"object_id": held_risk_id}, held_risk_id

    if not risk:
        return None, None

    risk_id = public_object_id(risk)
    normalized_risk = dict(risk)
    normalized_risk["object_id"] = risk_id
    if risk_id == PEDESTRIAN_RISK_ID and world_object_status(world_objects, risk_id) != "done":
        return normalized_risk, risk_id
    return normalized_risk, None
