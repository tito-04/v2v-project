import math
from typing import Any


def segment_intersects_rect(
    p1: tuple[float, float],
    p2: tuple[float, float],
    rx1: float,
    ry1: float,
    rx2: float,
    ry2: float,
) -> bool:
    xmin, xmax = min(rx1, rx2), max(rx1, rx2)
    ymin, ymax = min(ry1, ry2), max(ry1, ry2)

    def inside(p: tuple[float, float]) -> bool:
        return xmin <= p[0] <= xmax and ymin <= p[1] <= ymax

    if inside(p1) or inside(p2):
        return True

    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    t_enter, t_exit = 0.0, 1.0

    for p_val, q_val in (
        (-dx, p1[0] - xmin),
        (dx, xmax - p1[0]),
        (-dy, p1[1] - ymin),
        (dy, ymax - p1[1]),
    ):
        if p_val == 0.0:
            if q_val < 0.0:
                return False
        elif p_val < 0.0:
            t_enter = max(t_enter, q_val / p_val)
        else:
            t_exit = min(t_exit, q_val / p_val)

    return t_enter <= t_exit


def is_in_fov(
    vehicle_x: float,
    vehicle_y: float,
    heading_deg: float,
    obj_x: float,
    obj_y: float,
    fov_range_m: float,
    fov_half_angle_deg: float,
    occluders: list[dict[str, Any]] | None = None,
) -> bool:
    dx = obj_x - vehicle_x
    dy = obj_y - vehicle_y
    distance = math.hypot(dx, dy)
    if distance > fov_range_m:
        return False
    if distance < 0.1:
        return True

    obj_angle_deg = math.degrees(math.atan2(dy, dx))
    rel_angle = (obj_angle_deg - heading_deg + 360.0) % 360.0
    if rel_angle > 180.0:
        rel_angle -= 360.0
    if abs(rel_angle) > fov_half_angle_deg:
        return False

    return not any(
        segment_intersects_rect(
            (vehicle_x, vehicle_y),
            (obj_x, obj_y),
            float(occluder["x1"]),
            float(occluder["y1"]),
            float(occluder["x2"]),
            float(occluder["y2"]),
        )
        for occluder in occluders or []
        if occluder.get("type") == "rect"
    )


def vehicle_frame(
    vehicle_x: float,
    vehicle_y: float,
    heading_deg: float,
    obj_x: float,
    obj_y: float,
) -> tuple[float, float]:
    heading_rad = math.radians(heading_deg)
    dx = obj_x - vehicle_x
    dy = obj_y - vehicle_y
    forward = dx * math.cos(heading_rad) + dy * math.sin(heading_rad)
    lateral = -dx * math.sin(heading_rad) + dy * math.cos(heading_rad)
    return forward, lateral


def is_in_path_corridor(
    vehicle_x: float,
    vehicle_y: float,
    heading_deg: float,
    obj_x: float,
    obj_y: float,
    lookahead_m: float = 60.0,
    half_width_m: float = 6.0,
) -> bool:
    forward, lateral = vehicle_frame(vehicle_x, vehicle_y, heading_deg, obj_x, obj_y)
    return 0.0 <= forward <= lookahead_m and abs(lateral) <= half_width_m


def first_risk_object(
    vehicle: dict[str, Any],
    objects: list[dict[str, Any]],
    lookahead_m: float = 60.0,
    half_width_m: float = 6.0,
) -> dict[str, Any] | None:
    candidates = []
    for obj in objects:
        if not obj.get("blocks_vehicle_path", False):
            continue
        if not is_in_path_corridor(
            float(vehicle["x"]),
            float(vehicle.get("y", 0.0)),
            float(vehicle.get("heading", 0.0)),
            float(obj["x"]),
            float(obj.get("y", 0.0)),
            lookahead_m=lookahead_m,
            half_width_m=half_width_m,
        ):
            continue
        forward, lateral = vehicle_frame(
            float(vehicle["x"]),
            float(vehicle.get("y", 0.0)),
            float(vehicle.get("heading", 0.0)),
            float(obj["x"]),
            float(obj.get("y", 0.0)),
        )
        candidates.append((forward, abs(lateral), obj))

    if not candidates:
        return None
    candidates.sort(key=lambda item: (item[0], item[1]))
    return candidates[0][2]
