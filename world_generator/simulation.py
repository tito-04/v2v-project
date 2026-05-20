from typing import Any


def compare(value: float, op: str, target: float) -> bool:
    if op == ">":
        return value > target
    if op == ">=":
        return value >= target
    if op == "<":
        return value < target
    if op == "<=":
        return value <= target
    if op == "==":
        return value == target
    raise ValueError(f"unsupported comparison operator: {op}")


class WorldSimulation:
    def __init__(self, scenario: dict[str, Any], tick_seconds: float, step_meters: float) -> None:
        self.scenario = scenario
        self.tick_seconds = tick_seconds
        self.step_meters = step_meters
        self.speed = step_meters / tick_seconds
        self.vehicles: dict[str, dict[str, Any]] = {}
        self.route_indexes: dict[str, int] = {}
        self.reset()

    def reset(self) -> None:
        self.vehicles = {}
        self.route_indexes = {}
        for name, config in self.scenario["vehicles"].items():
            start = config["start"]
            self.vehicles[name] = {
                "x": float(start["x"]),
                "y": float(start["y"]),
                "heading": float(start.get("heading", 0.0)),
                "speed": 0.0,
            }
            self.route_indexes[name] = 0

    def tick(self) -> bool:
        for name in self.vehicles:
            self._advance_vehicle(name)
        if self._should_reset():
            self.reset()
            return True
        return False

    def obstacles(self) -> list[dict[str, Any]]:
        return list(self.scenario.get("obstacles", []))

    def _advance_vehicle(self, name: str) -> None:
        vehicle_config = self.scenario["vehicles"][name]
        route = vehicle_config["route"]
        route_idx = min(self.route_indexes[name], len(route) - 1)
        segment = route[route_idx]
        vehicle = self.vehicles[name]

        axis = segment["axis"]
        direction = float(segment["direction"])
        vehicle["heading"] = float(segment.get("heading", vehicle.get("heading", 0.0)))
        vehicle["speed"] = abs(self.speed)
        vehicle[axis] += direction * self.step_meters

        until = segment.get("until")
        if isinstance(until, dict):
            op = until.get("op") or (">=" if direction > 0 else "<=")
            if compare(float(vehicle[axis]), op, float(until["value"])):
                on_reach = segment.get("on_reach", {})
                snap = on_reach.get("snap", {})
                for snap_axis in ("x", "y", "heading"):
                    if snap_axis in snap:
                        vehicle[snap_axis] = float(snap[snap_axis])
                if on_reach.get("next_segment") and self.route_indexes[name] < len(route) - 1:
                    self.route_indexes[name] += 1
                    next_segment = route[self.route_indexes[name]]
                    vehicle["heading"] = float(next_segment.get("heading", vehicle["heading"]))

    def _should_reset(self) -> bool:
        reset_config = self.scenario.get("reset", {})
        conditions = reset_config.get("conditions", [])
        if not conditions:
            return False

        results = []
        for condition in conditions:
            vehicle = self.vehicles[condition["vehicle"]]
            value = float(vehicle[condition["axis"]])
            results.append(compare(value, condition.get("op", ">="), float(condition["value"])))

        if reset_config.get("mode", "any") == "all":
            return all(results)
        return any(results)
