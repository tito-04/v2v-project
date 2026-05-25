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


def _actor_start(config: dict[str, Any]) -> dict[str, Any]:
    start = config.get("start")
    if isinstance(start, dict):
        return start
    return config


def _actor_key(kind: str, actor_id: str) -> str:
    if kind == "vehicle":
        return actor_id
    return f"{kind}:{actor_id}"


class WorldSimulation:
    def __init__(self, scenario: dict[str, Any], tick_seconds: float, step_meters: float) -> None:
        self.scenario = scenario
        self.tick_seconds = tick_seconds
        self.step_meters = step_meters
        self.default_speed = step_meters / tick_seconds
        self.elapsed_seconds = 0.0
        self.vehicles: dict[str, dict[str, Any]] = {}
        self._vehicle_configs: dict[str, dict[str, Any]] = {}
        self._obstacle_configs: dict[str, dict[str, Any]] = {}
        self._obstacles: dict[str, dict[str, Any]] = {}
        self.route_indexes: dict[str, int] = {}
        self.controls: dict[str, dict[str, Any]] = {}
        self.reset()

    def reset(self) -> None:
        self.elapsed_seconds = 0.0
        self.vehicles = {}
        self._vehicle_configs = {}
        self._obstacle_configs = {}
        self._obstacles = {}
        self.route_indexes = {}
        self.controls = {}

        for name, config in self.scenario["vehicles"].items():
            self._vehicle_configs[name] = config
            has_start_delay = float(config.get("start_delay_seconds", 0.0)) > 0.0
            self.vehicles[name] = self._initial_actor_state(
                actor_id=name,
                kind="vehicle",
                config=config,
                default_status="waiting" if has_start_delay else "moving",
            )
            if has_start_delay:
                self.vehicles[name]["speed"] = 0.0
                self.vehicles[name]["target_speed"] = 0.0
                self.vehicles[name]["reason"] = "start-delay"
            self.route_indexes[name] = 0

        for idx, config in enumerate(self.scenario.get("obstacles", []), start=1):
            obstacle_id = str(config.get("id", idx))
            self._obstacle_configs[obstacle_id] = config
            if config.get("triggered_by_vehicle_stop"):
                status = "waiting"
            else:
                status = "moving" if config.get("route") else "static"
            self._obstacles[obstacle_id] = self._initial_actor_state(
                actor_id=obstacle_id,
                kind=str(config.get("kind", "obstacle")),
                config=config,
                default_status=status,
            )
            self.route_indexes[_actor_key("obstacle", obstacle_id)] = 0

    def apply_control(
        self,
        vehicle_name: str,
        action: str,
        reason: str = "",
        risk_object_id: str | None = None,
        ttl_seconds: float = 1.5,
        stop_axis: str | None = None,
        stop_value: float | int | str | None = None,
        stop_direction: float | int | str | None = None,
        stop_x: float | int | str | None = None,
        stop_y: float | int | str | None = None,
        stop_heading: float | int | str | None = None,
        stop_route_index: int | str | None = None,
    ) -> None:
        if vehicle_name not in self.vehicles:
            return
        normalized_action = action.lower()
        if normalized_action in {"go", "resume", "clear"}:
            self._apply_resume_speed(vehicle_name)
            self.controls.pop(vehicle_name, None)
            return
        control: dict[str, Any] = {
            "action": normalized_action,
            "reason": reason,
            "risk_object_id": risk_object_id,
            "expires_at": self.elapsed_seconds + max(ttl_seconds, self.tick_seconds),
        }
        if stop_axis is not None:
            control["stop_axis"] = str(stop_axis)
        if stop_value is not None:
            control["stop_value"] = float(stop_value)
        if stop_direction is not None:
            control["stop_direction"] = float(stop_direction)
        if stop_x is not None:
            control["stop_x"] = float(stop_x)
        if stop_y is not None:
            control["stop_y"] = float(stop_y)
        if stop_heading is not None:
            control["stop_heading"] = float(stop_heading)
        if stop_route_index is not None:
            control["stop_route_index"] = int(stop_route_index)
        self.controls[vehicle_name] = control

    def _apply_resume_speed(self, vehicle_name: str) -> None:
        vehicle_config = self._vehicle_configs.get(vehicle_name, {})
        if "resume_speed_mps" not in vehicle_config:
            return
        vehicle = self.vehicles.get(vehicle_name)
        if not vehicle:
            return
        if vehicle.get("status") not in {"stopped", "braking"}:
            return
        resume_speed = float(vehicle_config["resume_speed_mps"])
        vehicle["base_speed"] = resume_speed
        vehicle["target_speed"] = resume_speed

    def tick(self) -> bool:
        self.elapsed_seconds += self.tick_seconds
        for name in self.vehicles:
            self._advance_vehicle(name)
        for obstacle_id in self._obstacles:
            self._advance_obstacle(obstacle_id)
        if self._should_reset():
            self.reset()
            return True
        return False

    def obstacles(self) -> list[dict[str, Any]]:
        return list(self._obstacles.values())

    def world_snapshot(self) -> dict[str, Any]:
        return {
            "vehicles": {name: dict(actor) for name, actor in self.vehicles.items()},
            "obstacles": {actor["id"]: dict(actor) for actor in self._obstacles.values()},
            "elapsed_seconds": self.elapsed_seconds,
        }

    def _initial_actor_state(
        self,
        actor_id: str,
        kind: str,
        config: dict[str, Any],
        default_status: str,
    ) -> dict[str, Any]:
        start = _actor_start(config)
        base_speed = float(config.get("speed_mps", self.default_speed if kind == "vehicle" else 0.0))
        dimensions = config.get("dimensions", {})
        return {
            "id": actor_id,
            "kind": kind,
            "x": float(start["x"]),
            "y": float(start["y"]),
            "heading": float(start.get("heading", config.get("heading", 0.0))),
            "speed": float(config.get("initial_speed_mps", base_speed if kind == "vehicle" else 0.0)),
            "target_speed": base_speed,
            "base_speed": base_speed,
            "status": default_status,
            "reason": "",
            "risk_object_id": None,
            "blocks_vehicle_path": bool(config.get("blocks_vehicle_path", kind in {"pedestrian", "obstacle"})),
            "width": float(dimensions.get("y", dimensions.get("width", 2.0))),
            "length": float(dimensions.get("x", dimensions.get("length", 2.0))),
        }

    def _advance_vehicle(self, name: str) -> None:
        vehicle_config = self._vehicle_configs[name]
        vehicle = self.vehicles[name]
        if self.elapsed_seconds < float(vehicle_config.get("start_delay_seconds", 0.0)):
            vehicle["speed"] = 0.0
            vehicle["target_speed"] = 0.0
            vehicle["status"] = "waiting"
            vehicle["reason"] = "start-delay"
            vehicle["risk_object_id"] = None
            return

        control = self._active_control(name)
        if control and control["action"] == "stop_at":
            if self._advance_vehicle_to_stop_line(name, control):
                return

        if control and control["action"] in {"stop", "brake", "stop_at"}:
            vehicle["target_speed"] = 0.0
            vehicle["status"] = "braking" if vehicle["speed"] > 0.05 else "stopped"
            vehicle["reason"] = control.get("reason", "")
            vehicle["risk_object_id"] = control.get("risk_object_id")
        else:
            vehicle["target_speed"] = vehicle["base_speed"]
            vehicle["status"] = "moving" if vehicle["target_speed"] > 0 else "stopped"
            vehicle["reason"] = ""
            vehicle["risk_object_id"] = None
        self._advance_actor(
            actor=vehicle,
            config=vehicle_config,
            route_key=name,
            acceleration=float(vehicle_config.get("accel_mps2", 9999.0)),
            deceleration=float(vehicle_config.get("decel_mps2", 9999.0)),
        )
        if float(vehicle.get("target_speed", 0.0)) == 0.0 and float(vehicle.get("speed", 0.0)) == 0.0:
            vehicle["status"] = "stopped"

    def _advance_vehicle_to_stop_line(self, name: str, control: dict[str, Any]) -> bool:
        vehicle_config = self._vehicle_configs[name]
        vehicle = self.vehicles[name]
        stop_line = self._stop_line_for_control(vehicle_config, control)
        if not stop_line:
            return False

        route = vehicle_config.get("route", [])
        if not route:
            return False
        route_idx = min(self.route_indexes[name], len(route) - 1)
        segment = route[route_idx]
        vehicle["reason"] = control.get("reason", "")
        vehicle["risk_object_id"] = control.get("risk_object_id")
        axis = str(stop_line["axis"])
        if segment.get("axis") != axis:
            self._park_at_stop_line(name, vehicle, stop_line)
            return True

        route_index = stop_line.get("route_index")
        if isinstance(route_index, int) and route_idx != route_index:
            self._park_at_stop_line(name, vehicle, stop_line)
            return True

        direction = float(stop_line["direction"])
        stop_value = float(stop_line["value"])
        current_value = float(vehicle[axis])
        distance_to_line = (stop_value - current_value) * direction

        if distance_to_line <= 0.0:
            self._park_at_stop_line(name, vehicle, stop_line)
            return True

        target_speed = float(vehicle.get("base_speed", 0.0))
        current_speed = max(float(vehicle.get("speed", 0.0)), 0.0)
        acceleration = float(vehicle_config.get("accel_mps2", 9999.0))
        vehicle["target_speed"] = target_speed
        vehicle["speed"] = self._approach(current_speed, target_speed, acceleration * self.tick_seconds)
        vehicle["heading"] = float(segment.get("heading", vehicle.get("heading", 0.0)))

        step = direction * float(vehicle["speed"]) * self.tick_seconds
        next_value = current_value + step
        crossed = (stop_value - next_value) * direction <= 0.0
        if crossed:
            self._park_at_stop_line(name, vehicle, stop_line)
        else:
            vehicle[axis] = next_value
            vehicle["status"] = "approaching-stop"
        return True

    def _stop_line_for_control(
        self,
        vehicle_config: dict[str, Any],
        control: dict[str, Any],
    ) -> dict[str, float | str] | None:
        configured = vehicle_config.get("cooperative_stop", {})
        axis = control.get("stop_axis", configured.get("stop_axis"))
        value = control.get("stop_value", configured.get("stop_value"))
        direction = control.get("stop_direction", configured.get("stop_direction", 1))
        if axis not in {"x", "y"} or value is None:
            return None
        direction = float(direction)
        if direction == 0.0:
            return None
        pose = configured.get("stop_pose", {})
        if not isinstance(pose, dict):
            pose = {}
        stop_line: dict[str, Any] = {"axis": axis, "value": float(value), "direction": direction, "pose": dict(pose)}
        if "stop_x" in control:
            stop_line["pose"]["x"] = control["stop_x"]
        if "stop_y" in control:
            stop_line["pose"]["y"] = control["stop_y"]
        if "stop_heading" in control:
            stop_line["pose"]["heading"] = control["stop_heading"]
        if "stop_route_index" in control:
            stop_line["pose"]["route_index"] = control["stop_route_index"]
        if "route_index" in stop_line["pose"]:
            stop_line["route_index"] = int(stop_line["pose"]["route_index"])
        return stop_line

    def _park_at_stop_line(self, route_key: str, vehicle: dict[str, Any], stop_line: dict[str, Any]) -> None:
        axis = str(stop_line["axis"])
        vehicle[axis] = float(stop_line["value"])
        pose = stop_line.get("pose", {})
        if isinstance(pose, dict):
            for pose_axis in ("x", "y", "heading"):
                if pose_axis in pose:
                    vehicle[pose_axis] = float(pose[pose_axis])
            if "route_index" in pose:
                route = self._vehicle_configs[route_key].get("route", [])
                max_index = max(len(route) - 1, 0)
                self.route_indexes[route_key] = min(max(int(pose["route_index"]), 0), max_index)
        vehicle["speed"] = 0.0
        vehicle["target_speed"] = 0.0
        vehicle["status"] = "stopped"

    def _advance_obstacle(self, obstacle_id: str) -> None:
        obstacle_config = self._obstacle_configs[obstacle_id]
        obstacle = self._obstacles[obstacle_id]
        if not obstacle_config.get("route"):
            obstacle["speed"] = 0.0
            obstacle["target_speed"] = 0.0
            obstacle["status"] = "static"
            return
        if obstacle["status"] == "done":
            obstacle["speed"] = 0.0
            obstacle["target_speed"] = 0.0
            return
        if obstacle_config.get("triggered_by_vehicle_stop") and obstacle["status"] == "waiting":
            if not self._triggered_obstacle_should_cross(obstacle_id, obstacle_config):
                obstacle["speed"] = 0.0
                obstacle["target_speed"] = 0.0
                return
            obstacle["status"] = "crossing"

        obstacle["target_speed"] = obstacle["base_speed"]
        if obstacle["status"] != "crossing":
            obstacle["status"] = "moving" if obstacle["target_speed"] > 0 else "stopped"
        self._advance_actor(
            actor=obstacle,
            config=obstacle_config,
            route_key=_actor_key("obstacle", obstacle_id),
            acceleration=float(obstacle_config.get("accel_mps2", 9999.0)),
            deceleration=float(obstacle_config.get("decel_mps2", 9999.0)),
        )

    def _triggered_obstacle_should_cross(self, obstacle_id: str, config: dict[str, Any]) -> bool:
        vehicle_name = config.get("triggered_by_vehicle_stop")
        if not isinstance(vehicle_name, str):
            return True
        vehicle = self.vehicles.get(vehicle_name)
        if not vehicle:
            return False
        expected_risk = config.get("trigger_risk_object_id", obstacle_id)
        return (
            vehicle.get("status") == "stopped"
            and vehicle.get("risk_object_id") == expected_risk
        )

    def _active_control(self, vehicle_name: str) -> dict[str, Any] | None:
        control = self.controls.get(vehicle_name)
        if not control:
            return None
        if control["expires_at"] < self.elapsed_seconds:
            self.controls.pop(vehicle_name, None)
            return None
        return control

    def _advance_actor(
        self,
        actor: dict[str, Any],
        config: dict[str, Any],
        route_key: str,
        acceleration: float,
        deceleration: float,
    ) -> bool:
        route = config.get("route", [])
        if not route:
            return False

        target_speed = max(float(actor.get("target_speed", 0.0)), 0.0)
        current_speed = max(float(actor.get("speed", 0.0)), 0.0)
        rate = acceleration if target_speed >= current_speed else deceleration
        actor["speed"] = self._approach(current_speed, target_speed, rate * self.tick_seconds)
        if actor["speed"] <= 1e-6:
            actor["speed"] = 0.0
            return False

        route_idx = min(self.route_indexes[route_key], len(route) - 1)
        segment = route[route_idx]
        axis = segment["axis"]
        direction = float(segment["direction"])
        actor["heading"] = float(segment.get("heading", actor.get("heading", 0.0)))
        actor[axis] += direction * actor["speed"] * self.tick_seconds

        until = segment.get("until")
        if isinstance(until, dict):
            op = until.get("op") or (">=" if direction > 0 else "<=")
            if compare(float(actor[axis]), op, float(until["value"])):
                on_reach = segment.get("on_reach", {})
                snap = on_reach.get("snap", {})
                for snap_axis in ("x", "y", "heading"):
                    if snap_axis in snap:
                        actor[snap_axis] = float(snap[snap_axis])
                if on_reach.get("next_segment"):
                    if self.route_indexes[route_key] < len(route) - 1:
                        self.route_indexes[route_key] += 1
                    elif config.get("loop_route"):
                        self.route_indexes[route_key] = 0
                    else:
                        actor["speed"] = 0.0
                        actor["target_speed"] = 0.0
                        actor["status"] = str(config.get("post_route_status", actor.get("status", "stopped")))
                        return True
                    next_segment = route[self.route_indexes[route_key]]
                    actor["heading"] = float(next_segment.get("heading", actor["heading"]))
                elif route_idx >= len(route) - 1 and config.get("post_route_status"):
                    actor["speed"] = 0.0
                    actor["target_speed"] = 0.0
                    actor["status"] = str(config["post_route_status"])
                    return True
        return False

    @staticmethod
    def _approach(current: float, target: float, max_delta: float) -> float:
        if max_delta <= 0:
            return target
        if current < target:
            return min(target, current + max_delta)
        return max(target, current - max_delta)

    def _should_reset(self) -> bool:
        reset_config = self.scenario.get("reset", {})
        conditions = reset_config.get("conditions", [])
        if not conditions:
            return False

        results = []
        for condition in conditions:
            actor = self.vehicles[condition["vehicle"]]
            value = float(actor[condition["axis"]])
            results.append(compare(value, condition.get("op", ">="), float(condition["value"])))

        if reset_config.get("mode", "any") == "all":
            return all(results)
        return any(results)
