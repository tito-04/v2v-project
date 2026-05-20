import copy
import json
import os
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENARIOS_DIR = PROJECT_ROOT / "scenarios"
DEFAULT_SCENARIO_NAME = "intersection-occlusion"


class ScenarioError(ValueError):
    pass


def _scenario_path(name: str) -> Path:
    clean_name = name.removesuffix(".json")
    if "/" in clean_name or "\\" in clean_name or not clean_name:
        raise ScenarioError(f"invalid scenario name: {name!r}")
    return SCENARIOS_DIR / f"{clean_name}.json"


def available_scenarios() -> list[str]:
    if not SCENARIOS_DIR.exists():
        return []
    return sorted(path.stem for path in SCENARIOS_DIR.glob("*.json"))


def load_scenario(name: str | None = None, file_path: str | None = None) -> dict[str, Any]:
    if file_path:
        path = Path(file_path)
        if not path.is_absolute():
            path = PROJECT_ROOT / path
    else:
        path = _scenario_path(name or DEFAULT_SCENARIO_NAME)

    try:
        with path.open("r", encoding="utf-8") as handle:
            scenario = json.load(handle)
    except FileNotFoundError as exc:
        raise ScenarioError(f"scenario file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ScenarioError(f"invalid scenario JSON in {path}: {exc}") from exc

    validate_scenario(scenario, path)
    return scenario


def load_scenario_from_env() -> dict[str, Any]:
    return load_scenario(
        name=os.getenv("SCENARIO_NAME") or DEFAULT_SCENARIO_NAME,
        file_path=os.getenv("SCENARIO_FILE") or None,
    )


def validate_scenario(scenario: dict[str, Any], path: Path | None = None) -> None:
    label = str(path) if path else scenario.get("name", "<memory>")
    if not isinstance(scenario.get("name"), str) or not scenario["name"]:
        raise ScenarioError(f"{label}: missing scenario name")

    vehicles = scenario.get("vehicles")
    if not isinstance(vehicles, dict):
        raise ScenarioError(f"{label}: missing vehicles object")
    for vehicle_name in ("ego", "lead"):
        vehicle = vehicles.get(vehicle_name)
        if not isinstance(vehicle, dict):
            raise ScenarioError(f"{label}: missing vehicle {vehicle_name}")
        start = vehicle.get("start")
        route = vehicle.get("route")
        if not isinstance(start, dict) or "x" not in start or "y" not in start:
            raise ScenarioError(f"{label}: vehicle {vehicle_name} needs start x/y")
        if not isinstance(route, list) or not route:
            raise ScenarioError(f"{label}: vehicle {vehicle_name} needs a non-empty route")
        for segment in route:
            if segment.get("axis") not in ("x", "y"):
                raise ScenarioError(f"{label}: route axis must be x or y")
            if float(segment.get("direction", 0)) == 0.0:
                raise ScenarioError(f"{label}: route direction cannot be zero")

    obstacles = scenario.get("obstacles", [])
    if not isinstance(obstacles, list):
        raise ScenarioError(f"{label}: obstacles must be a list")
    for idx, obstacle in enumerate(obstacles, start=1):
        if "x" not in obstacle or "y" not in obstacle:
            raise ScenarioError(f"{label}: obstacle {idx} needs x/y")


def public_metadata(scenario: dict[str, Any]) -> dict[str, Any]:
    return {
        "name": scenario["name"],
        "title": scenario.get("title", scenario["name"]),
        "description": scenario.get("description", ""),
        "layout": copy.deepcopy(scenario.get("layout", {})),
    }
