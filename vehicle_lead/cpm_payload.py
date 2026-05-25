import time
import math
from typing import Any


def meters_to_deg_lon(meters: float, latitude_deg: float) -> float:
    denom = 111320.0 * math.cos(math.radians(latitude_deg))
    if abs(denom) < 1e-9:
        return 0.0
    return meters / denom


def meters_to_deg_lat(meters: float) -> float:
    return meters / 111320.0


def build_cpm_payload(
    lead_x: float,
    lead_y: float,
    perceived: list[dict[str, Any]],
    *,
    base_lat: float,
    base_lon: float,
    fov_range_m: float,
    fov_half_angle_deg: float,
) -> dict[str, Any]:
    lat = base_lat + meters_to_deg_lat(lead_y)
    lon = base_lon + meters_to_deg_lon(lead_x, base_lat)
    objects = []
    for idx, obj in enumerate(perceived):
        dx = round(float(obj["x"]) - lead_x, 2)
        dy = round(float(obj["y"]) - lead_y, 2)
        public_id = obj.get("object_id") or obj.get("id")
        item = {
            "objectId": idx + 1,
            "kind": str(obj.get("kind", "obstacle")),
            "blocksVehiclePath": bool(obj.get("blocks_vehicle_path", False)),
            "sensorIdList": [1],
            "measurementDeltaTime": 0,
            "position": {
                "xCoordinate": {"value": dx, "confidence": 1},
                "yCoordinate": {"value": dy, "confidence": 1},
            },
            "velocity": {
                "cartesianVelocity": {
                    "xVelocity": {"value": float(obj.get("speed", 0.0)), "confidence": 1},
                    "yVelocity": {"value": 0.0, "confidence": 1},
                }
            },
            "objectDimensionX": {"value": 2.0, "confidence": 1},
            "objectDimensionY": {"value": 2.0, "confidence": 1},
        }
        if public_id is not None:
            item["objectPublicId"] = str(public_id)
        objects.append(item)
    return {
        "managementContainer": {
            "referenceTime": int((time.time() * 1000.0) % 65536),
            "referencePosition": {
                "latitude": lat,
                "longitude": lon,
                "positionConfidenceEllipse": {
                    "semiMajorConfidence": 4095,
                    "semiMajorOrientation": 0,
                    "semiMinorConfidence": 4095,
                },
                "altitude": {"altitudeValue": 800001, "altitudeConfidence": 15},
            },
        },
        "cpmContainers": [
            {
                "containerId": 3,
                "containerData": [{
                    "sensorId": 1,
                    "sensorType": 1,
                    "perceptionRegionShape": {
                        "radial": {
                            "range": int(fov_range_m),
                            "horizontalOpeningAngleStart": int(90 - fov_half_angle_deg),
                            "horizontalOpeningAngleEnd": int(90 + fov_half_angle_deg),
                        }
                    },
                    "shadowingApplies": False,
                }],
            },
            {
                "containerId": 5,
                "containerData": {
                    "numberOfPerceivedObjects": len(objects),
                    "perceivedObjects": objects,
                },
            },
        ],
    }
