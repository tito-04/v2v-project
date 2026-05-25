# MQTT Topic Contract

This project follows vanetza-nap CAM/CPM input/output conventions. The UI has a split view: `World Truth` consumes ground-truth world topics, while `Ego World Model` only uses ego pose, direct FoV perception, CAM, and CPM.

## Main Broker

- Topic: `world/pos/lead`
  - Producer: world-generator
  - Consumer: vehicle-lead
  - Payload: `{ "id": "lead", "kind": "vehicle", "x": float, "y": float, "heading": float, "speed": float, "target_speed": float, "status": string, "reason": string, "risk_object_id": string|null, "timestamp": unix_seconds, "updated_at": unix_seconds }`

- Topic: `world/pos/ego`
  - Producer: world-generator
  - Consumer: vehicle-ego
  - Payload: same shape as `world/pos/lead`

- Topic: `world/pos/obstacle/<id>`
  - Producer: world-generator
  - Consumers: vehicle-lead, vehicle-ego
  - Payload: `{ "id": string, "kind": "pedestrian"|"obstacle"|..., "x": float, "y": float, "heading": float, "speed": float, "status": string, "blocks_vehicle_path": bool, "width": float, "length": float, "timestamp": unix_seconds, "updated_at": unix_seconds }`

- Topic: `world/scenario`
  - Producer: world-generator
  - Consumer: vehicle-ego
  - Payload: scenario metadata used by the UI (`name`, `title`, `description`, `layout`)

- Topic: `world/control/<vehicle>`
  - Producers: vehicle-lead, vehicle-ego
  - Consumer: world-generator
  - Payload: `{ "action": "stop"|"stop_at"|"resume", "reason": string, "risk_object_id": string|null, "ttl_seconds": float, "timestamp": unix_seconds }`
  - `stop_at` may include `{ "stop_axis": "x"|"y", "stop_value": float, "stop_direction": -1|1, "stop_x": float, "stop_y": float, "stop_heading": float, "stop_route_index": int }`; the world keeps or parks the vehicle at that configured stop pose until the risk clears.

- Topic: `world/tx/cam` and `world/tx/cpm`
  - Producer: vehicle-lead
  - Consumer: vehicle-ego dashboard only
  - Payload: `{ "message_type": "cam"|"cpm", "sequence": int, "station": "lead", "generated_at": unix_seconds, "sent_at": unix_seconds, "object_count": int }`
  - Note: these topics are observer telemetry for packet delay/loss metrics. They are not used by the ego vehicle model.

## Lead Broker

- Topic: `vanetza/in/cam`
  - Producer: vehicle-lead
  - Consumer: lead-vanetza
  - Payload: JSON CAM as expected by vanetza-nap (input message without ITS PDU header)

- Topic: `vanetza/time/cam`
  - Producer: vehicle-lead
  - Consumer: vehicle-ego
  - Payload: CAM timing metadata fallback; packet metrics primarily use `world/tx/cam`

## Ego Broker

- Topic: `vanetza/out/cam`
  - Producer: ego-vanetza
  - Consumer: vehicle-ego
  - Payload: decoded JSON CAM from vanetza-nap (output message with header and metadata)

## CPM

- Topic: `vanetza/in/cpm`
  - Producer: vehicle-lead
  - Consumer: lead-vanetza
  - Payload: JSON CPM generated from blocking objects inside the lead FoV and not blocked by scenario occluders

- Topic: `vanetza/out/cpm`
  - Producer: ego-vanetza
  - Consumer: vehicle-ego
  - Payload: decoded JSON CPM forwarded over V2V
