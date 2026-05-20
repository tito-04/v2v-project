# MQTT Topic Contract

This project follows vanetza-nap CAM/CPM input/output conventions. The UI acts as an ego observer view: the ego position comes from `world/pos/ego`, and the lead position comes from CAMs received by the ego.

## Main Broker

- Topic: `world/pos/lead`
  - Producer: world-generator
  - Consumer: vehicle-lead
  - Payload: `{ "x": float, "y": float, "heading": float, "speed": float, "timestamp": unix_seconds }`

- Topic: `world/pos/ego`
  - Producer: world-generator
  - Consumer: vehicle-ego
  - Payload: `{ "x": float, "y": float, "heading": float, "speed": float, "timestamp": unix_seconds }`

- Topic: `world/pos/obstacle/<id>`
  - Producer: world-generator
  - Consumers: vehicle-lead, vehicle-ego
  - Payload: `{ "x": float, "y": float, "heading": float, "speed": float, "timestamp": unix_seconds }`

- Topic: `world/scenario`
  - Producer: world-generator
  - Consumer: vehicle-ego
  - Payload: scenario metadata used by the UI (`name`, `title`, `description`, `layout`)

## Lead Broker

- Topic: `vanetza/in/cam`
  - Producer: vehicle-lead
  - Consumer: lead-vanetza
  - Payload: JSON CAM as expected by vanetza-nap (input message without ITS PDU header)

- Topic: `vanetza/time/cam`
  - Producer: lead-vanetza (if enabled)
  - Consumer: vehicle-ego
  - Payload: CAM timing metadata (used for end-to-end latency metrics)

## Ego Broker

- Topic: `vanetza/out/cam`
  - Producer: ego-vanetza
  - Consumer: vehicle-ego
  - Payload: decoded JSON CAM from vanetza-nap (output message with header and metadata)

## CPM

- Topic: `vanetza/in/cpm`
  - Producer: vehicle-lead
  - Consumer: lead-vanetza
  - Payload: JSON CPM generated from objects inside the lead FoV and not blocked by scenario occluders

- Topic: `vanetza/out/cpm`
  - Producer: ego-vanetza
  - Consumer: vehicle-ego
  - Payload: decoded JSON CPM forwarded over V2V
