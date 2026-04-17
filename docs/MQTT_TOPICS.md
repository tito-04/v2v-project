# MQTT Topic Contract

## Main Broker

- Topic: `world/pos/lead`
  - Producer: world-generator
  - Consumer: vehicle-lead
  - Payload: `{ "x": float, "timestamp": unix_seconds }`

- Topic: `world/pos/ego`
  - Producer: world-generator
  - Consumer: vehicle-ego
  - Payload: `{ "x": float, "timestamp": unix_seconds }`

## Lead Broker

- Topic: `vanetza/in/cam`
  - Producer: vehicle-lead
  - Consumer: lead-vanetza
  - Payload: JSON CAM as expected by vanetza-nap

## Ego Broker

- Topic: `vanetza/out/cam`
  - Producer: ego-vanetza
  - Consumer: vehicle-ego
  - Payload: decoded JSON CAM from vanetza-nap

## CPM (reserved for later phase)

- Topic: `vanetza/in/cpm`
- Topic: `vanetza/out/cpm`
