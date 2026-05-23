# V2V Project

This project implements a CAM + CPM V2V simulation with:

- three MQTT brokers (`main-broker`, `lead-broker`, `ego-broker`)
- two vanetza-nap stations (`lead-vanetza`, `ego-vanetza`)
- deterministic world generator with selectable scenarios
- lead vehicle CAM and CPM publisher
- ego vehicle split-screen 3D UI dashboard
- dynamic pedestrian occlusion and vehicle brake/stop reactions
- packet delay/loss observability for CAM and CPM
- runtime network impairment controls (netem)

## Quick start

1. Copy env file:

```bash
cp .env.example .env
```

2. Run preflight:

```bash
./scripts/preflight.sh
```

3. Start the default scenario:

```bash
./scripts/scenario.sh run intersection-occlusion
```

4. Open UI:

- http://localhost:18080

You can change the host port with `UI_HOST_PORT` in `.env`.

## Scenarios

Scenarios live in `scenarios/*.json`. They define vehicle starts/routes, static or moving obstacles, road layout, and optional rectangular occluders.

Obstacles can be legacy static objects (`x`, `y`) or dynamic actors with `kind`, `start`, `route`, `speed_mps`, `dimensions`, and `blocks_vehicle_path`.

Available initial scenarios:

- `intersection-occlusion`: dynamic intersection demo with an occluding building and a pedestrian that waits on the crosswalk until ego stops.
- `straight-obstacles`: old straight-road demo with ego and lead driving east past multiple obstacles.

CLI:

```bash
./scripts/scenario.sh list
./scripts/scenario.sh run straight-obstacles
./scripts/scenario.sh run intersection-occlusion
./scripts/scenario.sh smoke straight-obstacles
```

You can also set `SCENARIO_NAME` in `.env`, or set `SCENARIO_FILE` to a custom JSON file path.

## Useful commands

- Apply mild impairment on lead station:

```bash
./scripts/netem_profiles.sh apply mild lead-vanetza eth0
```

- Isolate delay or loss:

```bash
./scripts/netem_profiles.sh apply delay-only lead-vanetza eth0
./scripts/netem_profiles.sh apply loss-only lead-vanetza eth0
```

- Clear impairment:

```bash
./scripts/netem_profiles.sh clear lead-vanetza eth0
```
