# V2V Project (CAM-first)

This project implements a CAM-first V2V simulation with:

- three MQTT brokers (`main-broker`, `lead-broker`, `ego-broker`)
- two vanetza-nap stations (`lead-vanetza`, `ego-vanetza`)
- deterministic world generator
- lead vehicle CAM publisher
- ego vehicle 3D UI dashboard
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

3. Start stack:

```bash
docker compose up --build
```

4. Open UI:

- http://localhost:18080

You can change the host port with `UI_HOST_PORT` in `.env`.

## Useful commands

- Apply mild impairment on lead station:

```bash
./scripts/netem_profiles.sh apply mild lead-vanetza eth0
```

- Clear impairment:

```bash
./scripts/netem_profiles.sh clear lead-vanetza eth0
```

## Next phase

CPM integration is intentionally deferred until CAM + 3D + impairment demo is stable.
