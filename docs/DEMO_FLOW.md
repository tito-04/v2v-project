# Demo Flow (Scenarios + CAM/CPM + Network Impairment)

## Scenario selection

- List scenarios:

```bash
./scripts/scenario.sh list
```

- Run the intersection demo:

```bash
./scripts/scenario.sh run intersection-occlusion
```

- Run the old straight-road obstacle demo:

```bash
./scripts/scenario.sh run straight-obstacles
```

## Baseline
- Start stack and show smooth CAM updates in 3D UI.
- Metrics should show low CAM age and stale=no.
- When the lead sees an obstacle, CPM should appear and the UI should show V2V detection active.

## Mild profile
- Apply netem mild profile to V2V path.
- Show slight delay growth and occasional stale transitions.

## Severe profile
- Apply netem severe profile.
- Show obvious lag, reduced update responsiveness, and stale warnings.

## Recovery
- Clear netem profile.
- Show return to baseline responsiveness.
