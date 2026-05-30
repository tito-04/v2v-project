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
- Start stack and show the split 3D UI.
- Left panel ("World Truth") should show ego, lead, the waiting pedestrian, the occluding building, crosswalk, and vehicle brake state.
- Right panel ("Ego World Model") should initially know less than the world truth, then gain CAM/CPM/direct detections as packets arrive or the pedestrian becomes visible.
- Metrics should show low CAM/CPM loss and low delay.
- In `intersection-occlusion`, the lead should detect the pedestrian first and send CPM. Ego should stop on the east-west segment before turning south; ego and lead should resume only after the pedestrian is done crossing.

## Mild profile
- Apply netem mild profile to the V2V ad-hoc interface (`eth1` by default).
- Show slight delay growth, pending packets, and occasional stale/ghost model objects.

## Severe profile
- Apply netem severe profile.
- Show obvious lag, higher loss, reduced ego model responsiveness, and delayed braking when the ego depends on CPM.

## Isolated impairment
- Apply `delay-only` to show the model lagging while packets still arrive.
- Apply `loss-only` to show missing packets, stale objects, and rising loss percentage without large delay.

## Recovery
- Clear netem profile.
- Show return to baseline responsiveness.
