# Demo Flow (CAM + 3D + Network Impairment)

1. Baseline
- Start stack and show smooth CAM updates in 3D UI.
- Metrics should show low CAM age and stale=no.

2. Mild profile
- Apply netem mild profile to V2V path.
- Show slight delay growth and occasional stale transitions.

3. Severe profile
- Apply netem severe profile.
- Show obvious lag, reduced update responsiveness, and stale warnings.

4. Recovery
- Clear netem profile.
- Show return to baseline responsiveness.
