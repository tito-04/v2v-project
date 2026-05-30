# V2V Cooperative Perception Project Report

## Objectives of the work
The main objective of this project is to implement and demonstrate a Vehicle-to-Vehicle (V2V) Cooperative Perception simulation using standard ETSI messaging (CAM and CPM). It aims to show how sharing sensor data via V2V communication helps a vehicle safely navigate an occluded intersection where Vulnerable Road Users (VRUs), such as pedestrians, are blocked from the ego vehicle's direct line of sight by static obstacles. Furthermore, it aims to analyze how network degradation (packet loss and latency) impacts the safety and responsiveness of cooperative maneuvers.

## Architecture & Technologies
The system relies on a containerized, modular architecture orchestrated via Docker Compose, leveraging several key technologies:
- **Backend & Physics:** Native `Python` handles the backend logic, routing, vehicle decision loops, payload structuring, and the world simulation physics.
- **Communications:** Three distinct `MQTT Brokers` handle internal Pub/Sub messaging. Emulation of the ETSI ITS-G5 stack is handled by `vanetza-nap` (which encodes/decodes ASN.1 formatted standard CAM/CPM packets).
- **Web UI & 3D Rendering:** The dashboard is built using standard `HTML/JavaScript`. `Three.js` is employed to render the split-screen 3D interfaces, utilizing the `stl-loader.js` extension to import and display physical `.stl` 3D objects mapping cars and pedestrians.
- **Real-Time Data Sync:** `Flask-SocketIO` is used as a `WebSocket` server on the Python side, enabling real-time, low-latency streaming of the World Model object state directly into the browser's JavaScript memory.
- **Network Impairment Emulation:** `Bash scripts` (`netem_profiles.sh`) dynamically manipulate the Linux Traffic Control layer (`tc qdisc netem`) to forcefully inject deterministic packet loss and delays inside the Docker containers' `eth1` interfaces.

## Implementation

### 1. Vehicle Movement and Control Policies (`world_generator/simulation.py`)
Locomotion in the World Generator operates on a deterministic tick-based physics model.
- **Advancement loop:** Every cycle (`tick_seconds`), the engine calculates path updates. The simulation loops over components and advances their relative Cartesian positions based on a vector mapped by `target_speed`.
- **Stop-and-Go Decisions:** Whenever an obstacle or pedestrian blocks a predefined safety corridor, a proactive function (`apply_control`) triggers. It issues an `action = "stop"` which adds an override control block to the vehicle state, freezing its trajectory. Once the risk leaves the collision corridor, a `"resume"` command clears the restriction.

```python
def apply_control(self, vehicle_name: str, action: str, reason: str = "", risk_object_id: str | None = None, ...):
    normalized_action = action.lower()
    
    # Resolves the movement lock if the path clears
    if normalized_action in {"go", "resume", "clear"}:
        self._apply_resume_speed(vehicle_name)
        self.controls.pop(vehicle_name, None)
        return
        
    # Appends a control block forcing the vehicle to brake/stop
    self.controls[vehicle_name] = {
        "action": normalized_action,
        "reason": reason,
        "risk_object_id": risk_object_id,
        "expires_at": self.elapsed_seconds + max(ttl_seconds, self.tick_seconds),
    }
```

### 2. Localization and Standard Message Generation (`vehicle_lead/cpm_payload.py`)
To emulate standard hardware sensors and integrate with `vanetza-nap` ETSI structures, the simulation translates arbitrary localized Cartesian coordinates (meters) into global GPS coordinates (WGS84).
- **Geo-Localization Translation:** At a micro-scale, $1$ degree of latitude is constant (roughly $111,320$ meters). However, the absolute length of $1$ degree of longitude shrinks as we move towards the poles. Therefore, we must multiply the conversion constant by the cosine of the chosen latitude in radians.

```python
def meters_to_deg_lon(meters: float, latitude_deg: float) -> float:
    # Scale longitude denominator accurately by latitude angle
    denom = 111320.0 * math.cos(math.radians(latitude_deg))
    if abs(denom) < 1e-9:
        return 0.0
    return meters / denom

def meters_to_deg_lat(meters: float) -> float:
    # Latitude mapping is a linear constant equivalent
    return meters / 111320.0
```
- **CPM Mapping:** `build_cpm_payload()` uses these offsets (e.g., `lat = BASE_LAT + meters_to_deg_lat(y_meter)`) to encapsulate relative physical perceptions into standard ETSI `managementContainer` and `cpmContainers` arrays for a syntactically correct V2V broadcast over the ITS-G5 protocol.

### 3. Calculations and Risk Occlusion (`world_generator/risk.py`)
To properly emulate sensors, objects outside the immediate line of sight must be mathematically occluded.
- **Field of View (`is_in_fov`):** Calculates Euclidean distance (`math.hypot(dx, dy)`) and angular constraints (`math.atan2(dy, dx)`) to verify if an object falls within the sensor's physical cone bounds.
- **Ray-Casting Intersections (`segment_intersects_rect`):** To calculate visual blocking by a building, the script traces a parametric line segment from the ego vehicle ($p_1$) to the pedestrian ($p_2$), computing if it intersects the building's bounding box ($[x_{min}, x_{max}]$ and $[y_{min}, y_{max}]$). It utilizes $t_{enter}$ and $t_{exit}$ vectors.

```python
dx = p2[0] - p1[0]
dy = p2[1] - p1[1]
t_enter, t_exit = 0.0, 1.0

# Ray bounding-box testing for all 4 rectangle quadrant planes 
for p_val, q_val in ((-dx, p1[0] - xmin), (dx, xmax - p1[0]), (-dy, p1[1] - ymin), (dy, ymax - p1[1])):
    if p_val == 0.0:
        if q_val < 0.0:
            return False # The ray is parallel and outside the box
    elif p_val < 0.0:
        t_enter = max(t_enter, q_val / p_val)
    else:
        t_exit = min(t_exit, q_val / p_val)

return t_enter <= t_exit # True implies the building occludes the pedestrian
```

### 4. Data Fusion and Model State (`vehicle_ego/model_state.py`)
The Ego vehicle continuously receives conflicting or overlapping measurements (e.g., seeing a vehicle directly vs. receiving its coordinates simultaneously via CPM). 
- **Confidence and Staleness (`upsert_model_object`):** The system resolves identity duplication through a matrix of `source_priority`. A direct local sight has the highest priority and overrides delayed ad-hoc network telemetry. 
- Additionally, if packet-loss occurs, stale networked objects are eventually discarded inside the UI rendering engine if `updated_at` ages past critical thresholds.

```python
def upsert_model_object(objects: dict, key: str, item: dict, source_priority: int) -> None:
    existing = objects.get(key)
    
    # If the existing object is from a higher priority source (like direct sight) 
    # and isn't stale, ignore the incoming lower-priority V2V packet coordinates.
    if existing and int(existing.get("source_priority", 0)) > source_priority and not existing.get("stale", False):
        
        # Update the timestamp anyway so the object doesn't phase out
        existing["updated_at"] = max(float(existing.get("updated_at", 0.0)), float(item.get("updated_at", 0.0)))
        existing["stale"] = False
        return

    # Otherwise, the new item data overwrites the entry
    item["source_priority"] = source_priority
    objects[key] = item
```

### 5. Network Impairment Emulation (`scripts/netem_profiles.sh`)
To reliably test the V2V algorithms against realistic network degradation without requiring physical hardware interference, we manipulate the Linux Traffic Control subsystem (`tc`) directly within the Docker containers.
- **Traffic Control (`tc qdisc netem`):** The script executes commands directly inside the target container (e.g., `lead-vanetza`) on the explicit ad-hoc interface (`eth1`). The `netem` (Network Emulator) queuing discipline allows us to synthetically queue, delay, or drop packets as they traverse the stack.
- **Profiles:** Instead of random values, the script defines deterministic profiles. For example, a `mild` profile introduces an average delay of $300$ ms (with a $\pm 50$ ms random jitter) and forcefully drops $5\%$ of the packets. A `severe` profile spikes this up to $800 \pm 200$ ms with a $25\%$ packet loss, pushing the ego vehicle's V2V latency to unsafe reaction thresholds.

```bash
# Inside scripts/netem_profiles.sh
case "${profile}" in
  baseline)
    # Clears rules mapping default instant local behavior
    docker exec "${container}" tc qdisc del dev "${iface}" root 2>/dev/null || true
    ;;
  mild)
    docker exec "${container}" tc qdisc replace dev "${iface}" root netem delay 300ms 50ms loss 5%
    ;;
  severe)
    docker exec "${container}" tc qdisc replace dev "${iface}" root netem delay 800ms 200ms loss 25%
    ;;
esac
```

## Flow of the demo
1. **Scenario Bootstrapping:** Starts by loading the predefined `intersection-occlusion` route coordinates via the Bash CLI script (`./scripts/scenario.sh run intersection-occlusion`).
2. **Baseline UI Analysis:** The dashboard splits the screen: Left visualizes the absolute "World Truth" (all elements, including blocked paths) and Right shows the isolated "Ego World Model" (initially unaware of the occluded pedestrian).
3. **Cooperative Perception:** The Lead vehicle, stationed with clear forward sight, detects the pedestrian and broadcasts a geographically standardized CPM. 
4. **Collision Avoidance:** The Ego vehicle, completely occluded by the building, receives this remote CPM, resolving the position via `upsert_model_object`. Ego maps the pedestrian and safely applies emergency braking before entering the turn blindly.
5. **Network Impairment Profiles:** We simulate ad-hoc network degradation by executing `./scripts/netem_profiles.sh apply severe lead-vanetza eth1`.
6. **Isolated Issues & Delay Limits:** With network simulation active, the latency rises artificially. The metrics plot will observe delays bridging `500+ ms` packet times. When packet loss kicks in, ghost objects stay frozen, ultimately marking them as `stale` inside the model logic constraints.

## Results
- **Success of CPM:** Under ideal baseline conditions, the CPM significantly extends the Ego vehicle's awareness footprint beyond static blindspots. It provides a robust increase in Time To Collision (TTC) bounds.
- **Risks of Severe Network Degradation:** During our `netem` latency or packet-loss testing, CPMs arrive out-of-order or with large lag spikes. This invalidates the Ego World Model's real-time accuracy. High delays cause stale detections that translate directly into delayed emergency braking sequences, proving that while V2V perception prevents blind collisions, it introduces a hard dependency on ultra low-latency QoS loops to perform physical real-time safety.

## Links
- **Repository:** [https://github.com/tito-04/v2v-project]
- **Video Demo:** [ https://drive.google.com/file/d/1zgbZbJz5wUn4CB1vrx11TtiuRGrkwmtr/view?usp=sharing
                    https://drive.google.com/file/d/1LApid-zgD0b4B4RjaDSEJc8nilEduqlO/view?usp=drive_link]
