import * as THREE from "./vendor/three.module.js";
import { STLLoader } from "./vendor/stl-loader.js";

let current = {
  selfX: 10,
  selfY: 0,
  selfHeading: 0,
  objects: {},  // All objects including obstacles, lead car, etc.
  stale: true,
  camRateHz: 0,
  camAge: null,
  camLatency: null,
};

function applyState(payload) {
  if (!payload || !payload.self || !payload.metrics) {
    return;
  }

  current.selfX = payload.self.x;
  current.selfY = payload.self.y ?? 0;
  current.selfHeading = payload.self.heading ?? 0;
  current.objects = payload.objects ?? {};
  current.stale = payload.metrics.stale;
  current.camRateHz = payload.metrics.cam_rate_hz;
  current.camAge = payload.metrics.last_cam_age_sec;
  current.camLatency = payload.metrics.last_cam_latency_sec ?? null;
}

function connectStateSource() {
  if (typeof io !== "undefined") {
    const socket = io({ transports: ["polling"], upgrade: false });
    socket.on("state_update", (payload) => {
      applyState(payload);
    });
    socket.on("connect_error", () => {
      // Polling fallback keeps UI alive if realtime transport fails.
      setInterval(fetchState, 1000);
    });
    return;
  }

  setInterval(fetchState, 1000);
}

async function fetchState() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) {
      return;
    }
    const payload = await response.json();
    applyState(payload);
  } catch {
    // Keep last known state on transient failures.
  }
}

function startFallbackUi(reason) {
  const sceneEl = document.getElementById("scene");
  sceneEl.innerHTML = `
    <div style="display:flex;align-items:center;justify-content:center;height:100%;color:#e8ecf2;">
      <div style="background:rgba(0,0,0,0.55);padding:18px 22px;border-radius:8px;max-width:640px;">
        <div style="font-weight:700;margin-bottom:8px;">3D renderer unavailable</div>
        <div style="font-size:14px;opacity:.9;">${reason}. Showing live telemetry only.</div>
      </div>
    </div>
  `;
  connectStateSource();
  setInterval(renderMetrics, 250);
}

function init3d() {
  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true });
  } catch {
    startFallbackUi("WebGL initialization failed");
    return;
  }

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0e1117);
  const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 5000);
  renderer.setSize(window.innerWidth, window.innerHeight);
  document.getElementById("scene").appendChild(renderer.domElement);

  // No fog — top-down view must see the full scene
  const ambientLight = new THREE.AmbientLight(0xffffff, 2.8);
  scene.add(ambientLight);
  const dirLight = new THREE.DirectionalLight(0xffffff, 1.5);
  dirLight.position.set(0, 500, 0); // straight down for top-down view
  scene.add(dirLight);

  // E-W road — 16 m wide (2 × 8 m lanes), centred at z=0
  const road = new THREE.Mesh(
    new THREE.PlaneGeometry(3000, 16),
    new THREE.MeshStandardMaterial({ color: 0x2f3136, roughness: 0.95, metalness: 0.02 })
  );
  road.rotation.x = -Math.PI / 2;
  road.position.y = -0.01;
  scene.add(road);

  // N-S road — 16 m wide (2 × 8 m lanes), centred at x=200 (intersection centre)
  const nsRoad = new THREE.Mesh(
    new THREE.PlaneGeometry(16, 3000),
    new THREE.MeshStandardMaterial({ color: 0x2f3136, roughness: 0.95, metalness: 0.02 })
  );
  nsRoad.rotation.x = -Math.PI / 2;
  nsRoad.position.x = 200;
  nsRoad.position.y = -0.005;
  scene.add(nsRoad);

  // Building (SW corner occluder): world x∈[188,196], y∈[-34,-4]
  // world.y maps DIRECTLY to Three.js z — no negation
  const bx1 = 188, by1 = -34, bx2 = 196, by2 = -4;
  const building = new THREE.Mesh(
    new THREE.BoxGeometry(bx2 - bx1, 10, Math.abs(by2 - by1)),
    new THREE.MeshStandardMaterial({ color: 0x6b5c4a, roughness: 0.9, metalness: 0.1 })
  );
  building.position.x = (bx1 + bx2) / 2; // 192
  building.position.y = 5;
  building.position.z = (by1 + by2) / 2;  // -19 — south of intersection (FIXED)
  scene.add(building);

  // E-W centre lane line (dashed yellow)
  const laneLine = new THREE.Mesh(
    new THREE.PlaneGeometry(3000, 1.2),
    new THREE.MeshBasicMaterial({ map: createDashTexture(), transparent: true })
  );
  laneLine.rotation.x = -Math.PI / 2;
  laneLine.position.y = 0.02;
  scene.add(laneLine);

  // N-S centre lane line (dashed yellow, running along Z)
  const nsLaneLine = new THREE.Mesh(
    new THREE.PlaneGeometry(1.2, 3000),
    new THREE.MeshBasicMaterial({ map: createDashTexture(), transparent: true })
  );
  nsLaneLine.rotation.x = -Math.PI / 2;
  nsLaneLine.position.x = 200;
  nsLaneLine.position.y = 0.02;
  scene.add(nsLaneLine);

  // Road edge lines (solid white) — E-W at z=±8
  for (const ez of [-8, 8]) {
    const e = new THREE.Mesh(
      new THREE.PlaneGeometry(3000, 0.4),
      new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.55 })
    );
    e.rotation.x = -Math.PI / 2;
    e.position.z = ez;
    e.position.y = 0.03;
    scene.add(e);
  }

  // Road edge lines (solid white) — N-S at x=192 and x=208
  for (const ex of [192, 208]) {
    const e = new THREE.Mesh(
      new THREE.PlaneGeometry(0.4, 3000),
      new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.55 })
    );
    e.rotation.x = -Math.PI / 2;
    e.position.x = ex;
    e.position.y = 0.03;
    scene.add(e);
  }

  addRoadsideTrees(scene);

  let egoCar = buildVehicle(0x2ec4b6);
  let leadCar = buildVehicle(0xff9f1c);
  scene.add(egoCar);
  scene.add(leadCar);

  const worldObjects = {}; // key → THREE.Mesh for world-source objects (obstacles)

  // FoV cone for lead car (80 m range, ±60° half-angle)
  const FOV_RANGE = 80;
  const FOV_HALF_DEG = 60;
  const fovShape = new THREE.Shape();
  fovShape.moveTo(0, 0);
  for (let i = 0; i <= 40; i++) {
    const a = THREE.MathUtils.degToRad(-FOV_HALF_DEG + (i / 40) * FOV_HALF_DEG * 2);
    fovShape.lineTo(Math.cos(a) * FOV_RANGE, Math.sin(a) * FOV_RANGE);
  }
  fovShape.lineTo(0, 0);
  const fovGeo = new THREE.ShapeGeometry(fovShape);
  const fovMat = new THREE.MeshBasicMaterial({ color: 0x4cc9f0, transparent: true, opacity: 0.13, side: THREE.DoubleSide });
  const fovMesh = new THREE.Mesh(fovGeo, fovMat);
  // Wrap in a Group so rotation.x (tilt flat) and heading (rotation.z in group space)
  // are independent — avoids Euler XYZ gimbal composition bug.
  const fovGroup = new THREE.Group();
  fovGroup.rotation.x = -Math.PI / 2; // lie flat on ground, permanent
  fovGroup.position.y = 0.05;
  fovGroup.add(fovMesh);
  scene.add(fovGroup);

  // FoV cone for ego car (same 80 m range, ±60° half-angle)
  const egoFovShape = new THREE.Shape();
  egoFovShape.moveTo(0, 0);
  for (let i = 0; i <= 40; i++) {
    const a = THREE.MathUtils.degToRad(-FOV_HALF_DEG + (i / 40) * FOV_HALF_DEG * 2);
    egoFovShape.lineTo(Math.cos(a) * FOV_RANGE, Math.sin(a) * FOV_RANGE);
  }
  egoFovShape.lineTo(0, 0);
  const egoFovGeo = new THREE.ShapeGeometry(egoFovShape);
  const egoFovMat = new THREE.MeshBasicMaterial({ color: 0x52b788, transparent: true, opacity: 0.13, side: THREE.DoubleSide });
  const egoFovMesh = new THREE.Mesh(egoFovGeo, egoFovMat);
  const egoFovGroup = new THREE.Group();
  egoFovGroup.rotation.x = -Math.PI / 2;
  egoFovGroup.position.y = 0.05;
  egoFovGroup.add(egoFovMesh);
  scene.add(egoFovGroup);

  camera.position.set(200, 350, -120);
  camera.lookAt(200, 0, 60);

  connectStateSource();

  loadVehicleModel((geometry) => {
    const nextEgo = buildVehicle(0x2ec4b6, geometry);
    const nextLead = buildVehicle(0xff9f1c, geometry);
    scene.add(nextEgo);
    scene.add(nextLead);
    scene.remove(egoCar);
    scene.remove(leadCar);
    egoCar = nextEgo;
    leadCar = nextLead;
  });

  function animate() {
    requestAnimationFrame(animate);

    const camObj = Object.values(current.objects).find(o => o.source === "cam");
    const leadX = camObj ? camObj.x : leadCar.position.x;
    const leadY = camObj ? (camObj.y ?? 0) : 0;
    const leadStale = camObj ? !!camObj.stale : true;
    const leadHeading = camObj ? (camObj.heading ?? 0) : 0;

    egoCar.position.x += (current.selfX - egoCar.position.x) * 0.15;
    egoCar.position.z += (current.selfY - egoCar.position.z) * 0.15;
    egoCar.rotation.y = Math.PI - THREE.MathUtils.degToRad(current.selfHeading ?? 0);
    leadCar.position.x += (leadX - leadCar.position.x) * 0.15;
    leadCar.position.z += (leadY - leadCar.position.z) * 0.15;
    leadCar.rotation.y = Math.PI - THREE.MathUtils.degToRad(leadHeading);
    leadCar.material.color.set(leadStale ? 0xff4d6d : 0xff9f1c);

    // Update FoV cone to follow lead car and rotate with heading
    // rotation.z in group-local space == rotation around world Y (heading) because
    // group.rotation.x = -PI/2 maps group's +Z to world +Y.
    fovGroup.position.x = leadCar.position.x;
    fovGroup.position.z = leadCar.position.z;
    fovMesh.rotation.z = Math.PI - THREE.MathUtils.degToRad(leadHeading);
    const hasCpm = Object.values(current.objects).some(o => o.source === "cpm" && !o.stale);
    fovMat.color.set(hasCpm ? 0x52b788 : 0x4cc9f0);
    fovMat.opacity = hasCpm ? 0.28 : 0.13;

    // Update FoV cone to follow ego car and rotate with heading
    egoFovGroup.position.x = egoCar.position.x;
    egoFovGroup.position.z = egoCar.position.z;
    egoFovMesh.rotation.z = Math.PI - THREE.MathUtils.degToRad(current.selfHeading ?? 0);
    // Cone color updates later based on detection of any object in FoV

    // Sync world objects (obstacles) — lead_car excluded (rendered as vehicle mesh)
    const worldEntries = Object.entries(current.objects).filter(([, o]) => o.source !== "cam" && o.source !== "lead_car");
    const worldKeys = new Set(worldEntries.map(([k]) => k));
    for (const key of Object.keys(worldObjects)) {
      if (!worldKeys.has(key)) {
        scene.remove(worldObjects[key]);
        delete worldObjects[key];
      }
    }
    for (const [key, obj] of worldEntries) {
      if (!worldObjects[key]) {
        // Flat disc — clearly visible from top-down
        const geo = new THREE.CylinderGeometry(3, 3, 1, 24);
        const mat = new THREE.MeshBasicMaterial({ color: 0xff2200 });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.position.y = 0.5;
        scene.add(mesh);
        worldObjects[key] = mesh;
      }

      // Detection-based colour (MeshBasicMaterial — always bright):
      //   grey   = stale
      //   red    = in world, nobody detects it (occluded by building)
      //   orange = lead FoV detected (CPM will be sent)
      //   green  = ego detects directly
      //   cyan   = CPM object forwarded by vanetza
      const inEgo  = obj.in_ego_fov  === true;
      const inLead = obj.in_lead_fov === true;
      let obsColor;
      if (obj.stale)                obsColor = 0x888888;
      else if (obj.source === "cpm") obsColor = 0x00eeff;
      else if (inEgo)               obsColor = 0x00ff44;
      else if (inLead)              obsColor = 0xffaa00;
      else                          obsColor = 0xff2200;
      worldObjects[key].material.color.set(obsColor);
      
      worldObjects[key].position.x += ((obj.x ?? 0) - worldObjects[key].position.x) * 0.15;
      worldObjects[key].position.z += ((obj.y ?? 0) - worldObjects[key].position.z) * 0.15;
    }

    // Update FoV cone color based on detection status (any object in FoV)
    const hasAnyDetection = Object.values(current.objects).some(o => o.in_ego_fov && (o.source === "world" || o.source === "lead_car"));
    egoFovMat.color.set(hasAnyDetection ? 0x52b788 : 0x2ec4b6);
    egoFovMat.opacity = hasAnyDetection ? 0.28 : 0.13;

    renderMetrics();

    const centerX = (egoCar.position.x + leadCar.position.x) / 2;
    const centerZ = (egoCar.position.z + leadCar.position.z) / 2;
    // Top-down camera: fixed height 350, stays south of midpoint so North is "up" on screen
    camera.position.x += (centerX        - camera.position.x) * 0.025;
    camera.position.y  = 350;
    camera.position.z += (centerZ - 130 - camera.position.z) * 0.025;
    camera.lookAt(centerX, 0, centerZ + 20);

    renderer.render(scene, camera);
  }

  animate();

  window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });
}

function buildVehicle(color, geometry) {
  const material = new THREE.MeshStandardMaterial({ color, roughness: 0.4, metalness: 0.25 });
  const body = geometry
    ? new THREE.Mesh(geometry, material)
    : new THREE.Mesh(new THREE.BoxGeometry(12, 4, 6), material);

  body.position.y = geometry ? 3.6 : 2.0;
  return body;
}

function loadVehicleModel(onLoad) {
  if (typeof STLLoader === "undefined") {
    return;
  }

  const loader = new STLLoader();
  loader.load(
    "/static/models/tesla.stl",
    (geometry) => {
      geometry.computeBoundingBox();
      geometry.center();

      const size = new THREE.Vector3();
      geometry.boundingBox.getSize(size);
      const maxSide = Math.max(size.x, size.z, 1e-6);
      const scale = 12 / maxSide;
      geometry.scale(scale, scale, scale);
      geometry.rotateX(-Math.PI / 2);
      geometry.rotateY(Math.PI / 2);

      onLoad(geometry);
    },
    undefined,
    (error) => {
      console.warn("Failed to load STL model", error);
    }
  );
}

function createDashTexture() {
  const canvas = document.createElement("canvas");
  canvas.width = 256;
  canvas.height = 32;

  const ctx = canvas.getContext("2d");
  if (ctx) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#d4b83d";
    ctx.fillRect(0, 10, 120, 12);
  }

  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.ClampToEdgeWrapping;
  texture.repeat.set(50, 1);
  texture.offset.set(0, 0);
  texture.anisotropy = 4;
  return texture;
}

function addRoadsideTrees(scene) {
  const trunkGeometry = new THREE.CylinderGeometry(0.6, 0.8, 6, 6);
  const trunkMaterial = new THREE.MeshStandardMaterial({ color: 0x6b4b2a, roughness: 0.9 });
  const canopyGeometry = new THREE.ConeGeometry(3.2, 8, 7);
  const canopyMaterial = new THREE.MeshStandardMaterial({ color: 0x2d6a34, roughness: 0.8 });

  const treesPerSide = 70;
  const totalTrees = treesPerSide * 2;
  const trunks = new THREE.InstancedMesh(trunkGeometry, trunkMaterial, totalTrees);
  const canopies = new THREE.InstancedMesh(canopyGeometry, canopyMaterial, totalTrees);

  const dummy = new THREE.Object3D();
  let seed = 12345;
  const rand = () => {
    seed = (seed * 16807) % 2147483647;
    return (seed - 1) / 2147483646;
  };

  for (let i = 0; i < totalTrees; i += 1) {
    const side = i % 2 === 0 ? 1 : -1;
    const row = Math.floor(i / 2);
    const x = -900 + row * 26 + (rand() - 0.5) * 10;
    const z = side * (78 + rand() * 12);
    const scale = 0.75 + rand() * 0.6;

    dummy.position.set(x, 3 * scale, z);
    dummy.scale.set(scale, scale, scale);
    dummy.updateMatrix();
    trunks.setMatrixAt(i, dummy.matrix);

    dummy.position.set(x, 10 * scale, z);
    dummy.scale.set(scale, scale, scale);
    dummy.updateMatrix();
    canopies.setMatrixAt(i, dummy.matrix);
  }

  scene.add(trunks);
  scene.add(canopies);
}

function renderMetrics() {
  const ageText = current.camAge == null ? "n/a" : `${current.camAge.toFixed(2)}s`;
  const rateText = Number.isFinite(current.camRateHz) ? current.camRateHz.toFixed(2) : "0.00";
  const latencyText = current.camLatency == null ? "n/a" : `${current.camLatency.toFixed(2)}s`;

  const objLines = Object.entries(current.objects).map(([key, obj]) => {
    const staleFlag = obj.stale ? " [STALE]" : "";
    const sid = obj.station_id != null ? ` sid=${obj.station_id}` : "";
    const dist = obj.distance_m != null ? ` dist=${obj.distance_m.toFixed(1)}m` : "";
    const from = obj.detected_by != null ? ` from=sid${obj.detected_by}` : "";
    return `  ${key}${sid}  x=${(obj.x ?? 0).toFixed(1)} y=${(obj.y ?? 0).toFixed(1)}${dist}${from}  [${obj.source ?? "?"}]${staleFlag}`;
  });
  const cpmAlert = Object.values(current.objects).some(o => o.source === "cpm" && !o.stale)
    ? "*** V2V DETECTION ACTIVE — obstacle received via CPM ***"
    : "";

  // Pedestrian detection debug lines (one per world-source obstacle)
  const pedDebugLines = Object.entries(current.objects)
    .filter(([, o]) => o.source === "world")
    .map(([k, o]) => {
      if (o.in_ego_fov === true)  return `[DEBUG] ${k}: DETETADO pelo ego (verde)`;
      if (o.in_lead_fov === true) return `[DEBUG] ${k}: DETETADO pelo lead (laranja) — aguarda CPM`;
      return `[DEBUG] ${k}: NAO DETETADO — ocluido pelo edificio (vermelho)`;
    });

  document.getElementById("metrics").textContent = [
    `CAM Rate: ${rateText} Hz  |  Age: ${ageText}  |  Latency: ${latencyText}`,
    `Stale: ${current.stale ? "yes" : "no"}`,
    objLines.length ? `Objects (${objLines.length}):\n${objLines.join("\n")}` : "Objects: none",
    cpmAlert,
    ...pedDebugLines,
  ].filter(Boolean).join("\n");
}

init3d();
