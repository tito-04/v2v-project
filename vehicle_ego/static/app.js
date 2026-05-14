import * as THREE from "./vendor/three.module.js";
import { STLLoader } from "./vendor/stl-loader.js";

let current = {
  selfX: 20,
  selfY: 0,
  objects: {},
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
  const camera = new THREE.PerspectiveCamera(65, window.innerWidth / window.innerHeight, 0.1, 5000);
  renderer.setSize(window.innerWidth, window.innerHeight);
  document.getElementById("scene").appendChild(renderer.domElement);

  scene.fog = new THREE.FogExp2(0x05070c, 0.0009);

  const hemiLight = new THREE.HemisphereLight(0x88aaff, 0x1f1f1f, 1.1);
  scene.add(hemiLight);

  const dirLight = new THREE.DirectionalLight(0xffffff, 1.0);
  dirLight.position.set(40, 80, 30);
  scene.add(dirLight);

  const road = new THREE.Mesh(
    new THREE.PlaneGeometry(3000, 120),
    new THREE.MeshStandardMaterial({ color: 0x2f3136, roughness: 0.95, metalness: 0.02 })
  );
  road.rotation.x = -Math.PI / 2;
  road.position.y = -0.01;
  scene.add(road);

  const laneLine = new THREE.Mesh(
    new THREE.PlaneGeometry(3000, 2),
    new THREE.MeshStandardMaterial({
      map: createDashTexture(),
      transparent: true,
      roughness: 0.7,
      metalness: 0.1,
    })
  );
  laneLine.rotation.x = -Math.PI / 2;
  laneLine.position.y = 0.02;
  scene.add(laneLine);

  const laneOffsetZ = 20;

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
  fovMesh.rotation.x = -Math.PI / 2;
  fovMesh.position.y = 0.05;
  scene.add(fovMesh);

  camera.position.set(-30, 45, 70);
  camera.lookAt(0, 0, 0);

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

    egoCar.position.x += (current.selfX - egoCar.position.x) * 0.15;
    egoCar.position.z += (current.selfY - egoCar.position.z) * 0.15;
    leadCar.position.x += (leadX - leadCar.position.x) * 0.15;
    leadCar.position.z += (leadY - leadCar.position.z) * 0.15;
    leadCar.material.color.set(leadStale ? 0xff4d6d : 0xff9f1c);

    // Update FoV cone to follow lead car
    fovMesh.position.x = leadCar.position.x;
    fovMesh.position.z = leadCar.position.z;
    const hasCpm = Object.values(current.objects).some(o => o.source === "cpm" && !o.stale);
    fovMat.color.set(hasCpm ? 0x52b788 : 0x4cc9f0);
    fovMat.opacity = hasCpm ? 0.28 : 0.13;

    // Sync world objects (obstacles) as colored boxes by source
    const worldEntries = Object.entries(current.objects).filter(([, o]) => o.source !== "cam");
    const worldKeys = new Set(worldEntries.map(([k]) => k));
    for (const key of Object.keys(worldObjects)) {
      if (!worldKeys.has(key)) {
        scene.remove(worldObjects[key]);
        delete worldObjects[key];
      }
    }
    for (const [key, obj] of worldEntries) {
      if (!worldObjects[key]) {
        const geo = new THREE.BoxGeometry(8, 5, 5);
        const color = obj.source === "cpm" ? 0x52b788 : 0xe63946;
        const mat = new THREE.MeshStandardMaterial({ color, roughness: 0.5, metalness: 0.1 });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.position.y = 2.5;
        scene.add(mesh);
        worldObjects[key] = mesh;
      }
      // Update color dynamically (cpm=green, world=red, stale=grey)
      const targetColor = obj.stale ? 0x888888 : (obj.source === "cpm" ? 0x52b788 : 0xe63946);
      worldObjects[key].material.color.set(targetColor);
      worldObjects[key].position.x += ((obj.x ?? 0) - worldObjects[key].position.x) * 0.15;
      worldObjects[key].position.z += ((obj.y ?? 0) - worldObjects[key].position.z) * 0.15;
    }

    const centerX = (egoCar.position.x + leadCar.position.x) / 2;
    camera.position.x += (centerX - 30 - camera.position.x) * 0.03;
    camera.lookAt(centerX, 0, (egoCar.position.z + leadCar.position.z) / 2);

    renderMetrics();
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

  document.getElementById("metrics").textContent = [
    `CAM Rate: ${rateText} Hz  |  Age: ${ageText}  |  Latency: ${latencyText}`,
    `Stale: ${current.stale ? "yes" : "no"}`,
    objLines.length ? `Objects (${objLines.length}):\n${objLines.join("\n")}` : "Objects: none",
    cpmAlert,
  ].filter(Boolean).join("\n");
}

init3d();
