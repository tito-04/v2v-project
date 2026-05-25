import * as THREE from "./vendor/three.module.js";
import { STLLoader } from "./vendor/stl-loader.js";

let current = {
  self: { x: 10, y: 0, heading: 0, speed: 0, status: "loading" },
  world: { vehicles: {}, objects: {} },
  egoModel: { self: {}, objects: {}, last_action: {} },
  network: { cam: {}, cpm: {}, timeline: [] },
  metrics: { stale: true },
  scenario: null,
};

let vehicleModelGeometry = null;
let vehicleModelVersion = 0;

function applyState(payload) {
  if (!payload) {
    return;
  }
  current.self = payload.self ?? current.self;
  current.world = payload.world ?? { vehicles: {}, objects: {} };
  current.egoModel = payload.ego_model ?? {
    self: payload.self ?? current.self,
    objects: payload.objects ?? {},
    last_action: {},
  };
  current.network = payload.network ?? current.network;
  current.metrics = payload.metrics ?? current.metrics;
  current.scenario = payload.scenario ?? current.scenario;
}

function connectStateSource() {
  fetchState();
  if (typeof io !== "undefined") {
    const socket = io({ transports: ["polling"], upgrade: false });
    socket.on("state_update", applyState);
    socket.on("connect_error", () => {
      setInterval(fetchState, 1000);
    });
    return;
  }
  setInterval(fetchState, 1000);
}

async function fetchState() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (response.ok) {
      applyState(await response.json());
    }
  } catch {
    // Keep last known state on transient failures.
  }
}

function startFallbackUi(reason) {
  document.getElementById("scene").innerHTML = `
    <div class="fallback">
      <strong>3D renderer unavailable</strong>
      <span>${reason}. Showing live telemetry only.</span>
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

  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setScissorTest(true);
  document.getElementById("scene").appendChild(renderer.domElement);

  const worldView = createView("World Truth", 0x111a20);
  const modelView = createView("Ego World Model", 0x10141d);

  loadVehicleModel();

  connectStateSource();

  function animate() {
    requestAnimationFrame(animate);
    syncScenarioLayout(worldView);
    syncScenarioLayout(modelView);

    syncWorldView(worldView);
    syncModelView(modelView);
    renderMetrics();

    const width = renderer.domElement.clientWidth;
    const height = renderer.domElement.clientHeight;
    const half = Math.floor(width / 2);

    renderView(renderer, worldView, 0, 0, half, height);
    renderView(renderer, modelView, half, 0, width - half, height);
  }

  animate();

  window.addEventListener("resize", () => {
    renderer.setSize(window.innerWidth, window.innerHeight);
  });
}

function createView(title, bgColor) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(bgColor);

  const camera = new THREE.PerspectiveCamera(48, 1, 0.1, 5000);
  camera.position.set(200, 360, -130);
  camera.lookAt(200, 0, 20);

  const ambientLight = new THREE.AmbientLight(0xffffff, 2.5);
  scene.add(ambientLight);
  const sun = new THREE.DirectionalLight(0xfff2d4, 1.7);
  sun.position.set(-90, 260, -100);
  scene.add(sun);

  const layoutGroup = new THREE.Group();
  const actorGroup = new THREE.Group();
  scene.add(layoutGroup);
  scene.add(actorGroup);
  addGround(scene);
  return {
    title,
    scene,
    camera,
    layoutGroup,
    actorGroup,
    renderedScenarioName: null,
    meshes: {},
    fovs: {
      ego: buildFovCone(0x2ec4b6),
      lead: buildFovCone(0xffc857),
    },
  };
}

function renderView(renderer, view, x, y, width, height) {
  renderer.setViewport(x, y, width, height);
  renderer.setScissor(x, y, width, height);
  view.camera.aspect = Math.max(width / Math.max(height, 1), 0.1);
  view.camera.updateProjectionMatrix();
  renderer.render(view.scene, view.camera);
}

function syncWorldView(view) {
  const vehicles = current.world?.vehicles ?? {};
  const objects = current.world?.objects ?? {};
  const actors = {};

  for (const [key, vehicle] of Object.entries(vehicles)) {
    actors[`vehicle_${key}`] = { ...vehicle, renderRole: key };
  }
  for (const [key, obj] of Object.entries(objects)) {
    actors[`object_${key}`] = { ...obj, renderRole: "object" };
  }

  syncActors(view, actors, "world");
  updateFov(view, "ego", vehicles.ego, 0x2ec4b6, true);
  updateFov(view, "lead", vehicles.lead, 0xffc857, true);
  updateCamera(view, vehicles.ego ?? current.self);
}

function syncModelView(view) {
  const actors = {};
  const self = current.egoModel?.self ?? current.self;
  actors.vehicle_ego_model = { ...self, id: "ego", kind: "vehicle", renderRole: "ego" };
  for (const [key, obj] of Object.entries(current.egoModel?.objects ?? {})) {
    actors[`model_${key}`] = { ...obj, renderRole: "model" };
  }

  syncActors(view, actors, "model");
  updateFov(view, "ego", self, 0x52b788, true);
  updateFov(view, "lead", null, 0xffc857, false);
  updateEgoModelCamera(view, self);
}

function syncActors(view, actors, mode) {
  const keys = new Set(Object.keys(actors));
  for (const key of Object.keys(view.meshes)) {
    if (!keys.has(key)) {
      disposeObject(view.meshes[key]);
      view.actorGroup.remove(view.meshes[key]);
      delete view.meshes[key];
    }
  }

  for (const [key, actor] of Object.entries(actors)) {
    if (!view.meshes[key]) {
      view.meshes[key] = buildActor(actor);
      view.actorGroup.add(view.meshes[key]);
    }
    updateActorMesh(view.meshes[key], actor, mode);
  }
}

function buildActor(actor) {
  if (actor.kind === "vehicle") {
    return buildVehicle();
  }
  if (actor.kind === "pedestrian") {
    return buildPedestrian();
  }
  return buildObstacle();
}

function buildVehicle() {
  const group = new THREE.Group();
  const bodyMat = new THREE.MeshStandardMaterial({ color: 0x2ec4b6, roughness: 0.42, metalness: 0.18 });
  const glassMat = new THREE.MeshStandardMaterial({ color: 0x0c2731, roughness: 0.25, metalness: 0.1 });
  const brakeMat = new THREE.MeshBasicMaterial({ color: 0x440000 });

  const body = createVehicleBodyMesh(bodyMat);
  group.add(body);

  const cabin = new THREE.Mesh(new THREE.BoxGeometry(5.5, 2.1, 4.5), glassMat);
  cabin.position.set(-0.8, 3.5, 0);
  group.add(cabin);

  for (const z of [-2.1, 2.1]) {
    const light = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.55, 1.2), brakeMat);
    light.position.set(-6.15, 2.0, z);
    group.add(light);
  }

  group.userData.bodyMat = bodyMat;
  group.userData.bodyMesh = body;
  group.userData.vehicleModelVersion = vehicleModelGeometry ? vehicleModelVersion : 0;
  group.userData.brakeMat = brakeMat;
  return group;
}

function createVehicleBodyMesh(bodyMat) {
  if (vehicleModelGeometry) {
    const mesh = new THREE.Mesh(vehicleModelGeometry, bodyMat);
    mesh.position.y = 3;
    return mesh;
  }
  const body = new THREE.Mesh(new THREE.BoxGeometry(12, 3, 6), bodyMat);
  body.position.y = 4;
  return body;
}

function ensureVehicleModel(group) {
  if (!vehicleModelGeometry) {
    return;
  }
  if (group.userData.vehicleModelVersion === vehicleModelVersion) {
    return;
  }
  if (group.userData.bodyMesh) {
    group.remove(group.userData.bodyMesh);
    disposeObject(group.userData.bodyMesh);
  }
  const body = createVehicleBodyMesh(group.userData.bodyMat);
  group.add(body);
  group.userData.bodyMesh = body;
  group.userData.vehicleModelVersion = vehicleModelVersion;
}

function buildPedestrian() {
  const group = new THREE.Group();
  const bodyMat = new THREE.MeshStandardMaterial({ color: 0xff5d73, roughness: 0.7 });
  const headMat = new THREE.MeshStandardMaterial({ color: 0xffd6a5, roughness: 0.8 });
  const body = new THREE.Mesh(new THREE.CylinderGeometry(0.75, 0.9, 3.2, 16), bodyMat);
  body.position.y = 1.6;
  const head = new THREE.Mesh(new THREE.SphereGeometry(0.85, 16, 12), headMat);
  head.position.y = 3.65;
  group.add(body);
  group.add(head);
  group.userData.bodyMat = bodyMat;
  group.userData.brakeMat = null;
  return group;
}

function buildObstacle() {
  const mat = new THREE.MeshStandardMaterial({ color: 0xff6b35, roughness: 0.8 });
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(2.8, 2.8, 1.2, 24), mat);
  mesh.position.y = 0.6;
  mesh.userData.bodyMat = mat;
  return mesh;
}

function updateActorMesh(mesh, actor, mode) {
  if (actor.kind === "vehicle") {
    ensureVehicleModel(mesh);
  }
  const targetX = Number(actor.x ?? 0);
  const targetZ = Number(actor.y ?? 0);
  mesh.position.x += (targetX - mesh.position.x) * 0.18;
  mesh.position.z += (targetZ - mesh.position.z) * 0.18;
  mesh.rotation.y = Math.PI - THREE.MathUtils.degToRad(Number(actor.heading ?? 0)) + Math.PI;

  const color = actorColor(actor, mode);
  const mat = mesh.userData.bodyMat;
  if (mat) {
    mat.color.set(color);
    mat.opacity = actor.stale ? 0.42 : 1.0;
    mat.transparent = !!actor.stale;
  }
  if (mesh.userData.brakeMat) {
    const braking = actor.status === "braking" || actor.status === "stopped";
    mesh.userData.brakeMat.color.set(braking ? 0xff1f2d : 0x440000);
  }
}

function actorColor(actor, mode) {
  if (actor.stale) {
    return 0x8b95a1;
  }
  if (actor.status === "braking" || actor.status === "stopped") {
    return 0xff3b30;
  }
  if (actor.renderRole === "lead") {
    return 0xffb000;
  }
  if (actor.renderRole === "ego" || actor.id === "ego") {
    return 0x2ec4b6;
  }
  if (actor.source === "cpm") {
    return 0x00e5ff;
  }
  if (actor.source === "cam") {
    return 0xffb000;
  }
  if (actor.source === "direct") {
    return 0x52b788;
  }
  if (actor.kind === "pedestrian") {
    return mode === "model" ? 0x00e5ff : 0xff5d73;
  }
  return 0xff6b35;
}

function buildFovCone(color) {
  const range = 80;
  const halfDeg = 60;
  const shape = new THREE.Shape();
  shape.moveTo(0, 0);
  for (let i = 0; i <= 44; i += 1) {
    const a = THREE.MathUtils.degToRad(-halfDeg + (i / 44) * halfDeg * 2);
    shape.lineTo(Math.cos(a) * range, Math.sin(a) * range);
  }
  shape.lineTo(0, 0);
  const mesh = new THREE.Mesh(
    new THREE.ShapeGeometry(shape),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.16, side: THREE.DoubleSide })
  );
  const group = new THREE.Group();
  group.rotation.x = -Math.PI / 2;
  group.position.y = 0.08;
  group.add(mesh);
  group.userData.mesh = mesh;
  return group;
}

function updateFov(view, name, actor, color, visible) {
  const cone = view.fovs[name];
  if (!cone.parent) {
    view.scene.add(cone);
  }
  cone.visible = !!actor && visible;
  if (!actor) {
    return;
  }
  cone.position.x += (Number(actor.x ?? 0) - cone.position.x) * 0.25;
  cone.position.z += (Number(actor.y ?? 0) - cone.position.z) * 0.25;
  cone.userData.mesh.rotation.z = Math.PI - THREE.MathUtils.degToRad(Number(actor.heading ?? 0)) + Math.PI;
  cone.userData.mesh.material.color.set(color);
  cone.userData.mesh.material.opacity = actor.status === "braking" ? 0.28 : 0.15;
}

function updateCamera(view, focus) {
  const actor = focus ?? { x: 200, y: 0 };
  const cameraConfig = current.scenario?.layout?.camera ?? {};
  const height = cameraConfig.height ?? 350;
  const zOffset = cameraConfig.z_offset ?? -130;
  const lookAheadY = cameraConfig.look_ahead_y ?? 20;
  const x = Number(actor.x ?? 200);
  const z = Number(actor.y ?? 0);
  view.camera.position.x += (x - view.camera.position.x) * 0.035;
  view.camera.position.y = height;
  view.camera.position.z += (z + zOffset - view.camera.position.z) * 0.035;
  view.camera.lookAt(x, 0, z + lookAheadY);
}

function updateEgoModelCamera(view, focus) {
  const actor = focus ?? { x: 200, y: 0, heading: 0 };
  const x = Number(actor.x ?? 200);
  const z = Number(actor.y ?? 0);
  const heading = THREE.MathUtils.degToRad(Number(actor.heading ?? 0));
  const forwardX = Math.cos(heading);
  const forwardZ = Math.sin(heading);
  const chaseDistance = 26;
  const cameraHeight = 13;
  const lookAhead = 58;

  const targetX = x - forwardX * chaseDistance;
  const targetZ = z - forwardZ * chaseDistance;
  const lookX = x + forwardX * lookAhead;
  const lookZ = z + forwardZ * lookAhead;

  view.camera.position.x += (targetX - view.camera.position.x) * 0.12;
  view.camera.position.y += (cameraHeight - view.camera.position.y) * 0.12;
  view.camera.position.z += (targetZ - view.camera.position.z) * 0.12;
  view.camera.lookAt(lookX, 2.8, lookZ);
}

function syncScenarioLayout(view) {
  const scenarioName = current.scenario?.name ?? "__loading";
  if (view.renderedScenarioName === scenarioName) {
    return;
  }
  clearGroup(view.layoutGroup);
  const layout = current.scenario?.layout ?? fallbackLayout();
  for (const road of layout.roads ?? []) {
    addRoad(view.layoutGroup, road);
  }
  for (const occluder of layout.occluders ?? []) {
    addOccluder(view.layoutGroup, occluder);
  }
  addIntersectionDetails(view.layoutGroup, layout);
  view.renderedScenarioName = scenarioName;
}

function fallbackLayout() {
  return {
    type: "intersection",
    roads: [
      { orientation: "x", center: { x: 0, y: 0 }, length: 3000, width: 16 },
      { orientation: "y", center: { x: 200, y: 0 }, length: 3000, width: 16 },
    ],
    occluders: [{ type: "rect", x1: 188, y1: -34, x2: 196, y2: -4, height: 10 }],
  };
}

function addGround(scene) {
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(5000, 5000),
    new THREE.MeshStandardMaterial({ color: 0x17231b, roughness: 1.0 })
  );
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -0.06;
  scene.add(ground);
}

function addRoad(group, road) {
  const orientation = road.orientation ?? "x";
  const center = road.center ?? { x: 0, y: 0 };
  const length = road.length ?? 1000;
  const width = road.width ?? 16;
  const isNorthSouth = orientation === "y";

  const roadMesh = new THREE.Mesh(
    new THREE.PlaneGeometry(isNorthSouth ? width : length, isNorthSouth ? length : width),
    new THREE.MeshStandardMaterial({ color: 0x2c3035, roughness: 0.96, metalness: 0.02 })
  );
  roadMesh.rotation.x = -Math.PI / 2;
  roadMesh.position.set(center.x ?? 0, 0, center.y ?? 0);
  group.add(roadMesh);

  const laneLine = new THREE.Mesh(
    new THREE.PlaneGeometry(isNorthSouth ? 0.9 : length, isNorthSouth ? length : 0.9),
    new THREE.MeshBasicMaterial({ map: createDashTexture(isNorthSouth ? Math.PI / 2 : 0), transparent: true })
  );
  laneLine.rotation.x = -Math.PI / 2;
  laneLine.position.set(center.x ?? 0, 0.035, center.y ?? 0);
  group.add(laneLine);

  for (const side of [-1, 1]) {
    const edge = new THREE.Mesh(
      new THREE.PlaneGeometry(isNorthSouth ? 0.35 : length, isNorthSouth ? length : 0.35),
      new THREE.MeshBasicMaterial({ color: 0xf5f7fa, transparent: true, opacity: 0.5 })
    );
    edge.rotation.x = -Math.PI / 2;
    edge.position.x = (center.x ?? 0) + (isNorthSouth ? side * width / 2 : 0);
    edge.position.z = (center.y ?? 0) + (isNorthSouth ? 0 : side * width / 2);
    edge.position.y = 0.045;
    group.add(edge);
  }

  for (const side of [-1, 1]) {
    const sidewalk = new THREE.Mesh(
      new THREE.PlaneGeometry(isNorthSouth ? 5 : length, isNorthSouth ? length : 5),
      new THREE.MeshStandardMaterial({ color: 0x596066, roughness: 0.9 })
    );
    sidewalk.rotation.x = -Math.PI / 2;
    sidewalk.position.x = (center.x ?? 0) + (isNorthSouth ? side * (width / 2 + 3.2) : 0);
    sidewalk.position.z = (center.y ?? 0) + (isNorthSouth ? 0 : side * (width / 2 + 3.2));
    sidewalk.position.y = 0.01;
    group.add(sidewalk);
  }
}

function addIntersectionDetails(group, layout) {
  if (layout.type !== "intersection") {
    return;
  }
  const stripeMat = new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.82 });
  for (let i = 0; i < 7; i += 1) {
    const stripe = new THREE.Mesh(new THREE.PlaneGeometry(1.2, 10), stripeMat);
    stripe.rotation.x = -Math.PI / 2;
    stripe.position.set(196 + i * 2.2, 0.07, -14);
    group.add(stripe);
  }
  const riskMat = new THREE.MeshBasicMaterial({ color: 0xff3b30, transparent: true, opacity: 0.12, side: THREE.DoubleSide });
  const risk = new THREE.Mesh(new THREE.PlaneGeometry(18, 34), riskMat);
  risk.rotation.x = -Math.PI / 2;
  risk.position.set(204, 0.065, -14);
  group.add(risk);
}

function addOccluder(group, occluder) {
  if (occluder.type !== "rect") {
    return;
  }
  const x1 = Number(occluder.x1);
  const y1 = Number(occluder.y1);
  const x2 = Number(occluder.x2);
  const y2 = Number(occluder.y2);
  const height = Number(occluder.height ?? 8);

  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(Math.abs(x2 - x1), height, Math.abs(y2 - y1)),
    new THREE.MeshStandardMaterial({ color: 0x8a7358, roughness: 0.84, metalness: 0.08 })
  );
  mesh.position.set((x1 + x2) / 2, height / 2, (y1 + y2) / 2);
  group.add(mesh);

  const roof = new THREE.Mesh(
    new THREE.BoxGeometry(Math.abs(x2 - x1) + 1.5, 0.4, Math.abs(y2 - y1) + 1.5),
    new THREE.MeshBasicMaterial({ color: 0xd6b36a, transparent: true, opacity: 0.75 })
  );
  roof.position.set((x1 + x2) / 2, height + 0.22, (y1 + y2) / 2);
  group.add(roof);
}

function createDashTexture(rotationRad = 0) {
  const canvas = document.createElement("canvas");
  canvas.width = 256;
  canvas.height = 32;
  const ctx = canvas.getContext("2d");
  if (ctx) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#f4c542";
    ctx.fillRect(0, 10, 108, 12);
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.ClampToEdgeWrapping;
  texture.repeat.set(52, 1);
  texture.anisotropy = 4;
  if (rotationRad) {
    texture.center.set(0.5, 0.5);
    texture.rotation = rotationRad;
  }
  return texture;
}

function loadVehicleModel() {
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
      vehicleModelGeometry = geometry;
      vehicleModelVersion += 1;
    },
    undefined,
    (error) => {
      console.warn("Failed to load STL model", error);
    }
  );
}

function addRoadsideTrees(scene) {
  const trunkGeometry = new THREE.CylinderGeometry(0.55, 0.75, 5.5, 6);
  const trunkMaterial = new THREE.MeshStandardMaterial({ color: 0x6b4b2a, roughness: 0.9 });
  const canopyGeometry = new THREE.ConeGeometry(3.0, 7.5, 7);
  const canopyMaterial = new THREE.MeshStandardMaterial({ color: 0x2f6b3f, roughness: 0.85 });
  const totalTrees = 120;
  const trunks = new THREE.InstancedMesh(trunkGeometry, trunkMaterial, totalTrees);
  const canopies = new THREE.InstancedMesh(canopyGeometry, canopyMaterial, totalTrees);
  const dummy = new THREE.Object3D();
  let seed = 1337;
  const rand = () => {
    seed = (seed * 16807) % 2147483647;
    return (seed - 1) / 2147483646;
  };

  for (let i = 0; i < totalTrees; i += 1) {
    const row = Math.floor(i / 2);
    const side = i % 2 === 0 ? 1 : -1;
    const x = -760 + row * 26 + (rand() - 0.5) * 10;
    const z = side * (70 + rand() * 18);
    const scale = 0.72 + rand() * 0.6;

    dummy.position.set(x, 2.7 * scale, z);
    dummy.scale.set(scale, scale, scale);
    dummy.updateMatrix();
    trunks.setMatrixAt(i, dummy.matrix);

    dummy.position.set(x, 8.7 * scale, z);
    dummy.updateMatrix();
    canopies.setMatrixAt(i, dummy.matrix);
  }
  scene.add(trunks);
  scene.add(canopies);
}

function renderMetrics() {
  const scenarioName = current.scenario?.name ?? "loading";
  const worldVehicles = current.world?.vehicles ?? {};
  const modelObjects = current.egoModel?.objects ?? {};
  const cam = current.network?.cam ?? {};
  const cpm = current.network?.cpm ?? {};
  const action = current.egoModel?.last_action ?? {};
  const camLoss = numberText(cam.loss_percent, "%");
  const cpmLoss = numberText(cpm.loss_percent, "%");
  const camDelay = secondsText(cam.last_delay_sec);
  const cpmDelay = secondsText(cpm.last_delay_sec);
  const timeline = (current.network?.timeline ?? []).slice(-18).map((event) => {
    const label = event.type?.toUpperCase?.() ?? "?";
    if (event.status === "lost") return `${label}#${event.sequence}:lost`;
    if (event.status === "received") return `${label}#${event.sequence}:rx ${secondsText(event.delay_sec)}`;
    if (event.status === "sent") return `${label}#${event.sequence}:tx`;
    return `${label}:rx?`;
  });

  document.querySelector("#left-label .title").textContent = "World Truth";
  document.querySelector("#left-label .meta").textContent = worldVehicles.ego?.status
    ? `ego ${worldVehicles.ego.status} | lead ${worldVehicles.lead?.status ?? "n/a"}`
    : "waiting for world state";
  document.querySelector("#right-label .title").textContent = "Ego World Model";
  document.querySelector("#right-label .meta").textContent = `action ${action.action ?? "n/a"} | objects ${Object.keys(modelObjects).length}`;

  document.getElementById("metrics").textContent = [
    `Scenario: ${scenarioName}`,
    `CAM sent/rx/lost: ${cam.sent ?? 0}/${cam.received ?? 0}/${cam.lost ?? 0} | loss ${camLoss} | delay ${camDelay}`,
    `CPM sent/rx/lost: ${cpm.sent ?? 0}/${cpm.received ?? 0}/${cpm.lost ?? 0} | loss ${cpmLoss} | delay ${cpmDelay}`,
    `Ego action: ${action.action ?? "resume"} ${action.risk_object_id ? `risk=${action.risk_object_id}` : ""}`,
    timeline.length ? `Timeline: ${timeline.join("  ")}` : "Timeline: waiting for packets",
  ].join("\n");
}

function secondsText(value) {
  return value == null || !Number.isFinite(Number(value)) ? "n/a" : `${Number(value).toFixed(2)}s`;
}

function numberText(value, suffix) {
  return value == null || !Number.isFinite(Number(value)) ? `0.0${suffix}` : `${Number(value).toFixed(1)}${suffix}`;
}

function clearGroup(group) {
  while (group.children.length > 0) {
    const child = group.children[0];
    group.remove(child);
    disposeObject(child);
  }
}

function disposeObject(object) {
  object.traverse?.((node) => {
    node.geometry?.dispose?.();
    if (Array.isArray(node.material)) {
      node.material.forEach((material) => material.dispose?.());
    } else {
      node.material?.dispose?.();
    }
  });
}

init3d();
