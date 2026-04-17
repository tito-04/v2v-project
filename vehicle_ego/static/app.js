let current = {
  egoX: 20,
  leadX: 50,
  stale: true,
  camRateHz: 0,
  camAge: null,
  stationId: null,
};

function applyState(payload) {
  if (!payload || !payload.ego || !payload.lead || !payload.metrics) {
    return;
  }

  current.egoX = payload.ego.x;
  current.leadX = payload.lead.x;
  current.stale = payload.metrics.stale;
  current.camRateHz = payload.metrics.cam_rate_hz;
  current.camAge = payload.metrics.last_cam_age_sec;
  current.stationId = payload.lead.station_id;
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
  if (typeof THREE === "undefined") {
    startFallbackUi("Three.js failed to load");
    return;
  }

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
    new THREE.MeshStandardMaterial({ color: 0xd4b83d, roughness: 0.7, metalness: 0.1 })
  );
  laneLine.rotation.x = -Math.PI / 2;
  laneLine.position.y = 0.02;
  scene.add(laneLine);

  function buildVehicle(color) {
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(12, 4, 6),
      new THREE.MeshStandardMaterial({ color, roughness: 0.4, metalness: 0.25 })
    );
    body.position.y = 2.0;
    return body;
  }

  const egoCar = buildVehicle(0x2ec4b6);
  const leadCar = buildVehicle(0xff9f1c);
  scene.add(egoCar);
  scene.add(leadCar);

  camera.position.set(-30, 45, 70);
  camera.lookAt(0, 0, 0);

  connectStateSource();

  function animate() {
    requestAnimationFrame(animate);

    egoCar.position.x += (current.egoX - egoCar.position.x) * 0.15;
    leadCar.position.x += (current.leadX - leadCar.position.x) * 0.15;
    leadCar.material.color.set(current.stale ? 0xff4d6d : 0xff9f1c);

    const centerX = (egoCar.position.x + leadCar.position.x) / 2;
    camera.position.x += (centerX - 30 - camera.position.x) * 0.03;
    camera.lookAt(centerX, 0, 0);

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

function renderMetrics() {
  const ageText = current.camAge == null ? "n/a" : `${current.camAge.toFixed(2)}s`;
  const rateText = Number.isFinite(current.camRateHz) ? current.camRateHz.toFixed(2) : "0.00";

  document.getElementById("metrics").textContent = [
    `Lead Station ID: ${current.stationId ?? "n/a"}`,
    `CAM Rate: ${rateText} Hz`,
    `Last CAM Age: ${ageText}`,
    `Stale: ${current.stale ? "yes" : "no"}`,
  ].join("\n");
}

init3d();
